"""
Dieses Modul stellt eine robuste Infrastruktur für die Verwaltung,
Überwachung und Kommunikation von Multiprozess-Systemen bereit.

Enthalten sind Wrapper-Klassen für Events und Queues, 
spezialisierte Thread- und Prozessklassen sowie ein zentrales Prozess-Management.
Die Architektur ermöglicht die sichere und flexible Steuerung von Serverprozessen 
mit Hilfe von Events, Timern und interprozessualer Kommunikation.

Hauptbestandteile:
- MyEvent: Wrapper für multiprocessing.Event für einen einheitlichen Umgang mit Events.
- ServerEvents: Dataclass zur Bündelung aller relevanten Events für Serverprozesse.
- MyQueue, WriteOnlyQueue, ReadOnlyQueue: Wrapper für multiprocessing.Queue
    mit Zugriffsbeschränkungen.
- MyTimerThread: Thread-basierter Timer zur wiederholten Ausführung von Funktionen.
- user_input: Thread zur Verarbeitung von Benutzereingaben zur Laufzeitkontrolle.
- ServerProcess: Basisklasse für Prozesse mit serverseitiger Kommunikation (z.B. über SocketIO).
- ProcessManager: Zentrale Klasse zur Verwaltung, Überwachung, Steuerung und Terminierung 
    aller Prozesse im System.

Typische Anwendungsfälle:
- Steuerung und Überwachung von Serverprozessen in verteilten Systemen.
- Sichere Kommunikation und Synchronisation zwischen Prozessen und Threads.
- Einfache Erweiterbarkeit für weitere Prozess- und Eventtypen.

Abhängigkeiten:
- multiprocessing, threading, socketio, time, dataclasses, typing

Hinweis:
Dieses Modul ist für den Einsatz in Systemen mit hohen Anforderungen an Parallelität, 
Zuverlässigkeit und Wartbarkeit konzipiert.
"""
import multiprocessing
import time
from pathlib import Path
import logging
import cv2
import numpy as np
import datetime
import threading
from queue import Queue
import os

from models.tb_dataclasses import QueueMessage, QueueTestEvents, QueuesMembers, SocketEventsToBackend, QueueMessageHeader
from logging import Logger
from tb_events import IrEvents
from tb_queues import MainQueues, SocketQueues
#from tb_ir import app_ir, camera_control, frame_database (just for testing the system without camera)
from tb_ir import app_ir, frame_database

# Minimale Zustands/Hilfsobjekte, die von den Funktionen genutzt werden

class SystemMode:
    NORMAL = "Normal"
    TEST = "Test"
    FAULT = "Fault"

#mock camera
USE_MOCK_CAMERA = os.getenv("USE_MOCK_CAMERA", "0") == "1"

# Konfig-/Statuswerte 
START_THRESHOLD = 50.0
STOP_THRESHOLD = 45.0
POST_EVENT_DURATION = 5
MANUAL_RECORD_LIMIT = 600
recording_type = "EVENT"
mode = SystemMode.NORMAL

save_dir = Path("Output_data")
save_dir.mkdir(parents=True, exist_ok=True)

# Locks/Queues/Flags
camera_lock = threading.Lock()
db_lock = threading.Lock()
anomaly_queue = Queue()

anomaly_worker_thread = None
frame = None
temp = None
recording = False
anomaly_active = False
anomaly_thread = None
manual_record_thread = None
last_trigger_time = 0
last_test_time = 0
exit_flag = False
event_recording_enabled = True
manual_stop_flag = False
if USE_MOCK_CAMERA:
    from mock_camera import MockCameraController as CameraController
else:
    from tb_ir.camera_control import CameraController

# Kamera, Speicherung & Aufzeichnung 

def generate_error_image(width=160, height=120):
    img = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(img, "CAMERA ERROR", (10, height // 2), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 255), 2)
    return img


def save_frames_as_video(frames, filename, fps=32):
    if not frames:
        return
    height, width, _ = frames[0].shape
    out = cv2.VideoWriter(str(filename), cv2.VideoWriter_fourcc(*'MJPG'), fps, (width, height))
    for frame in frames:
        out.write(frame)
    out.release()

def record_video(cam, mode, duration=POST_EVENT_DURATION):
    global manual_stop_flag
    manual_stop_flag = False
    duration = min(duration, MANUAL_RECORD_LIMIT)  # Enforce limit
    try:
        with camera_lock:
            frame, temp = cam.get_frame()
    except Exception as e:
        logging.error(f"Failed to start recording: {e}")
        return
    if frame is None or frame.shape[0] == 0 or frame.shape[1] == 0:
        logging.error("Invalid frame received.")
        return

    filename = save_dir / f"thermal_video_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
    height, width = frame.shape[:2]
    writer = cv2.VideoWriter(str(filename), cv2.VideoWriter_fourcc(*'MJPG'), 32, (width, height))
    if not writer.isOpened():
        logging.error("Failed to open video writer.")
        return

    start_time = time.time()
    logging.info("Recording started.")

    while not manual_stop_flag:
        with camera_lock:
            frame, temp = cam.get_frame()
        if frame is not None:
            writer.write(frame)

        elapsed = time.time() - start_time
        if elapsed >= duration:  # Use passed duration
            logging.info(f"Stopped after {elapsed:.1f}s.")
            break
        if exit_flag:
            break
        time.sleep(0.01)

    writer.release()
    logging.info("Recording finished and saved.")

def stop_manual_recording_from_server():
    global manual_stop_flag, recording, manual_record_thread
    if recording and manual_record_thread and manual_record_thread.is_alive():
        manual_stop_flag = True
        manual_record_thread.join(timeout=2)
        recording = False
        logging.info("Manual recording stopped by server")
        return True
    logging.info("No manual recording active to stop")
    return False

def save_anomaly_video(cam, db_path, temp, timestamp, save_dir, duration=5, fps=32):
    global exit_flag
    try:
        # Retro-Frames holen
        with db_lock:
            db = frame_database.FrameDatabase(db_path)
            retrospective_frames = db.get_frames_from_last_n_seconds(seconds=10)
            db.close()

        # Post-Event-Frames sammeln
        post_frames = []
        start_time = time.time()
        while time.time() - start_time < duration:
            if exit_flag:
                break
            with camera_lock:
                frame, _ = cam.get_frame()
            if frame is not None:
                post_frames.append(frame)
            time.sleep(1 / fps)

        # Zusammenführen & speichern
        all_frames = retrospective_frames + post_frames
        filename = save_dir / f"merged_anomaly_temp{int(temp)}_{timestamp}.avi"
        save_frames_as_video(all_frames, filename, fps=fps)
        logging.info(f"Combined anomaly video saved as {filename}")
    except Exception as e:
        logging.error(f"Error in anomaly video thread: {e}")

def anomaly_worker():
    global recording
    while not anomaly_queue.empty():
        temp, timestamp = anomaly_queue.get()
        ts_str = timestamp.strftime("%Y%m%d_%H%M%S")
        logging.info(f"Processing anomaly event at {temp:.2f}°C ({ts_str})")
        recording = True
        save_anomaly_video(cam, "prozess.db", temp, ts_str, save_dir, POST_EVENT_DURATION)
        recording = False

# IR-Prozess

class Tb_IrProcess(multiprocessing.Process):
    """
    Basisklasse für alle Prozesse im System, die mit dem Server kommunizieren.
    """
    def __init__(self, name: str, logger: Logger, events: IrEvents, main_queues : MainQueues, socket_queues : SocketQueues) -> None:
        """
        Initialisiert den ServerProcess.

        Args:
            logger (Any): Logger-Objekt.
            name (str): Name des Prozesses.
            url (str): Server-URL.
            events (ServerEvents): Events zur Steuerung.

        Raises:
            ValueError: Bei ungültigen Parametern.
        """
        # Name prüfen: String, min. 3, max. 50 Zeichen
        if not (3 <= len(name) <= 50):
            raise ValueError("Name muss zwischen 3 und 50 Zeichen lang sein.")
        super().__init__(name=name)
        self.logger = logger
        self.events = events
        
        self.main_queues : MainQueues = main_queues
        self.socket_queues : SocketQueues = socket_queues
        self.logger.debug(f"{self.__class__.__name__} - {self.name} init")

    def shutdown(self):
        """
        Setzt das Shutdown-Event, um den Prozess zu beenden.
        """
        if not self.events.shutdown.is_set():
            self.events.shutdown.set()
            self.logger.debug(f"{self.__class__.__name__} - {self.name} called shutdown")
   
    def run(self) -> None:
        global anomaly_worker_thread 
        global mode, frame, temp, recording, anomaly_active
        global anomaly_thread, manual_record_thread
        global last_trigger_time, last_test_time, exit_flag, event_recording_enabled
        global cam, db  # anomaly_worker auf dieselbe Instanz zugreift

        self.logger.debug(f"{self.__class__.__name__} - {self.name} running")

        app_ir.load_config()  
        msg_out : QueueMessage
        RETRIGGER_COOLDOWN = 15
        TEST_TIMEOUT = 180
        last_trigger_time = 0
        last_test_time = time.time()

        init : bool = False
        cam = None
        db = None
        
        while not self.events.shutdown.is_set():
            if not init :
                try:
                    cam = CameraController()
                    db = frame_database.FrameDatabase("prozess.db")
                    init = True
                except Exception as e: 
                    self.logger.error(f"Failed to initialize camera or DB: {e}")
                    event_recording_enabled = False
                    cam = None 
                    init = False
            else:    
                msg_in = self.main_queues.ir.get()
                if not msg_in is None: 
                    if msg_in.header.source is QueuesMembers.MAIN and msg_in.header.dest is QueuesMembers.IR and msg_in.header.event is QueueTestEvents.REQ_FROM_MAIN_TO_IR:
                        if not self.queue_test_send_ack(msg=msg_in):
                            self.events.error.set()
                    else:
                        msg_out = app_ir.ir_command_handler(msg_in=msg_in)
                        if not self.queue_send_to_server(msg = msg_out):
                            self.events.error.set()
                ### Kamera hier einfügen
                
                #  Frame und Temperatur holen
                try:
                    with camera_lock:
                        frame, temp = cam.get_frame()
                    if frame is None:
                        # Kamera lieferte nichts -> Platzhalterbild und Temp ungültig
                        frame = generate_error_image()
                        temp = None
                except Exception as e:
                    self.logger.error(f"Camera error: {e}")
                    frame = generate_error_image()
                    temp = None

                # Frame in DB puffern (für retrospektive Ereignisvideos)
                if frame is not None and db is not None:
                    try:
                        with db_lock:
                            db.insert_frame(frame)
                    except Exception as e:
                        self.logger.warning(f"DB insert error: {e}")

                # Testmodus-Timeout (180 s) -> zurück in Normal
                if mode == SystemMode.TEST and (time.time() - last_test_time) > TEST_TIMEOUT:
                    self.logger.info("Test mode timeout -> NORMAL")
                    mode = SystemMode.NORMAL

                #  Ereignislogik im NORMAL-Modus (IO automatisch)
                if mode == SystemMode.NORMAL and temp is not None:
                    if temp > START_THRESHOLD and not anomaly_active:
                        #  Ereignis in Queue (startet Video-Worker)
                        anomaly_queue.put((temp, datetime.datetime.now()))

                        #  IO ansteuern (falls in app_ir vorhanden; sonst auskommentieren)
                        try:
                            app_ir.retry_io_action(app_ir.trigger_hupe, "HUPE Trigger")
                        except Exception:
                            pass
                        try:
                            app_ir.retry_io_action(app_ir.trigger_blitz, "BLITZ Trigger")
                        except Exception:
                            pass
                        try:
                            app_ir.retry_io_action(lambda: app_ir.set_relais_state(True), "Set RELAIS ON")
                        except Exception:
                            pass

                        anomaly_active = True
                        last_trigger_time = time.time()

                    elif temp < STOP_THRESHOLD and not recording:
                        # Anomalie „entschärfen“, sobald wieder unter Stop-Schwelle
                        anomaly_active = False

                #  Ereignislogik im TEST-Modus (freigestellte Aufzeichnungsart)
                if mode == SystemMode.TEST and temp is not None:
                    if (recording_type == "EVENT" and temp > START_THRESHOLD and not anomaly_active):
                        anomaly_queue.put((temp, datetime.datetime.now()))
                        anomaly_active = True
                    elif temp < STOP_THRESHOLD and not recording:
                        anomaly_active = False

                #  Anomalie-Worker starten (holt Retro-Frames + sammelt Post-Frames)
                if (not recording) and (not anomaly_queue.empty()) and \
                   (anomaly_worker_thread is None or not anomaly_worker_thread.is_alive()):
                    anomaly_worker_thread = threading.Thread(target=anomaly_worker, daemon=True)
                    anomaly_worker_thread.start()

                #  Aufnahme-Ende housekeeping (Relais zurücksetzen falls nötig)
                if recording and manual_record_thread and not manual_record_thread.is_alive():
                    try:
                        # Wenn du die IO in app_ir gekapselt hast:
                        try:
                            app_ir.set_relais_state(False)
                        except Exception:
                            pass
                    except Exception as e:
                        self.logger.warning(f"Reset relais failed: {e}")
                    recording = False
                    manual_record_thread = None
                ###
                self.events.heartbeat.set()
            time.sleep(1)

        if self.events.shutdown.is_set():
            self.events.shutdown.clear()
        
        if cam and hasattr(cam, "shutdown"):
            cam.shutdown()
        if db:
            db.close()
        time.sleep(1)

    def _prepare_server_msg(self, event : SocketEventsToBackend, payload : dict = {}) -> QueueMessage:
        header : QueueMessageHeader = QueueMessageHeader(
            source=QueuesMembers.IR, 
            dest = QueuesMembers.SERVER, 
            event =event,
            id="",
            user="",
            timestamp=time.time())
        return QueueMessage(header = header, payload=payload)    
    
    def _prepare_queue_test_msg(self, event : QueueTestEvents, payload : dict = {}) -> QueueMessage:
        header : QueueMessageHeader = QueueMessageHeader(
            source=QueuesMembers.IR, 
            dest = QueuesMembers.MAIN, 
            event =event,
            id="",
            user="",
            timestamp=time.time())
        return QueueMessage(header = header, payload=payload)    
        
    def queue_test_send_ack(self, msg : QueueMessage) -> bool:
        status : bool = False
        try:
            if msg.header.event == QueueTestEvents.REQ_FROM_MAIN_TO_IR:
                queue_test_msg_ack : QueueMessage = self._prepare_queue_test_msg(event=QueueTestEvents.ACK_FROM_IR_TO_MAIN)
                if not self.main_queues.main.put(item=queue_test_msg_ack):
                    status = False
                else:
                    status = True
        except Exception:
            status = False
        return status
    
    def queue_send_to_server(self, msg : QueueMessage) -> bool:
        status : bool = False
        try:
            if not self.main_queues.server.put(item=msg):
                status = False
            else:
                status = True
        except Exception:
            status = False
        return status
