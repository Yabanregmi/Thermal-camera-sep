from re import M
import cv2
import numpy as np
import datetime
from pathlib import Path
import time
import threading
import json
import logging
import csv
from queue import Queue
import socketio
from tb_ir import frame_database, camera_control
from models.tb_dataclasses import QueueMessage, SocketEventsFromBackend, SocketEventsToBackend, QueueMessageHeader
from tb_ir_process import QueuesMembers
from collections import deque

MANUAL_RECORD_LIMIT = 600  # Default maximum duration for manual recording

MIN_RECORD_DURATION = 10  # Minimum record time (seconds)
PRE_EVENT_DURATION = 10   # Pre-event frames for anomaly video

START_THRESHOLD = 50.0  # Default start threshold (°C)
STOP_THRESHOLD = 45.0   # Default stop thre shold (°C)

TEMP_THRESHOLD = 50.0
POST_EVENT_DURATION = 5
CONFIG_FILE = "config.json"
LOG_FILE = "system.log"
FRAME_LOG_FILE = "frame_log.csv"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)

if not Path(FRAME_LOG_FILE).exists():
    with open(FRAME_LOG_FILE, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp", "mode", "temperature", "recording"])

class SystemMode:
    NORMAL = "Normal"
    TEST = "Test"
    FAULT = "Fault"
    
# Global Variables 
cam = None
db = None
mode = SystemMode.NORMAL
frame = None
temp = None
recording = False
anomaly_thread = None
manual_record_thread = None
save_dir = Path("Output_data")
save_dir.mkdir(exist_ok=True)
camera_lock = threading.Lock()
db_lock = threading.Lock()
last_trigger_time = 0
last_test_time = time.time()
exit_flag = False
relais_frozen = False
anomaly_active = False  # To prevent duplicate anomaly videos
manual_stop_flag = False  # Flag to stop manual recording
event_recording_enabled = True  # Controls if event-triggered recording is active
MANUAL_RECORD_LIMIT = 600  # Default manual recording limit (in seconds)
anomaly_queue = Queue()
anomaly_worker_thread = None
anomaly_active = False  # tracks ongoing anomaly
sio = socketio.Client()
logger = logging.getLogger("IR_App")
events_ir = None  # Placeholder for IR app events(from IRAppProcess)
queue_ir = None  # Placeholder for IR app queue(from IRAppProcess)

# Error history for user notification
ERROR_HISTORY_LIMIT = 50  # Keep last 50 errors
error_history = deque(maxlen=ERROR_HISTORY_LIMIT)

def log_error_to_user(message):
    """
    Logs an error both to the system log and to the user-visible error queue.
    """
    logging.error(message)
    error_history.append({
        "timestamp": datetime.datetime.now().isoformat(),
        "message": message
    })


# Config Load/Save 
def load_config():
    global START_THRESHOLD, STOP_THRESHOLD, save_dir, POST_EVENT_DURATION
    global MIN_RECORD_DURATION, PRE_EVENT_DURATION, MANUAL_RECORD_LIMIT
    global event_recording_enabled, mode, recording_type

    config = {}
    if Path(CONFIG_FILE).exists():
        with open(CONFIG_FILE, "r") as f:
            config = json.load(f)

    START_THRESHOLD = config.get("start_threshold", START_THRESHOLD)
    STOP_THRESHOLD = config.get("stop_threshold", STOP_THRESHOLD)
    MIN_RECORD_DURATION = config.get("min_record_duration", MIN_RECORD_DURATION)
    PRE_EVENT_DURATION = config.get("pre_event_duration", PRE_EVENT_DURATION)
    POST_EVENT_DURATION = config.get("duration", POST_EVENT_DURATION)
    MANUAL_RECORD_LIMIT = config.get("manual_record_limit", MANUAL_RECORD_LIMIT)
    save_dir = Path(config.get("save_dir", str(save_dir)))
    save_dir.mkdir(parents=True, exist_ok=True)


    event_recording_enabled = config.get("event_recording_enabled", True)
    mode = config.get("mode", SystemMode.NORMAL)
    recording_type = config.get("recording_type", "EVENT")

    logging.info("Config loaded.")



def save_config():
    config = {
        "start_threshold": START_THRESHOLD,
        "stop_threshold": STOP_THRESHOLD,
        "min_record_duration": MIN_RECORD_DURATION,
        "pre_event_duration": PRE_EVENT_DURATION,
        "save_dir": str(save_dir),
        "duration": POST_EVENT_DURATION,
        "recording_type": recording_type,
        "manual_record_limit": MANUAL_RECORD_LIMIT,
        "event_recording_enabled": event_recording_enabled,
        "mode": mode  # Save current mode
    }
    with open(CONFIG_FILE, "w") as f:
        json.dump(config, f)
        logging.info("Config saved.")

def log_config_change(setting_name, old_value, new_value, user="server"):
    """
    Logs manual changes to configuration settings persistently.
    """
    logging.info(f"[CONFIG CHANGE] {setting_name} changed from {old_value} to {new_value} (by {user})")


def set_start_threshold(value, user="server"):
    global START_THRESHOLD
    old = START_THRESHOLD
    START_THRESHOLD = max(0, min(250, value))
    log_config_change("START_THRESHOLD", old, START_THRESHOLD, user)
    save_config()


def set_stop_threshold(value, user="server"):
    global STOP_THRESHOLD
    old = STOP_THRESHOLD
    STOP_THRESHOLD = max(0, min(250, value))
    log_config_change("STOP_THRESHOLD", old, STOP_THRESHOLD, user)
    save_config()


def set_threshold(value, user="server"):
    global TEMP_THRESHOLD
    old = TEMP_THRESHOLD
    TEMP_THRESHOLD = max(0, min(250, value))
    log_config_change("TEMP_THRESHOLD", old, TEMP_THRESHOLD, user)
    save_config()


def set_duration(seconds, user="server"):
    global POST_EVENT_DURATION
    old = POST_EVENT_DURATION
    POST_EVENT_DURATION = min(max(0, seconds), 180)
    log_config_change("POST_EVENT_DURATION", old, POST_EVENT_DURATION, user)
    save_config()


def set_manual_record_limit(seconds, user="server"):
    global MANUAL_RECORD_LIMIT
    old = MANUAL_RECORD_LIMIT
    MANUAL_RECORD_LIMIT = min(max(1, seconds), 3600)
    log_config_change("MANUAL_RECORD_LIMIT", old, MANUAL_RECORD_LIMIT, user)
    save_config()


def set_save_dir(path_str, user="server"):
    global save_dir
    old = str(save_dir)
    save_dir = Path(path_str)
    save_dir.mkdir(exist_ok=True)
    log_config_change("SAVE_DIR", old, str(save_dir), user)
    save_config()

def enable_event_recording(user="server"):
    global event_recording_enabled
    old = event_recording_enabled
    event_recording_enabled = True
    log_config_change("EVENT_RECORDING_ENABLED", old, event_recording_enabled, user)
    logging.info("Event recording ENABLED by server")
    return True


def disable_event_recording(user="server"):
    global event_recording_enabled
    old = event_recording_enabled
    event_recording_enabled = False
    log_config_change("EVENT_RECORDING_ENABLED", old, event_recording_enabled, user)
    logging.info("Event recording DISABLED by server")
    return True


def start_event_recording_from_server():  # backend callable
    return enable_event_recording()

def stop_event_recording_from_server():  # backend callable
    return disable_event_recording()

# Core Functions 
def generate_error_image(width=160, height=120):
    img = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(img, "CAMERA ERROR", (10, height // 2), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 255), 2)
    return img

def screenshot(frame_copy):#backend callable
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = save_dir / f"screenshot_{timestamp}.png"
    cv2.imwrite(str(filename), frame_copy)
    logging.info(f"Screenshot saved as {filename}")

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
        log_error_to_user(f"Failed to start recording: {e}")

        return
    if frame is None or frame.shape[0] == 0 or frame.shape[1] == 0:
        log_error_to_user("Invalid frame received.")

        return

    filename = save_dir / f"thermal_video_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
    height, width = frame.shape[:2]
    writer = cv2.VideoWriter(str(filename), cv2.VideoWriter_fourcc(*'MJPG'), 32, (width, height))
    if not writer.isOpened():
        log_error_to_user("Failed to open video writer.")
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
        with db_lock:
            db : frame_database.FrameDatabase = frame_database.FrameDatabase(db_path)
            retrospective_frames = db.get_frames_from_last_n_seconds(seconds=10)
            db.close()

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

        all_frames = retrospective_frames + post_frames
        filename = save_dir / f"merged_anomaly_temp{int(temp)}_{timestamp}.avi"
        save_frames_as_video(all_frames, filename, fps=fps)
        logging.info(f"Combined anomaly video saved as {filename}")
    except Exception as e:
        log_error_to_user(f"Error in anomaly video thread: {e}")


def display(frame, temp, mode, recording):
    annotated = np.ascontiguousarray(frame.copy())
    cv2.putText(annotated, f"Mode: {mode}", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

    

    rec_status = "RECORDING" if recording else "IDLE"
    rec_color = (0, 0, 255) if recording else (0, 255, 0)
    cv2.putText(annotated, f"Recording: {rec_status}", (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, rec_color, 2)

    if temp is not None:
        label = f"{temp:.2f}\u00B0C"
        temp_color = (0, 0, 255) if temp > TEMP_THRESHOLD else (255, 255, 0)
        cv2.putText(annotated, label, (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, temp_color, 2)

    else:
        cv2.putText(annotated, "Temp: N/A", (10, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        
    # Only show recording type when recording is active
    if recording:
        rec_type = "Manual" if recording_type == "Manual" else "Event"
        cv2.putText(annotated, f"Type: {rec_type}", (10, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    return annotated



# Backend Callable Functions 
def set_mode(new_mode, user="server"):
    global mode, last_test_time
    if new_mode in [SystemMode.NORMAL, SystemMode.TEST, SystemMode.FAULT]:
        old_mode = mode
        mode = new_mode
        if mode == SystemMode.TEST:
            last_test_time = time.time()
        log_config_change("SystemMode", old_mode, mode, user)
        save_config()
        return True
    logging.warning(f"Invalid mode requested: {new_mode}")
    return False

def get_system_status():
    last_error = error_history[-1] if error_history else None
    return {
        "mode": mode,
        "threshold": TEMP_THRESHOLD,
        "recording": recording,
        "last_trigger_time": last_trigger_time,
        "event_recording_enabled": event_recording_enabled,
        "start_threshold": START_THRESHOLD,
        "stop_threshold": STOP_THRESHOLD,
        "duration": POST_EVENT_DURATION,
        "save_dir": str(save_dir),
        "last_error": last_error
    }

def anomaly_worker():
    global recording
    while not anomaly_queue.empty():
        temp, timestamp = anomaly_queue.get()
        ts_str = timestamp.strftime("%Y%m%d_%H%M%S")
        logging.info(f"Processing anomaly event at {temp:.2f}°C ({ts_str})")
        recording = True
        save_anomaly_video(cam, "frame_store.db", temp, ts_str, save_dir, POST_EVENT_DURATION)
        recording = False


def trigger_mock_anomaly_from_server():
    """
    Allows the server to simulate an anomaly in mock mode.
    Equivalent to pressing key 'a'.
    """
    if USE_MOCK_CAMERA:
        cam.trigger_anomaly()
        logging.info("Anomaly triggered in mock camera (via server)")
        return True
    else:
        logging.warning("Anomaly trigger ignored: Not using mock camera")
        return False



def get_recent_errors(limit=10):  # backend callable
    """
    Returns the last `limit` errors for the server or UI.
    """
    return list(error_history)[-limit:]


def start_manual_recording_from_server():
    global manual_record_thread, recording
    if not recording:
        duration = min(POST_EVENT_DURATION, MANUAL_RECORD_LIMIT)
        manual_record_thread = threading.Thread(
            target=record_video, args=(cam, mode, duration)
        )
        manual_record_thread.start()
        recording = True
        logging.info(f"Manual recording triggered by server (limit {duration}s)")
        return True
    logging.info("Manual recording already in progress")
    return False


def trigger_mock_anomaly():
    if USE_MOCK_CAMERA:
        cam.trigger_anomaly()
        logging.info("Anomaly triggered in mock camera (by server)")
        return True
    logging.warning("Trigger ignored: Not using mock camera")
    return False

def trigger_hupe():
    try:
        # Actual hardware control code
        logging.info("HUPE TRIGGERED")
        return True
    except Exception as e:
        log_error_to_user(f"HUPE trigger failed: {e}")
        return False

def trigger_blitz():
    try:
        # Actual hardware control code
        logging.info("BLITZ TRIGGERED")
        return True
    except Exception as e:
        log_error_to_user(f"BLITZ trigger failed: {e}")
        return False

def set_relais_state(state):
    global relais_frozen
    try:
        if relais_frozen:
            logging.info("Attempted to change relais, but relais are frozen.")
            return False
        # Actual hardware code here
        logging.info(f"RELAIS SET TO: {'ON' if state else 'OFF'}")
        return True
    except Exception as e:
        log_error_to_user(f"Relais state change failed: {e}")
        return False



def freeze_relais():  # backend callable
    global relais_frozen
    try:
        relais_frozen = True
        logging.info("RELAIS STATE FROZEN")
    except Exception as e:
        log_error_to_user(f"Failed to freeze relais: {e}")

def unfreeze_relais():  # backend callable
    global relais_frozen
    try:
        relais_frozen = False
        logging.info("RELAIS STATE UNFROZEN")
    except Exception as e:
        log_error_to_user(f"Failed to unfreeze relais: {e}")

def trigger_hupe_from_server():#backend callable
    if mode == SystemMode.TEST:
        trigger_hupe()
        return True
    return False

def trigger_blitz_from_server():#backend callable
    if mode == SystemMode.TEST:
        trigger_blitz()
        return True
    return False

def set_relais_state_from_server(state: bool):#backend callable
    if mode == SystemMode.TEST:
        set_relais_state(state)
        return True
    return False

def freeze_relais_from_server():#backend callable
    if mode == SystemMode.TEST:
        freeze_relais()
        return True
    return False

def take_screenshot_from_server():#backend callable
    global frame
    if frame is not None:
        threading.Thread(target=screenshot, args=(frame.copy(),)).start()
        return True
    return False
def retry_io_action(action, action_name="IO Action", retries=3, delay=0.5):
    """
    Retries the given IO action up to `retries` times with a delay between attempts.
    Logs errors if all attempts fail.
    """
    for attempt in range(1, retries + 1):
        try:
            if action():
                logging.info(f"{action_name} succeeded on attempt {attempt}.")
                return True
            else:
                logging.warning(f"{action_name} failed on attempt {attempt}. Retrying...")
        except Exception as e:
            log_error_to_user(f"{action_name} exception on attempt {attempt}: {e}")
        time.sleep(delay)
    log_error_to_user(f"{action_name} failed after {retries} attempts.")
    return False

def safe_insert_frame(frame, retries=3, delay=0.2):
    for attempt in range(1, retries + 1):
        try:
            if not db is None: 
                with db_lock:
                    db.insert_frame(frame)
                return True
        except Exception as e:
            logging.warning(f"DB insert error on attempt {attempt}: {e}")
            time.sleep(delay)
    log_error_to_user("Failed to insert frame into DB after retries.")
    return False

def set_recording_type_from_server(rec_type, user="server"):
    """
    Allows server to set recording type: 'EVENT' or 'MANUAL'.
    """
    global recording_type
    if rec_type.upper() in ["EVENT", "MANUAL"]:
        old = recording_type
        recording_type = rec_type.upper()
        log_config_change("RECORDING_TYPE", old, recording_type, user)
        save_config()
        logging.info(f"Recording type set to {recording_type} via server.")
        return True
    logging.warning(f"Invalid recording type requested: {rec_type}")
    return False

def _prepare_backend_msg(event : SocketEventsToBackend, payload : dict = {}) -> QueueMessage:
    header : QueueMessageHeader = QueueMessageHeader(
        source=QueuesMembers.IR, 
        dest = QueuesMembers.BACKEND, 
        event =event,
        id="",
        user="",
        timestamp=time.time())
    return QueueMessage(header = header, payload=payload)    

def set_config(msg_in : QueueMessage) -> QueueMessage:
    try:
        source : QueuesMembers = msg_in.header.source
        dest : QueuesMembers = msg_in.header.dest
        user : str = msg_in.header.user
        id : str = msg_in.header.id 
        timestamp : float = msg_in.header.timestamp
        payload = msg_in.payload

        msg_out : QueueMessage

        if mode != SystemMode.TEST:
            msg_out = ack_config(id=id, status="error", message="Configuration changes only allowed in TEST mode.")
        else:
            if "start_threshold" in payload:
                set_start_threshold(payload["start_threshold"], user)
            if "stop_threshold" in payload:
                set_stop_threshold(payload["stop_threshold"], user)
            if "duration" in payload:
                set_duration(payload["duration"], user)
            if "manual_record_limit" in payload:
                set_manual_record_limit(payload["manual_record_limit"], user)
            if "save_dir" in payload:
                set_save_dir(payload["save_dir"], user)
            if "event_recording_enabled" in payload:
                if payload["event_recording_enabled"]:
                    enable_event_recording(user)
                else:
                    disable_event_recording(user)
            if "mode" in payload:
                set_mode(payload["mode"], user)
            if "recording_type" in payload:
                set_recording_type_from_server(payload["recording_type"], user)

            logger.info(f"[CONFIG] Updated by {user}: {payload}")
            msg_out = ack_config(id, "success", "Configuration updated.")

    except Exception as e:
        logger.info(f"Failed to set config: {e}")
        msg_out = ack_config(id=id, status="error", message="Error")
    return msg_out

def ack_config(id : str, status : str, message : str) -> QueueMessage:
    msg : QueueMessage = _prepare_backend_msg(
        event = SocketEventsToBackend.ACK_SET_CONFIG,
        payload = {
            "id": id,
            "status": status,
            "message": message,
            "mode": mode,
            "threshold": TEMP_THRESHOLD,
            "start_threshold": START_THRESHOLD,
            "stop_threshold": STOP_THRESHOLD,
            "duration": POST_EVENT_DURATION,
            "recording_type": recording_type
            }
    )
    return msg

def set_temperature(msg_in : QueueMessage) -> QueueMessage:
    """
    Handles temperature threshold setting from the server.
    Example data:
    {
        "command": "set_temperature",
        "request_id": "abc123",
        "payload": {
            "temp_threshold": 52.5
        }
    }
    """
    source : QueuesMembers = msg_in.header.source
    dest : QueuesMembers = msg_in.header.dest
    user : str = msg_in.header.user
    id : str = msg_in.header.id 
    timestamp : float = msg_in.header.timestamp
    payload = msg_in.payload

    msg_out : QueueMessage
    try:
        if not "temp_threshold" in payload:
            raise ValueError("temp_threshold not found in payload")
        temp_value = payload.get("temp_threshold")
        set_threshold(temp_value, user="server")
        msg_out = ack_set_temperature(id=id, status="success", message=f"Temperature threshold set to {temp_value}°C")
    except ValueError as e:
        msg_out = ack_set_temperature(id=id, status="error", message="Missing parameter value")
    except Exception as e:
        msg_out = ack_set_temperature(id=id, status="error", message=f"Failed to set temperature: {e}")
    return msg_out

def ack_set_temperature(id : str, status : str, message : str) -> QueueMessage:
    msg : QueueMessage = _prepare_backend_msg(
        event = SocketEventsToBackend.ACK_SET_TEMPRETURE,
        payload = {
                "id": id,
                "status": status,
                "message": message
            }
    )
    return msg

def manual_start_record(msg_in : QueueMessage) -> QueueMessage:
    global manual_record_thread, recording
    source : QueuesMembers = msg_in.header.source
    dest : QueuesMembers = msg_in.header.dest
    user : str = msg_in.header.user
    id : str = msg_in.header.id 
    timestamp : float = msg_in.header.timestamp
    payload = msg_in.payload

    msg_out : QueueMessage

    if mode != SystemMode.TEST:
        msg_out = ack_manual_start_record(id=id, status="error", message= "Only allowed in TEST mode")
    else:
        if not recording:
            manual_record_thread = threading.Thread(
                target=record_video, args=(cam, mode, POST_EVENT_DURATION))
            manual_record_thread.start()
            recording = True
            msg_out = ack_manual_start_record(id=id, status="success", message="Recording started")
        else:
            msg_out = ack_manual_start_record(id=id, status="error", message="Already recording")
    return msg_out

def ack_manual_start_record(id : str, status : str, message : str):
    msg : QueueMessage = _prepare_backend_msg(
        event = SocketEventsToBackend.ACK_MANUAL_START_RECORD,
        payload = {
                "id": id,
                "status": status,
                "message": message
            }
    )
    return msg

def manual_stop_record(msg_in : QueueMessage) -> QueueMessage:
    source : QueuesMembers = msg_in.header.source
    dest : QueuesMembers = msg_in.header.dest
    user : str = msg_in.header.user
    id : str = msg_in.header.id 
    timestamp : float = msg_in.header.timestamp
    payload = msg_in.payload

    msg_out : QueueMessage

    success = stop_manual_recording_from_server()
    if success:
        msg_out = ack_manual_stop_record(id=id, status="success", message="Recording stopped")
    else:
        msg_out = ack_manual_stop_record(id=id, status="error", message="No active recording")
    return msg_out

def ack_manual_stop_record(id : str, status : str, message : str) -> QueueMessage:
    msg : QueueMessage = _prepare_backend_msg(
        event = SocketEventsToBackend.ACK_MANUAL_STOP_RECORD,
        payload = {
                "id": id,
                "status": status,
                "message": message
            }
    )
    return msg


def timeout_stop_record(msg_in : QueueMessage) -> QueueMessage:
    global manual_stop_flag, recording
    source : QueuesMembers = msg_in.header.source
    dest : QueuesMembers = msg_in.header.dest
    user : str = msg_in.header.user
    id : str = msg_in.header.id 
    timestamp : float = msg_in.header.timestamp
    payload = msg_in.payload

    msg_out : QueueMessage
    if recording:
        manual_stop_flag = True
        msg_out = ack_timeout_stop_record(id=id, status="success", message="Recording timed out and stopped")
    else:
        msg_out = ack_timeout_stop_record(id=id, status="error", message="Nothing was recording")
    return msg_out

def ack_timeout_stop_record(id : str, status : str, message : str)->QueueMessage:
    msg : QueueMessage = _prepare_backend_msg(
        event = SocketEventsToBackend.ACK_TIMEOUT_STOP_RECORD,
        payload = {
                "id": id,
                "status": status,
                "message": message
            }
    )
    return msg

def call_live_temperature(msg_in : QueueMessage) -> QueueMessage:
    global temp
    source : QueuesMembers = msg_in.header.source
    dest : QueuesMembers = msg_in.header.dest
    user : str = msg_in.header.user
    id : str = msg_in.header.id 
    timestamp : float = msg_in.header.timestamp
    payload = msg_in.payload

    msg_out : QueueMessage

    msg_out = ack_live_temperature(id=id,status= "success",message = {
        "temp": temp,
        "mode": mode,
        "recording": recording
    })
    return msg_out

def ack_live_temperature(id : str, status : str, message : dict):
    msg : QueueMessage = _prepare_backend_msg(
        event = SocketEventsToBackend.ACK_CALL_LIVE_TEMPRETURE,
        payload = {
                "id": id,
                "status": status,
                "temperature": message.get("temp"),
                "mode": message.get("mode"),
                "recording": message.get("recording")
            }
    )
    return msg

def call_history_temperature(msg_in : QueueMessage) -> QueueMessage:
    source : QueuesMembers = msg_in.header.source
    dest : QueuesMembers = msg_in.header.dest
    user : str = msg_in.header.user
    id : str = msg_in.header.id 
    timestamp : float = msg_in.header.timestamp
    payload = msg_in.payload

    msg_out : QueueMessage

    recent_errors = get_recent_errors(limit=10)
    msg_out = ack_history_temperature(id=id, status="success", message=recent_errors)
    return msg_out

def ack_history_temperature(id : str, status : str, message : list):
    msg : QueueMessage = _prepare_backend_msg(
    event = SocketEventsToBackend.ACK_CALL_LIVE_TEMPRETURE,
    payload = {
            "id": id,
            "status": status,
            "errors": message
        }
    )
    return msg

def set_event(msg_in : QueueMessage) -> QueueMessage:
    source : QueuesMembers = msg_in.header.source
    dest : QueuesMembers = msg_in.header.dest
    user : str = msg_in.header.user
    id : str = msg_in.header.id 
    timestamp : float = msg_in.header.timestamp
    payload = msg_in.payload

    msg_out : QueueMessage

    try:
        if not "enable" in payload:
            raise ValueError("temp_threshold not found in payload")
        enable = payload.get("enable", True)

        if enable:
            enable_event_recording()
            msg_out = ack_event(id=id, status="success", message="Event recording enabled")
        else:
            disable_event_recording()
            msg_out = ack_event(id=id, status="success", message="Event recording disabled")
    except ValueError as e:
        msg_out = ack_set_temperature(id=id, status="error", message="Missing parameter value")
    except Exception as e:
        msg_out = ack_set_temperature(id=id, status="error", message=f"Failed to set temperature: {e}")
    return msg_out

def ack_event(id : str, status : str, message : str) -> QueueMessage:
    msg : QueueMessage = _prepare_backend_msg(
    event = SocketEventsToBackend.ACK_SET_EVENT,
    payload = {
            "id": id,
            "status": status,
            "errors": message
        }
    )
    return msg

def call_record(msg_in : QueueMessage) -> QueueMessage:
    source : QueuesMembers = msg_in.header.source
    dest : QueuesMembers = msg_in.header.dest
    user : str = msg_in.header.user
    id : str = msg_in.header.id 
    timestamp : float = msg_in.header.timestamp
    payload = msg_in.payload

    msg_out : QueueMessage

    if mode == SystemMode.TEST:
        msg_out = ack_call_record(id=id, status="success", message="Test anomaly triggered")
    else:
        msg_out = ack_call_record(id=id, status="error", message="Only allowed in TEST mode")
    return msg_out

def ack_call_record(id : str, status : str, message : str) -> QueueMessage:
    msg : QueueMessage = _prepare_backend_msg(
    event = SocketEventsToBackend.ACK_MANUAL_CALL_RECORD,
    payload = {
            "id": id,
            "status": status,
            "message": message
        }
    )
    return msg

def reset_alarm(msg_in : QueueMessage) -> QueueMessage:
    source : QueuesMembers = msg_in.header.source
    dest : QueuesMembers = msg_in.header.dest
    user : str = msg_in.header.user
    id : str = msg_in.header.id 
    timestamp : float = msg_in.header.timestamp
    payload = msg_in.payload

    msg_out : QueueMessage

    try:
        set_relais_state(False)
        msg_out = ack_reset_alarm(id=id, status="success", message="Alarm reset successful.")
    except Exception as e:
        msg_out = ack_reset_alarm(id=id, status="error", message=str(e))
    return msg_out

def ack_reset_alarm(id : str, status : str, message : str) -> QueueMessage:
    msg : QueueMessage = _prepare_backend_msg(
    event = SocketEventsToBackend.ACK_RESET_ALARM,
    payload = {
            "id": id,
            "status": status,
            "message": message
        }
    )
    return msg

def reset_error(msg_in : QueueMessage) -> QueueMessage:
    source : QueuesMembers = msg_in.header.source
    dest : QueuesMembers = msg_in.header.dest
    user : str = msg_in.header.user
    id : str = msg_in.header.id 
    timestamp : float = msg_in.header.timestamp
    payload = msg_in.payload

    msg_out : QueueMessage

    try:
        error_history.clear()
        unfreeze_relais()
        logging.info("Error state cleared by server.")
        msg_out = ack_reset_error(id=id, status="success", message="Error state cleared successfully.")
    except Exception as e:
        log_error_to_user(f"Failed to reset error: {e}")
        msg_out = ack_reset_error(id=id, status="error", message=str(e))

    return msg_out

def ack_reset_error(id : str, status : str, message : str) -> QueueMessage:
    msg : QueueMessage = _prepare_backend_msg(
    event = SocketEventsToBackend.ACK_RESET_ERROR,
    payload = {
            "id": id,
            "status": status,
            "message": message
        }
    )
    return msg

# IR Command Handler    
def ir_command_handler(msg_in : QueueMessage) -> QueueMessage:
    try:
        header = msg_in.header
        payload = msg_in.payload

        command = header.event
        data = payload
        msg_out : QueueMessage
    
        logger.info(f"[IR COMMAND] Received: {command} | Data: {data}")

        if command == SocketEventsFromBackend.REQ_SET_CONFIG:
            msg_out = set_config(msg_in=msg_in)

        elif command == SocketEventsFromBackend.REQ_SET_TEMPRETURE:
            msg_out = set_temperature(msg_in=msg_in)

        elif command == SocketEventsFromBackend.REQ_MANUAL_START_RECORD:
            msg_out = manual_start_record(msg_in=msg_in)

        elif command == SocketEventsFromBackend.REQ_MANUAL_STOP_RECORD:
            msg_out = manual_stop_record(msg_in=msg_in)
    
        elif command == "timeout_stop_record":
            msg_out = timeout_stop_record(msg_in=msg_in)

        elif command == SocketEventsFromBackend.REQ_CALL_LIVE_TEMPRETURE:  # Note: fix spelling if needed
            msg_out = call_live_temperature(msg_in=msg_in)

        elif command == SocketEventsFromBackend.REQ_CALL_HISTORY_TEMPRETURE:
            msg_out = call_history_temperature(msg_in=msg_in)

        elif command == SocketEventsFromBackend.REQ_SET_EVENT:
            msg_out = set_event(msg_in=msg_in)

        elif command == SocketEventsFromBackend.REQ_MANUAL_CALL_RECORD:
            msg_out = call_record(msg_in=msg_in)

        elif command == SocketEventsFromBackend.REQ_RESET_ALARM:
            msg_out = reset_alarm(msg_in=msg_in)

        elif command == SocketEventsFromBackend.REQ_RESET_ERROR:
            msg_out = reset_error(msg_in=msg_in)

        else:
            logger.warning(f"[IR COMMAND] Unknown command: {command}")

    except Exception as e:
        logger.error(f"[IR COMMAND] Handler error: {e}")
    return msg_out

# # Main Loop 
# def main():
#     global anomaly_worker_thread 
#     global cam, db, mode, frame, temp, recording, anomaly_active
#     global anomaly_thread, manual_record_thread
#     global last_trigger_time, last_test_time, exit_flag, event_recording_enabled

#     load_config()  
#     anomaly_worker_thread = None

#     RETRIGGER_COOLDOWN = 15
#     TEST_TIMEOUT = 180
#     last_trigger_time = 0
#     last_test_time = time.time()

#     try:
#         cam = camera_control.CameraController()
#         db = frame_database.FrameDatabase("frame_store.db")
#         threading.Thread(target=ir_command_handler, args=(queue_ir,), daemon=True).start()

#     except Exception as e:
#         logging.critical(f"Failed to initialize camera or DB: {e}")
#         mode = SystemMode.FAULT
#         event_recording_enabled = False
#         cam = None


#     try:
#         while True:
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord('q'):
#                 logging.info("Exiting now...")
#                 exit_flag = True
#                 break
#             elif key == ord('f'):
#                 set_mode(SystemMode.FAULT)
#             elif key == ord('t'):
#                 set_mode(SystemMode.TEST)
#             elif key == ord('n'):
#                 set_mode(SystemMode.NORMAL)
#             elif key == ord('s') and frame is not None:
#                 threading.Thread(target=screenshot, args=(frame.copy(),)).start()
#             elif key == ord('v') and frame is not None and not recording and mode == SystemMode.TEST:
#                 manual_record_thread = threading.Thread(
#                     target=record_video, args=(cam, mode, POST_EVENT_DURATION))
#                 manual_record_thread.start()
#                 recording = True

#             elif key == ord('a') and USE_MOCK_CAMERA:
#                 cam.trigger_anomaly()
#             elif key == ord('h') and mode == SystemMode.TEST:
#                 trigger_hupe()
#             elif key == ord('b') and mode == SystemMode.TEST:
#                 trigger_blitz()
#             elif key == ord('r') and mode == SystemMode.TEST:
#                 set_relais_state(True)
#             elif key == ord('z') and mode == SystemMode.TEST:
#                 freeze_relais()
#             elif key == ord('u') and mode == SystemMode.TEST:
#                 unfreeze_relais()
#             elif key == ord('f'):  # Reinitialize the camera
#                     try:
#                         cam = CameraController()
#                         logging.info("Camera re-initialized successfully. Switching to NORMAL mode.")
#                         set_mode(SystemMode.NORMAL)
#                     except Exception as e:
#                         log_error_to_user(f"Failed to initialize camera or DB: {e}")



#             if mode == SystemMode.TEST and (time.time() - last_test_time) > TEST_TIMEOUT:
#                 logging.info("Test mode timeout. Switching to NORMAL.")
#                 mode = SystemMode.NORMAL

#             try:
#                 with camera_lock:
#                     frame, temp = cam.get_frame()
#                     # Send heartbeat to überwachung.py (IRAppProcess)
#                 try:
#                     if 'events_ir' in globals() and events_ir:
#                         events_ir.heartbeat.set()
#                 except Exception:
#                     pass


#                 if frame is None:
#                     log_error_to_user("Camera returned no frame. Switching to FAULT mode.")
#                     set_mode(SystemMode.FAULT)
#                     frame = generate_error_image()  # Show error image
#                     temp = None
#             except Exception as e:
#                 log_error_to_user(f"Camera error: {e}. Switching to FAULT mode.")
#                 set_mode(SystemMode.FAULT)
#                 frame = generate_error_image()
#                 temp = None

#             if frame is not None:
#                 try:
#                     safe_insert_frame(frame)
#                     timestamp = datetime.datetime.now().isoformat()
#                     with open(FRAME_LOG_FILE, mode='a', newline='') as csvfile:
#                         writer = csv.writer(csvfile)
#                         writer.writerow([timestamp, mode, f"{temp:.2f}" if temp is not None else "N/A", recording])
#                 except Exception as e:
#                     logging.warning("DB insert error: %s", e)

#             # Event-based recording for Normal mode
#             # Event-based recording and IO control for Normal mode
#             # Event-based anomaly detection with STOP_THRESHOLD + queue
#             if mode == SystemMode.NORMAL and temp is not None:
#                 if temp > START_THRESHOLD and not anomaly_active:
#                     logging.info(f"New anomaly detected: Temp = {temp:.2f} °C")
#                     # Queue anomaly event
#                     anomaly_queue.put((temp, datetime.datetime.now()))

#                     # Trigger IO
#                     retry_io_action(trigger_hupe, "HUPE Trigger")
#                     retry_io_action(trigger_blitz, "BLITZ Trigger")
#                     retry_io_action(lambda: set_relais_state(True), "Set RELAIS ON")

#                     anomaly_active = True  # Mark anomaly as ongoing
#                 elif temp < STOP_THRESHOLD and not recording:
#                     anomaly_active = False  # Reset anomaly state for next event

#                 # Start anomaly worker if idle
#                 if not recording and not anomaly_queue.empty() and \
#                 (anomaly_worker_thread is None or not anomaly_worker_thread.is_alive()):
#                     anomaly_worker_thread = threading.Thread(target=anomaly_worker)
#                     anomaly_worker_thread.start()

#             # TEST MODE anomaly simulation
#             if mode == SystemMode.TEST and USE_MOCK_CAMERA and temp is not None:
#                 if temp > START_THRESHOLD and recording_type == "EVENT" and not anomaly_active:
#                     logging.info(f"Test Mode Anomaly: Temp = {temp:.2f} °C (EVENT mode)")
#                     anomaly_queue.put((temp, datetime.datetime.now()))
#                     anomaly_active = True
#                 elif temp < STOP_THRESHOLD and not recording:
#                     anomaly_active = False

#                 if not recording and not anomaly_queue.empty() and \
#                     (anomaly_worker_thread is None or not anomaly_worker_thread.is_alive()):
#                     anomaly_worker_thread = threading.Thread(target=anomaly_worker)
#                     anomaly_worker_thread.start()

#             elif recording and manual_record_thread and not manual_record_thread.is_alive():
#                 try:
#                     set_relais_state(False)
#                 except Exception as e:
#                     log_error_to_user(f"Failed to reset relais: {e}")
#                 logging.info("Event recording finished, system re-armed.")
#                 recording = False
#                 manual_record_thread = None

#             if anomaly_thread and not anomaly_thread.is_alive():
#                 anomaly_thread = None
#             if manual_record_thread and not manual_record_thread.is_alive():
#                 recording = False
#                 manual_record_thread = None
#             if exit_flag:
#                 break

#             if frame is not None:
#                 resized = cv2.resize(frame, (frame.shape[1] * 3, frame.shape[0] * 3))
#                 display_frame = display(resized, temp, mode, recording)
#                 cv2.imshow("Thermal View", display_frame)


#     finally:
#         exit_flag = True
#         logging.info("Shutting down threads...")
#         if manual_record_thread and manual_record_thread.is_alive():
#             manual_stop_flag = True
#             manual_record_thread.join(timeout=0.5)
#         if anomaly_worker_thread and anomaly_worker_thread.is_alive():
#             anomaly_worker_thread.join(timeout=0.5)

#         if cam and hasattr(cam, "shutdown"):
#             cam.shutdown()
#         if db:
#             db.close()
#         cv2.destroyAllWindows()
#         logging.info("Shutdown complete.")


# if __name__ == "__main__":
#     main()
