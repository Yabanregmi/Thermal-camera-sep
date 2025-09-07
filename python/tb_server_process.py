from typing import Any
import multiprocessing

from logging import Logger
from tb_events import ServerEvents
from tb_queues import MainQueues, SocketQueues, Tb_Queue
import time
from models.tb_dataclasses import QueuesMembers, SocketEventsFromBackend, SocketEventsToBackend, QueueMessage, QueueTestEvents, QueueMessageHeader
from typing import Callable, Any

MAXIMUM_TIMEOUT: int = 10
HandlerType = Callable[..., None]
from tb_socket import Tb_Socket

SERVER_PROCESS_TICK_s : float = 0.1
SERVER_PROCESS_BACKEND_TEST_MSG_TICK_s : float = SERVER_PROCESS_TICK_s*10*5

class Tb_ServerProcess(multiprocessing.Process):
    """
    Basisklasse für alle Prozesse im System, die mit dem Server kommunizieren.
    """
    def __init__(self, name: str, logger: Logger, logger_backend:Logger,url: str, events: ServerEvents, main_queues : MainQueues, socket_queues : SocketQueues) -> None:
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
        self._validate_str(value=name, name="Name", min_len=3, max_len=50)
        self._validate_str(value=url, name="URL", min_len=5, max_len=200)

        super().__init__(name=name)
        self.logger :Logger= logger
        self.url :str= url
        self.events : ServerEvents = events
        
        self.main_queues : MainQueues = main_queues
        self.backend_queue : Tb_Queue = Tb_Queue(name="Backend", logger=logger_backend)
        self.socket_queues : SocketQueues = socket_queues
        
        self.in_konfig_modus: bool = False
        self.is_connected: bool = False
        self.is_connected_error : bool = False
        self.tick_counter : int = 0
        self.tick_test_pre_counter : int =0
        self.events_backend = [
            "ack_config", "ack_tempreture", "timeout_stop_record",
            "send_live_tempreture", "ack_call_live_tempreture",
            "ack_send_live_tempreture"
        ]

        self.sio : Tb_Socket = Tb_Socket(logger=self.logger)
        self.sio.register_event_handler(SocketEventsFromBackend.CONNECT, self.connect_handler)
        self.sio.register_event_handler(SocketEventsFromBackend.DISCONNECT, self.disconnect_handler)
        self.sio.register_event_handler(SocketEventsFromBackend.CONNECT_ERROR, self.connect_error_handler)
        self.sio.register_event_handler(SocketEventsFromBackend.REQ_RESET_ALARM, self.reset_alarm_handler)
        self.sio.register_event_handler(SocketEventsFromBackend.REQ_RESET_ERROR, self.reset_error_handler)
        self.sio.register_event_handler(SocketEventsFromBackend.REQ_SET_CONFIG, self.set_config_handler)
        self.sio.register_event_handler(SocketEventsFromBackend.REQ_SET_TEMPRETURE, self.set_tempreture_handler)
        self.sio.register_event_handler(SocketEventsFromBackend.REQ_MANUAL_START_RECORD, self.manual_start_record_handler)
        self.sio.register_event_handler(SocketEventsFromBackend.REQ_MANUAL_STOP_RECORD, self.manual_stop_record_handler)
        self.sio.register_event_handler(SocketEventsFromBackend.REQ_MANUAL_CALL_RECORD, self.manual_call_record_handler)
        self.sio.register_event_handler(SocketEventsFromBackend.REQ_CALL_LIVE_TEMPRETURE, self.call_live_tempreture_handler)
        self.sio.register_event_handler(SocketEventsFromBackend.REQ_CALL_HISTORY_TEMPRETURE, self.call_history_tempreture_handler)

        self.logger.debug(f"{self.__class__.__name__} - {self.name} init")
    # ------------------- Hilfsfunktion -------------------

    @staticmethod
    def _validate_str(value: str, name: str, min_len: int, max_len: int) -> None:
        if not (min_len <= len(value) <= max_len):
            raise ValueError(f"{name} muss zwischen {min_len} und {max_len} Zeichen lang sein.")
        
    # Event-Handler
    def connect_handler(self) -> None:
        self.is_connected = True
        self.is_connected_error = False
        self.events.connect.set()
        self.logger.debug(f"Verbindung url: {self.url} protocol: {self.sio.wrapper_transport()} sid: {self.sio.wrapper_get_sid()} erfolgreich!")

    def disconnect_handler(self) -> None:
        self.is_connected = False
        self.events.disconnect.set()
        self.logger.debug(f"Verbindung zu {self.url} beendet!")

    def connect_error_handler(self, data: Any) -> None:
        self.is_connected = False
        self.events.error_on_connection.set()
        if not self.is_connected_error:
            self.logger.debug(f"Connection error: {data}")
            self.is_connected_error = True

    def reset_alarm_handler(self,payload) -> None:
        self._send_backend_msg_to_ir(event=SocketEventsFromBackend.REQ_RESET_ALARM,payload=payload)
        self.logger.debug("Received form backend: REQ_RESET_ALARM")

    def reset_error_handler(self,payload) -> None:
        self._send_backend_msg_to_ir(event=SocketEventsFromBackend.REQ_RESET_ERROR,payload=payload)
        self.logger.debug("Received form backend: REQ_RESET_ERROR")

    def set_config_handler(self,payload) -> None:
        self._send_backend_msg_to_ir(event=SocketEventsFromBackend.REQ_SET_CONFIG,payload=payload)
        self.logger.debug("Received form backend: REQ_SET_CONFIG")

    def set_tempreture_handler(self,payload) -> None:
        self._send_backend_msg_to_ir(event=SocketEventsFromBackend.REQ_SET_TEMPRETURE,payload=payload)
        self.logger.debug(f"Received form backend: REQ_SET_TEMPRETURE")

    def manual_start_record_handler(self,payload) -> None:
        self._send_backend_msg_to_ir(event=SocketEventsFromBackend.REQ_MANUAL_START_RECORD,payload=payload)
        self.logger.debug("Received form backend: REQ_MANUAL_START_RECORD")

    def manual_stop_record_handler(self,payload) -> None:
        self._send_backend_msg_to_ir(event=SocketEventsFromBackend.REQ_MANUAL_STOP_RECORD,payload=payload)
        self.logger.debug("Received form backend: REQ_MANUAL_STOP_RECORD")

    def manual_call_record_handler(self,payload) -> None:
        self._send_backend_msg_to_ir(event=SocketEventsFromBackend.REQ_MANUAL_CALL_RECORD,payload=payload)
        self.logger.debug("Received form backend: REQ_MANUAL_CALL_RECORD")

    def call_live_tempreture_handler(self,payload) -> None:
        self._send_backend_msg_to_ir(event=SocketEventsFromBackend.REQ_CALL_LIVE_TEMPRETURE,payload=payload)
        self.logger.debug("Received form backend: REQ_CALL_LIVE_TEMPRETURE")

    def call_history_tempreture_handler(self,payload) -> None:
        self._send_backend_msg_to_ir(event=SocketEventsFromBackend.REQ_CALL_HISTORY_TEMPRETURE,payload=payload)
        self.logger.debug("Received form backend: REQ_CALL_HISTORY_TEMPRETURE")
    
    # Backend-Sende-Methoden
    def _prepare_backend_msg(self, event : SocketEventsToBackend, payload : dict = {}) -> QueueMessage:
        header : QueueMessageHeader = QueueMessageHeader(
            source=QueuesMembers.SERVER, 
            dest = QueuesMembers.BACKEND, 
            event =event,
            id="",
            user="",
            timestamp=time.time())
        return QueueMessage(header = header, payload=payload)    
    
    def _prepare_queue_test_msg(self, event : QueueTestEvents, payload : dict = {}) -> QueueMessage:
        header : QueueMessageHeader = QueueMessageHeader(
            source=QueuesMembers.SERVER, 
            dest = QueuesMembers.MAIN, 
            event =event,
            id="",
            user="",
            timestamp=time.time())
        return QueueMessage(header = header, payload=payload)    

    def _send_backend_msg_to_ir(self, event : SocketEventsFromBackend, payload : dict = {}) -> None:
        header : QueueMessageHeader = QueueMessageHeader(
            source=QueuesMembers.BACKEND, 
            dest = QueuesMembers.IR, 
            event =event,
            id="",
            user="",
            timestamp=time.time())
        msg : QueueMessage =  QueueMessage(header = header, payload=payload)    
        self.backend_queue.put(item=msg)
        
    def send_backend_ack_config(self,data:dict) -> None:
        msg : QueueMessage = self._prepare_backend_msg(event = SocketEventsToBackend.ACK_SET_CONFIG, payload=data)     
        self.sio.send_event(msg=msg,callback=self.send_backend_ack_config_callback)

    def send_backend_ack_tempreture(self,data:dict) -> None:
        msg : QueueMessage = self._prepare_backend_msg(event = SocketEventsToBackend.ACK_SET_TEMPRETURE, payload=data)    
        self.sio.send_event(msg=msg,callback=self.send_backend_ack_tempreture_callback)

    def send_backend_timeout_stop_record(self,data:dict) -> None:
        msg : QueueMessage = self._prepare_backend_msg(event = SocketEventsToBackend.ACK_MANUAL_STOP_RECORD, payload=data) 
        self.sio.send_event(msg=msg,callback=self.send_backend_timeout_stop_record_callback)

    def send_backend_ack_call_live_tempreture(self,data:dict) -> None:
        msg : QueueMessage = self._prepare_backend_msg(event = SocketEventsToBackend.ACK_CALL_LIVE_TEMPRETURE, payload=data) 
        self.sio.send_event(msg=msg,callback=self.send_backend_ack_call_live_tempreture_callback)

    def send_backend_test(self) -> None:
        msg : QueueMessage = self._prepare_backend_msg(event = SocketEventsToBackend.REQ_TEST, payload={"test":"test"}) 
        self.sio.send_event(msg=msg,callback=self.send_backend_send_test_callback)

    def send_backend_ack_config_callback(self, response: Any = None) -> None:
        self.logger.debug(f"Callback ack_config empfangen mit response: {response}")

    def send_backend_ack_tempreture_callback(self, response: Any = None) -> None:
        self.logger.debug(f"Callback ack_tempreture empfangen mit response: {response}")

    def send_backend_timeout_stop_record_callback(self, response: Any = None) -> None:
        self.logger.debug(f"Callback timeout_stop_record empfangen mit response: {response}")

    def send_backend_ack_call_live_tempreture_callback(self, response: Any = None) -> None:
        self.logger.debug(f"Callback ack_call_live_tempreture empfangen mit response: {response}")

    def send_backend_ack_send_live_tempreture_callback(self, response: Any = None) -> None:
        self.logger.debug(f"Callback ack_send_live_tempreture empfangen mit response: {response}")

    def send_backend_send_live_tempreture_callback(self, response: Any = None) -> None:
        self.logger.debug(f"Callback send_live_tempreture empfangen mit response: {response}")

    def send_backend_send_test_callback(self, response: Any = None) -> None:
        self.logger.debug(f"Callback send_backend_send_test_callback empfangen mit response: {response}")

    def shutdown(self):
        """
        Setzt das Shutdown-Event, um den Prozess zu beenden.
        """
        if not self.events.shutdown.is_set():
            self.events.shutdown.set()
            self.logger.debug(f"{self.__class__.__name__} - {self.name} called shutdown")

    def reset(self) -> bool:
        """
        Setzt den Prozess zurück und trennt die Verbindung.

        Returns:
            bool: True, wenn die Verbindung erfolgreich getrennt wurde, sonst False.
        """
        self.state : bool = True
        if self.is_connected :
            self.sio.wrapper_shutdown()
            self.sio.wrapper_disconnect()
            self.is_connected = False
            self.state = self.events.disconnect.wait(timeout=MAXIMUM_TIMEOUT)
        self.logger.debug(f"{self.__class__.__name__} - {self.name} reset")
        return self.state

    def run(self) -> None:
        """
        Führt die Hauptlogik des Server-Prozesses aus.
        """
        self.logger.debug(f"{self.__class__.__name__} - {self.name} running")
        while not self.events.shutdown.is_set():
            if  self.events.error_on_connection.is_set():
                self.events.error_on_connection.clear()
                self.events.error_from_server_process.set()

            if not self.is_connected_error:
                if not self.events.server_process_okay.is_set():
                    self.events.server_process_okay.set()
        
            if not self.is_connected:
                try:
                    self.sio.wrapper_connect(self.url, transports=["websocket"])
                    self.is_connected = self.events.connect.wait(timeout=MAXIMUM_TIMEOUT)
                except Exception:
                    self.is_connected = False
        
            msg_from_internal = self.main_queues.server.get()
            
            if not msg_from_internal is None and msg_from_internal.header.dest is QueuesMembers.SERVER:
                if msg_from_internal.header.source is QueuesMembers.MAIN and msg_from_internal.header.event in QueueTestEvents :
                    if msg_from_internal.header.event is QueueTestEvents.REQ_FROM_MAIN_TO_SERVER:
                        if not self.queue_test_send_ack(req_msg=msg_from_internal):
                            self.events.error_from_server_process.set()
                elif self.is_connected :
                    if msg_from_internal.header.source is QueuesMembers.IR and msg_from_internal.header.event in SocketEventsToBackend:
                        if msg_from_internal.header.event == SocketEventsToBackend.ACK_SET_CONFIG:
                            self.send_backend_ack_config(data=msg_from_internal.payload)
                        if msg_from_internal.header.event == SocketEventsToBackend.ACK_SET_TEMPRETURE:
                            self.send_backend_ack_tempreture(data=msg_from_internal.payload)
                        if msg_from_internal.header.event == SocketEventsToBackend.ACK_TIMEOUT_STOP_RECORD:
                            self.send_backend_timeout_stop_record(data=msg_from_internal.payload)
                        if msg_from_internal.header.event == SocketEventsToBackend.ACK_CALL_HISTORY_TEMPRETURE:
                            self.send_backend_ack_call_live_tempreture(data=msg_from_internal.payload)

            if abs(self.tick_counter - self.tick_test_pre_counter) > 50:
                self.tick_test_pre_counter = self.tick_counter
                self.send_backend_test()
            
            if self.tick_counter < 100:
                self.tick_counter = self.tick_counter + 1
            else:
                self.tick_counter = 0

            self.events.heartbeat.set()
            time.sleep(SERVER_PROCESS_TICK_s)
        if not self.reset():
            self.logger.debug(f"{self.__class__.__name__} - {self.name} shutdown error")
        else:
            self.logger.debug(f"{self.__class__.__name__} - {self.name} shutdown")
        
        if self.events.shutdown.is_set():
            self.events.shutdown.clear()
        time.sleep(1)
        
    def queue_test_send_ack(self, req_msg : QueueMessage) -> bool:
        status : bool = False
        try:
            if req_msg.header.event is QueueTestEvents.REQ_FROM_MAIN_TO_SERVER:
                res_msg : QueueMessage = self._prepare_queue_test_msg(event=QueueTestEvents.ACK_FROM_SERVER_TO_MAIN)               
                if not self.main_queues.main.put(item=res_msg):
                    status = False
                else:
                    status = True
        except Exception:
            status = False
        return status