import socketio
from typing import Callable
from logging import Logger
from models.tb_dataclasses import SocketEventsFromBackend
from tb_queue_test import QueueMessage
import json

class Tb_Socket(socketio.Client):
    def __init__(self, logger: Logger):
        # Standard-Parameter können hier schon gesetzt werden
        super().__init__(# type: ignore
            reconnection=True,
            reconnection_attempts=3,
            reconnection_delay_max=5,
            handle_sigint=False)
        self.logger = logger
        self.logger.debug("Socket Init")

    def register_event_handler(self, event: SocketEventsFromBackend, handler: Callable[..., None]) -> None:
        try:
            self.on(event=event.value, handler=handler)  # type: ignore
            self.logger.debug(f"Event handler registered for event '{event.value}'")
        except Exception as e:
            err_msg = f"Fehler beim Senden des Events '{event.value}': {e}"
            self.logger.error(err_msg)

    def send_event(self, msg: QueueMessage, callback: Callable[..., None]) -> None:
        try:
            self.emit(event=msg.header.event.value, data=json.dumps(msg.payload), callback=callback)  # type: ignore
            self.logger.debug(f"Event '{msg.header.event.value}' gesendet mit Daten: {msg.payload}")
        except Exception as e:
            err_msg = f"Fehler beim Senden des Events '{msg.header.event.value}': {e}"
            self.logger.error(err_msg)

    def wrapper_transport(self) -> str:
        return self.transport()# type: ignore

    def wrapper_get_sid(self) -> str:
        return self.get_sid() # type: ignore

    def wrapper_shutdown(self) -> None:
        return self.shutdown() # type: ignore
    
    def wrapper_connect(self,url:str, transports :list[str]=["websocket"]) -> None:
        return self.connect(url=url,transports=transports) # type: ignore

    def wrapper_disconnect(self) -> None:
        return self.disconnect() # type: ignore