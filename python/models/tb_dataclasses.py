from email import header
import json
import time
from dataclasses import dataclass, asdict, field
from typing import Type, Literal
from enum import Enum

LogLevel = Literal[10, 20, 30, 40, 50]

class SocketEventsFromBackend(Enum):
    CONNECT = "connect"
    DISCONNECT = "disconnect"
    CONNECT_ERROR = "connect_error"
    REQ_RESET_ALARM = "REQ_RESET_ALARM"
    REQ_RESET_ERROR = "REQ_RESET_ERROR"
    REQ_SET_CONFIG = "REQ_SET_CONFIG"
    REQ_SET_TEMPRETURE = "REQ_SET_TEMPRETURE"
    REQ_MANUAL_START_RECORD = "REQ_MANUAL_START_RECORD"
    REQ_MANUAL_STOP_RECORD = "REQ_MANUAL_STOP_RECORD"
    REQ_MANUAL_CALL_RECORD = "REQ_MANUAL_CALL_RECORD"
    REQ_CALL_LIVE_TEMPRETURE = "REQ_CALL_LIVE_TEMPRETURE"
    REQ_CALL_HISTORY_TEMPRETURE = "REQ_CALL_HISTORY_TEMPRETURE"
    REQ_SET_EVENT = "REQ_SET_EVENT"
    MESSAGE = "MESSAGE"


class SocketEventsToBackend(Enum):
    ACK_RESET_ALARM = "ACK_RESET_ALARM"
    ACK_RESET_ERROR = "ACK_RESET_ERROR"
    ACK_SET_CONFIG = "ACK_SET_CONFIG"
    ACK_SET_TEMPRETURE = "ACK_SET_TEMPRETURE"
    ACK_MANUAL_START_RECORD = "ACK_MANUAL_START_RECORD"
    ACK_MANUAL_STOP_RECORD = "ACK_MANUAL_STOP_RECORD"
    ACK_TIMEOUT_STOP_RECORD = "ACK_TIMEOUT_STOP_RECORD"
    ACK_MANUAL_CALL_RECORD = "ACK_MANUAL_CALL_RECORD"
    ACK_CALL_LIVE_TEMPRETURE = "ACK_CALL_LIVE_TEMPRETURE"
    ACK_CALL_HISTORY_TEMPRETURE = "ACK_CALL_HISTORY_TEMPRETURE"
    ACK_SET_EVENT = "ACK_SET_EVENT"
    ACK_MESSAGE = "ACK_MESSAGE"
    REQ_TEST = "REQ_TEST"

class QueuesMembers(Enum):
    SERVER = "server"
    IR = "ir",
    MAIN = "main"
    BACKEND = "backend"

class QueueTestEvents(Enum):
    REQ_FROM_MAIN_TO_MAIN = "REQ_FROM_MAIN_TO_MAIN"
    REQ_FROM_MAIN_TO_SERVER = "REQ_FROM_MAIN_TO_SERVER"
    REQ_FROM_MAIN_TO_IR = "REQ_FROM_MAIN_TO_IR"
    ACK_FROM_SERVER_TO_MAIN = "ACK_FROM_SERVER_TO_MAIN"
    ACK_FROM_IR_TO_MAIN = "ACT_FROM_IR_TO_MAIN"

@dataclass
class QueueMessageHeader:
    source : QueuesMembers
    dest : QueuesMembers
    event: SocketEventsFromBackend | SocketEventsToBackend | QueueTestEvents
    id:str
    user:str
    timestamp: float

@dataclass
class QueueMessage:
    header : QueueMessageHeader
    payload: dict = field(default_factory=dict)

    # @classmethod
    # def from_json(cls: Type["QueueMessage"], json_str: str) -> "QueueMessage":
    #     try:
    #         data = json.loads(json_str)
    #     except json.JSONDecodeError as e:
    #         raise ValueError(f"Ungültiges JSON: {e}")

    #     # Enum-Werte aus Strings rekonstruieren, Fehler falls ungültig
    #     try:
    #         event = SocketEventsFromBackend(data["event"])
    #     except ValueError:
    #         raise ValueError(f"Invalid event in JSON: {data.get('event')}")

    #     timestamp = data.get("timestamp", time.time())
    #     try:
    #         timestamp = float(timestamp)
    #     except Exception:
    #         timestamp = time.time()

    #     data_field = data.get("data", "")
    #     if not isinstance(data_field, str):
    #         data_field = str(data_field)
        
    #     cls.header.event = event
    #     cls.header.timestamp=timestamp
    #     cls.payload = data_field
    #     return cls(header=cls.header,payload=cls.payload)

    # def to_json(self) -> str:
    #     # Enum-Werte als Strings serialisieren
    #     dict_repr = asdict(self)
    #     dict_repr["event"] = self.event.value
    #     return json.dumps(dict_repr)

