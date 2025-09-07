from dataclasses import dataclass, field
from time import time 
from tb_events import Tb_Event
from tb_queues import Tb_Queue
from models.tb_dataclasses import QueueMessage, QueueTestEvents, QueuesMembers, QueueMessageHeader
from logging import Logger
from abc import ABC

@dataclass
class Tb_QueueTestEvents:
    """
    Sammlung von Events zur Steuerung und Überwachung des Server-Prozesses.
    """
    name: str
    is_started: Tb_Event = field(init=False)
    is_done: Tb_Event = field(init=False)
    is_error: Tb_Event = field(init=False)
    is_okay : Tb_Event = field(init=False)

    def __post_init__(self):
        self.is_started = Tb_Event(name=f"{self.name} start")
        self.is_done = Tb_Event(name=f"{self.name} done")
        self.is_error = Tb_Event(name=f"{self.name} error")
        self.is_okay = Tb_Event(name=f"{self.name} okay")

@dataclass
class Tb_QueueTestFlags():
    set : bool = False
    error : bool  = False
    done : bool = False

class Tb_QueueTest(ABC): 
    def __init__(self,logger : Logger, events :Tb_QueueTestEvents, queue:Tb_Queue):
        self.name : str
        self.logger : Logger = logger
        self.events : Tb_QueueTestEvents = events
        self.queue : Tb_Queue = queue
        self.pre_cnt : int = 0
        self.started : bool = False
        self.error : bool = False
        self.source : QueuesMembers
        self.dest : QueuesMembers
        self.msg_event_req : QueueTestEvents
        self.msg_event_res : QueueTestEvents

    def start(self) -> None:
        if not self.events.is_started.is_set() and not self.events.is_error.is_set():
            header : QueueMessageHeader = QueueMessageHeader(source=self.source, dest = self.dest, event =self.msg_event_req,id="",user="",timestamp=time())
            msg : QueueMessage = QueueMessage(header = header, payload={})
            if not self.queue.put(item=msg):
                self.logger.error(f"QueueTest{self.name}: Queue full")
                self.events.is_error.set()
            else:
                self.events.is_started.set()
                self.logger.debug(f"QueueTest{self.name}: Set")
            
    def verfiy_response(self, msg : QueueMessage)  -> None:
        if msg.header.dest == QueuesMembers.MAIN and msg.header.event == self.msg_event_res:
            self.events.is_started.clear()
            self.events.is_done.set()
            self.logger.debug(f"QueueTest{self.name}: Done")

class Tb_QueueTestMain(Tb_QueueTest): 
    def __init__(self, logger :Logger, queue : Tb_Queue, events : Tb_QueueTestEvents):
        super().__init__(logger=logger,events=events,queue=queue)
        self.name : str = "Main"
        self.source = QueuesMembers.MAIN
        self.dest = QueuesMembers.MAIN
        self.msg_event_req = QueueTestEvents.REQ_FROM_MAIN_TO_MAIN
        self.msg_event_res = QueueTestEvents.REQ_FROM_MAIN_TO_MAIN
        self.logger.debug(f"QueueTest{self.name} init")

class Tb_QueueTestServer(Tb_QueueTest): 
    def __init__(self, logger :Logger, queue : Tb_Queue, events : Tb_QueueTestEvents):
        super().__init__(logger=logger,events=events,queue=queue)
        self.name :str = "Server"
        self.source = QueuesMembers.MAIN
        self.dest = QueuesMembers.SERVER
        self.msg_event_req = QueueTestEvents.REQ_FROM_MAIN_TO_SERVER
        self.msg_event_res = QueueTestEvents.ACK_FROM_SERVER_TO_MAIN
        self.logger.debug(f"QueueTest{self.name} init")
    
class Tb_QueueTestIr(Tb_QueueTest): 
    def __init__(self, logger :Logger, queue : Tb_Queue, events : Tb_QueueTestEvents):
        super().__init__(logger=logger,events=events,queue=queue)  
        self.name :str = "Ir"
        self.source = QueuesMembers.MAIN
        self.dest = QueuesMembers.IR
        self.msg_event_req = QueueTestEvents.REQ_FROM_MAIN_TO_IR
        self.msg_event_res = QueueTestEvents.ACK_FROM_IR_TO_MAIN
        self.logger.debug(f"QueueTest{self.name} init")

