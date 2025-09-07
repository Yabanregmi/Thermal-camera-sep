from tb_events import Tb_Event
from logging import Logger

class Tb_Heartbeat(): 
    def __init__(self, logger : Logger, event_heartbeat_server: Tb_Event, event_heartbeat_ir: Tb_Event):
        self.is_init : bool = True

        self.logger = logger
        self.event_heartbeat_server : Tb_Event = event_heartbeat_server
        self.event_heartbeat_ir : Tb_Event = event_heartbeat_ir
        self.pre_cnt : int = 0
        self.logger.debug("Heartbeat init")

    def run(self, cnt : int) -> bool: 
        error : bool = False
        diff : int = abs(cnt -  self.pre_cnt)
        
        if diff > 5:
            self.pre_cnt = cnt
            if not self.event_heartbeat_server.wait(timeout=0):
                self.logger.debug("Server heartbeat error")
                error = True
            if not self.event_heartbeat_ir.wait(timeout=0):
                self.logger.debug("Ir heartbeat error")
                error = True
        return error
