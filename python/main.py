from dataclasses import dataclass
import time
from tb_events import Tb_Event
from tb_logger import TbLogger
from sys import exit
from tb_main_helper import AppContext
from logging import DEBUG, Logger
from tb_queue_test import Tb_QueueTest
from pathlib import Path

@dataclass
class Errors():
    heartbeat : bool = False
    server : bool = False
    ir: bool = False
    test_queues_main : bool = False
    test_queues_server : bool = False
    test_queues_ir : bool = False
    
    def is_error(self) -> bool:
        status : bool = False
        if self.heartbeat or self.server or self.ir or self.test_queues_main  or self.test_queues_server  or self.test_queues_ir:
            status = True
        else:
            status = False
        return status

"""
main.py
--------
Startpunkt der Anwendung:
- Initialisiert und verwaltet alle Prozesse (Server, IO, DB)
- Sorgt für zentrales Logging
- Überwacht Heartbeats der Subprozesse im eingestellten Intervall
- Führt einen sauberen Shutdown aller Prozesse per Tastendruck ('q') durch
"""

def start_queue_test(queueTest : Tb_QueueTest, counter : int) -> None:
    if not queueTest.started and abs(counter - queueTest.pre_cnt) > 5:
        queueTest.pre_cnt = counter
        queueTest.started = True
        queueTest.start()

def check_queue_test(logger: Logger,queueTest : Tb_QueueTest, counter : int) -> None:
    if queueTest.started and abs(counter - queueTest.pre_cnt) > 5:
        queueTest.pre_cnt = counter
        if not queueTest.error and not queueTest.events.is_done.is_set():
            queueTest.events.is_error.set()
            queueTest.error = True
            logger.debug(f"QueueTest {queueTest.name}: error")
        elif queueTest.error:
            queueTest.events.is_done.clear()
            queueTest.error = False
            queueTest.events.is_okay.set()
            logger.debug(f"QueueTest {queueTest.name}: okay")
        queueTest.started = False

def main ():
    counter : int = 0
    loop_forever : bool = False
    errors = None 
    errors = Errors()
    app = None
    
    try:
        event_logger_stop : Tb_Event = Tb_Event(name="event_logger_stop")
        TbLogger.configure(logfile=Path("log.txt"),loglevel=DEBUG,event_stop=event_logger_stop)
        TbLogger.start()
        logger_main : Logger = TbLogger.get_logger(name = "logger_main_process")

        app = AppContext(logger=logger_main)
        app.start()

        loop_forever = True

        while loop_forever:
            for i in range(0,20): # type: ignore
                msg = app.main_queues.main.get() 
                if msg is None:
                    break
                
                if app.queue_test_main.started and app.queue_test_main.events.is_started.is_set():
                    app.queue_test_main.verfiy_response(msg=msg)

                if app.queue_test_server.started and app.queue_test_server.events.is_started.is_set():
                    app.queue_test_server.verfiy_response(msg=msg)

                if app.queue_test_ir.started and app.queue_test_ir.events.is_started.is_set():
                    app.queue_test_ir.verfiy_response(msg=msg)
            
            start_queue_test(queueTest=app.queue_test_main,counter=counter)
            start_queue_test(queueTest=app.queue_test_server,counter=counter)
            start_queue_test(queueTest=app.queue_test_ir,counter=counter)

            check_queue_test(logger=logger_main,queueTest=app.queue_test_main,counter=counter)

            check_queue_test(logger=logger_main,queueTest=app.queue_test_server,counter=counter)
            check_queue_test(logger=logger_main,queueTest=app.queue_test_ir,counter=counter)
            

            if app.queue_test_main.events.is_okay.wait(timeout=0):
                errors.test_queues_main = False # type: ignore

            if app.queue_test_main.events.is_error.wait(timeout=0):
                errors.test_queues_main = True # type: ignore

        
            if app.queue_test_server.events.is_okay.wait(timeout=0):
                errors.test_queues_server = False # type: ignore

            if app.queue_test_server.events.is_error.wait(timeout=0):
                errors.test_queues_server = True # type: ignore

                
            if app.queue_test_ir.events.is_okay.wait(timeout=0):
                errors.test_queues_ir = False # type: ignore

            if app.queue_test_ir.events.is_error.wait(timeout=0):
                errors.test_queues_ir = True # type: ignore

            errors.heartbeat = app.heartbeat.run(cnt=counter)
            
            # Error handling - Server process   
            if not errors.server and app.events_server.error_from_server_process.wait(timeout=0):
                app.logger_main.debug("Server process error")
                errors.server = True
            elif errors.server and app.events_server.server_process_okay.wait(timeout=0):
                app.logger_main.debug("Server process okay")
                errors.server = False

            if errors.is_error():
                app.logger_main.debug(f"Sammelstörung: {errors}")
                app.relays.off_1()
            else:
                app.relays.on_1()
                
            if app.events_user_input.aborted.is_set():
                app.events_user_input.aborted.clear()
                loop_forever = False
            
            counter = (counter + 1) % 21
            time.sleep(1)
    except Exception as e:
        print(f"Critical process error: {e}")
        TbLogger.get_logger(name="logger_main_process").exception(e)
    finally:     
        if app is not None:
            try:
                app.shutdown_all()
            except Exception as e:
                print(f"[MAIN] Error while shutting down app: {e}")
        TbLogger.stop()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[SYSTEM] Uncaught error in __main__: {e}")
        raise
    finally:
        time.sleep(1)
        exit(0)
        

    