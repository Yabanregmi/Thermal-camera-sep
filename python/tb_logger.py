import logging
import logging.handlers
from multiprocessing import Queue, Process
from pathlib import Path
import time
from logging import Logger
from typing import Literal
from tb_events import Tb_Event

LOG_LEVEL = Literal[0,10,20,30,40,50]

class TbLogger:
    @classmethod
    def configure(cls, logfile: Path, loglevel : LOG_LEVEL, event_stop : Tb_Event):
        cls.log_queue : Queue = Queue(-1) # type: ignore
        cls.logfile : Path = logfile
        cls.loglevel : LOG_LEVEL = loglevel
        cls.listener_process = Process(
            target =cls._listener_process_target, # type: ignore
            args=(cls.log_queue, cls.logfile), # type: ignore
            daemon=True
        )
        cls.event_stop : Tb_Event = event_stop

    @classmethod
    def _configure_handlers(cls, logfile: Path) -> list[logging.Handler]:
        formatter = logging.Formatter(
            fmt="%(asctime)s.%(msecs)03d - %(levelname)s - %(processName)s - %(module)s.%(funcName)s:%(lineno)d - %(message)s",
            datefmt="%d-%m-%Y %H:%M:%S",
        )
        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)
        file_handler = logging.FileHandler(logfile, encoding="utf-8")
        file_handler.setFormatter(formatter)
        return [stream_handler, file_handler]

    @classmethod
    def _listener_process_target(cls, log_queue : Queue, logfile : Path) -> None: # type: ignore
        handlers = cls._configure_handlers(logfile)
        listener = logging.handlers.QueueListener(log_queue, *handlers) # type: ignore
        try:
            listener.start()
            while not cls.event_stop.wait(timeout=0):
                time.sleep(0.1)
        except Exception:
            pass
        finally:
            listener.stop()

    @classmethod
    def start(cls):
        if not cls.listener_process:
            raise RuntimeError("TbLogger not configured. Call configure() first.")
        else :
            cls.listener_process.start()

    @classmethod
    def get_logger(cls, name : str) -> Logger:
        if not cls.log_queue: # type: ignore
            raise RuntimeError("TbLogger not configured. Call configure() first.")
        logger = logging.getLogger(name)
        logger.setLevel(cls.loglevel)
        logger.handlers = []
        logger.propagate = False
        logger.addHandler(logging.handlers.QueueHandler(cls.log_queue)) # type: ignore
        return logger

    @classmethod
    def stop(cls):
        if cls.listener_process:
            cls.event_stop.set()
            cls.listener_process.join()