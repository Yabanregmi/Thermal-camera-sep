from queue import Empty, Full
from multiprocessing import Queue
from models.tb_dataclasses import QueueMessage
from logging import Logger

QUEUE_MAXSIZE: int = 128

class Tb_Queue:
    """
    Wrapper für multiprocessing.Queue mit fester Maximalgröße.
    """
    
    def __init__(self, name : str, logger : Logger) -> None:
        """
        Initialisiert die interne Queue mit fester Maximalgröße.
        """
        try:
            self.name = name
            self._queue : Queue[QueueMessage] = Queue(maxsize=QUEUE_MAXSIZE)
            self.logger = logger
        except Exception:
            self.logger.critical(f"Error - {self.__class__.__name__} - {self.name} unkown error")

    def put(self, item: QueueMessage) -> bool:
        """
        Fügt ein Element zur Queue hinzu, falls Platz ist.
        Args:
            item (QueueMessage): Das hinzuzufügende Element.
        Returns:
            bool: True, wenn das Element hinzugefügt wurde, sonst False.
        """
        status : bool = False
        try:
            self._queue.put_nowait(item)
            status =  True
        except ValueError:
            self.logger.critical(f"Error - {self.__class__.__name__} - {self.name} is closed")
        except Full:
            self.logger.critical(f"Error - {self.__class__.__name__} - {self.name} is full")
        except Exception:
            self.logger.critical(f"Error - {self.__class__.__name__} - {self.name} unkown error")
        return status

    def get(self) -> QueueMessage | None:
        """
        Entnimmt ein Element aus der Queue.
        Returns:
            Optional[QueueMessage]: Das entnommene Element oder None, falls leer.
        """
        msg : QueueMessage | None = None
        try:
            msg = self._queue.get_nowait()
        except ValueError:
            self.logger.critical(f"Error - {self.__class__.__name__} - {self.name} is closed")
        except Empty:
            pass
            #self.logger.critical(f"Error - {self.__class__.__name__} - {self.name} is empty")
        except Exception:
            self.logger.critical(f"Error - {self.__class__.__name__} - {self.name} unkown error")
        return msg

    def shutdown(self) -> None:
        self.logger.debug(f"{self.__class__.__name__} - {self.name} shutdown")
        try:
            self._queue.close()
        except Exception:
            self.logger.critical(f"Error - {self.__class__.__name__} - {self.name} unkown error")
        
    def join(self) -> None:
        self.logger.debug(f"{self.__class__.__name__} - {self.name} join")
        try:
            self._queue.join_thread()
        except Exception:
            self.logger.critical(f"Error - {self.__class__.__name__} - {self.name} unkown error")

class Tb_Queue_Logger(Tb_Queue):
    def __init__(self, name : str) -> None:
        """
        Initialisiert die interne Queue mit fester Maximalgröße.
        """
        self.name = name
        self._queue : Queue[QueueMessage] = Queue(maxsize=QUEUE_MAXSIZE)
        
class Tb_WriteOnlyQueue:
    """
    Queue-Wrapper, der ausschließlich das Schreiben (put) erlaubt.
    """

    def __init__(self, queue: Tb_Queue) -> None:
        """
        Initialisiert die WriteOnlyQueue mit einer existierenden MyQueue.
        """
        self._queue: Tb_Queue = queue

    def put(self, item: QueueMessage, block: bool = False, timeout: float | None = 0) -> bool:
        """
        Fügt ein Element zur Queue hinzu.

        Args:
            item (QueueMessage): Das hinzuzufügende Element.
            block (bool): Ob blockierend gewartet werden soll.
            timeout (Optional[float]): Maximale Wartezeit.
        """
        return self._queue.put(item=item)

    def get(self) -> QueueMessage | None:
        """
        Das Lesen aus der Queue ist nicht erlaubt.

        Raises:
            RuntimeError: Diese Queue ist nur zum Schreiben.
        """
        raise RuntimeError("This queue is write-only!")


class Tb_ReadOnlyQueue:
    """
    Queue-Wrapper, der ausschließlich das Lesen (get) erlaubt.
    """

    def __init__(self, queue: Tb_Queue) -> None:
        """
        Initialisiert die ReadOnlyQueue mit einer existierenden MyQueue.
        """
        self._queue: Tb_Queue = queue

    def get(self) -> QueueMessage | None:
        """
        Entnimmt ein Element aus der Queue.

        Args:
            block (bool): Ob blockierend gewartet werden soll.
            timeout (Optional[float]): Maximale Wartezeit.

        Returns:
            QueueMessage: Das entnommene Element.
        """
        return self._queue.get()

    def put(self) -> None:
        """
        Das Schreiben in die Queue ist nicht erlaubt.

        Raises:
            RuntimeError: Diese Queue ist nur zum Lesen.
        """
        raise RuntimeError("This queue is read-only!")
    
class MainQueues:
    @classmethod
    def init(cls, logger_main : Logger, logger_server : Logger, logger_ir : Logger):
        cls.logger_main =logger_main
        cls.logger_server = logger_server
        cls.logger_ir = logger_ir

        cls.main = Tb_Queue(name = "main", logger=cls.logger_main)
        cls.server = Tb_Queue( name = "server", logger=cls.logger_server)
        cls.ir = Tb_Queue(name = "ir", logger=cls.logger_ir)

class SocketQueues:
    @classmethod
    def init(cls, logger_server_to_ir : Logger, logger_ir_to_server : Logger):
        cls.logger_server_to_ir = logger_server_to_ir
        cls.logger_ir_to_server = logger_ir_to_server

        cls.server_to_ir = Tb_Queue(name = "server_to_ir", logger=cls.logger_server_to_ir)
        cls.ir_to_server = Tb_Queue( name = "ir_to_server", logger=cls.logger_ir_to_server)
