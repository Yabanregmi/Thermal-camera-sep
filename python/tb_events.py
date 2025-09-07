# events.py
import multiprocessing

class Tb_Event:
    """
    Wrapper-Klasse für multiprocessing.Event, um einen 

    einheitlichen Umgang mit Events zu ermöglichen.
    """

    def __init__(self, name:str) -> None:
        """
        Initialisiert das interne Event-Objekt.
        """
        self._event = multiprocessing.Event()
        self.name = name

    def is_set(self) -> bool:
        """
        Prüft, ob das Event gesetzt ist.

        Returns:
            bool: True, wenn das Event gesetzt ist, sonst False.
        """
        status : bool = True
        try:
            status = self._event.is_set()
        except Exception:
            status = False
        return status

    def set(self) -> bool:
        """
        Setzt das Event.
        """
        status : bool = True
        try:
            self._event.set()
        except Exception:
            status = False
        return status
        
    def clear(self) -> bool:
        """
        Löscht das Event (setzt es zurück).
        """
        status : bool = True
        try:
            self._event.clear()
        except Exception:
            status = False
        return status
        
    def wait(self, timeout: int | None) -> bool:
        """
        Wartet, bis das Event gesetzt wird oder das Timeout abläuft.

        Args:
            timeout (Optional[int]): Maximale Wartezeit in Sekunden.

        Returns:
            bool: True, wenn das Event gesetzt wurde, sonst False.
        """
        if timeout == None:
            print(f"{__class__} :{self.name} Event wait unendlich")
        status : bool = False
        try:
            _timeout = timeout
            if not _timeout == None: 
                _timeout = float(_timeout)
            status = self._event.wait(timeout = _timeout)
            
            if status:
                self._event.clear()
        except Exception:
            status = False
        return status

class ServerEvents():
    """
    Sammlung von Events zur Steuerung und Überwachung des Server-Prozesses.
    """
    def __init__(self, name:str):
        self.name : str = name
        self.shutdown: Tb_Event = Tb_Event(name = f"{name} shutdown")
        self.heartbeat: Tb_Event = Tb_Event(name = f"{name} heartbeat")
        self.connect: Tb_Event = Tb_Event(name = f"{name} connect")
        self.disconnect: Tb_Event = Tb_Event(name = f"{name} disconnect")
        self.message: Tb_Event = Tb_Event(name = f"{name} message")
        self.alarm: Tb_Event = Tb_Event(name = f"{name} alarm")
        self.error_on_connection: Tb_Event = Tb_Event(name = f"{name} error_on_connection")
        self.error_from_server_process: Tb_Event = Tb_Event(name = f"{name} error_from_process")
        self.server_process_okay: Tb_Event = Tb_Event(name = f"{name} process_okay")

class IrEvents():
    """
    Sammlung von Events zur Steuerung und Überwachung des Server-Prozesses.
    """
    def __init__(self, name:str):
        self.name : str = name
        self.shutdown: Tb_Event = Tb_Event(name = f"{name} shutdown")
        self.heartbeat: Tb_Event = Tb_Event(name = f"{name} heartbeat")
        self.error: Tb_Event = Tb_Event(name = f"{name} error")
        self.okay: Tb_Event =  Tb_Event(name = f"{name} okay")
        
class TimerEvents():
    """
    Sammlung von Events zur Steuerung und Überwachung des Server-Prozesses.
    """
    def __init__(self, name:str):
        self.name : str = name
        self.shutdown: Tb_Event =  Tb_Event(name = f"{name} shutdown")
        self.restart: Tb_Event =  Tb_Event(name = f"{name} restart")

class UserInputsEvents():
    """
    Sammlung von Events zur Steuerung und Überwachung des Server-Prozesses.
    """
    def __init__(self, name:str):
        self.name : str = name
        self.aborted: Tb_Event = Tb_Event(name = f"{name} aborted")
        self.shutdown: Tb_Event = Tb_Event(name = f"{name} shutdown")