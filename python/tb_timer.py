from typing import Callable
import time
from threading import Thread
from logging import Logger
from tb_events import TimerEvents

class Tb_Timer(Thread):
    """
    Timer-Thread, der nach Ablauf eines Intervalls eine Funktion ausführt.

    Beispiel:
        t = MyTimerThread("Timer1", logger, 30, f, shutdown_event)
        t.start()
        t.abort()    # Stoppt den Timer, falls noch nicht ausgelöst
        t.restart()  # Startet den Timer erneut
    """

    def __init__(self, name: str, logger: Logger, interval_s: int, function: Callable[...,None],
                    events : TimerEvents):
        """
        Initialisiert den Timer-Thread.

        Args:
            name (str): Name des Timers.
            logger (Any): Logger-Objekt.
            interval_s (int): Intervall in Sekunden.
            function (Callable): Auszuführende Funktion nach Ablauf.
            shutdown (MyEvent): Event zum Beenden des Threads.

        Raises:
            ValueError: Bei ungültigen Parametern.
        """
        if function is None:
            raise ValueError("Keine Funktion übergeben")
        if not (3 <= len(name) <= 50):
            raise ValueError("Name muss zwischen 3 und 50 Zeichen lang sein.")
        if interval_s < 0:
            interval_s = 0
            raise ValueError("Timer Intervall < 0")
         
        self._name = name
        self._interval : int = interval_s
        self._function : Callable[...,None] = function
        self._max_timeout :int= 256
        self._logger = logger
        self.events : TimerEvents = events
        super().__init__(name=name)
        self._logger.debug(f"{self.__class__.__name__} - {self.name} init")
        self.run_one_time : bool = False
        
    def restart(self):
        """
        Startet den Timer erneut.
        """
        if not self.events.restart.is_set():
            self.events.restart.set()
            
    def shutdown(self):
        """
        Setzt das Shutdown-Event, um den Thread zu beenden.
        """
        if not self.events.shutdown.is_set():
            self.events.shutdown.set()
            self._logger.debug(f"{self.__class__.__name__} - {self.name} called shutdown")

    def run(self):
        """
        Startet den Timer-Thread und ruft nach Ablauf die Funktion auf.
        """
        self._logger.debug(f"Timer {self.name} run")
        self.run_one_time = True
        while not self.events.shutdown.is_set():
            try:
                if self.run_one_time or self.events.restart.wait(timeout=0):
                    self.run_one_time = False
                    time.sleep(self._interval)
                    self._function()       
            except Exception:
                self._logger.critical(f"{self.__class__.__name__} - {self.name} unkown error")
                
        self.events.restart.clear()
        self.events.shutdown.clear()   
        self._logger.debug(f"{self.__class__.__name__} - {self.name} shutdown")
    