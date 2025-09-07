import time
from threading import Thread
from logging import Logger
from tb_events import UserInputsEvents

class Tb_UserInput(Thread):
    """
    Thread zur Verarbeitung von Benutzereingaben, speziell zum Abbruch oder Herunterfahren.
    """
    def __init__(self, name : str, logger : Logger, events : UserInputsEvents):
        """
        Initialisiert den user_input-Thread.

        Args:
            logger (Any): Logger-Objekt.
            aborted (MyEvent): Event für Benutzerabbruch.
            shutdown (MyEvent): Event für Shutdown.
        """
        super().__init__(name="user_input")
        self.name = name
        self.logger = logger
        self.events : UserInputsEvents = events
        self.logger.debug(f"{self.__class__.__name__} - {self.name} init")

    def shutdown(self):
        """
        Setzt das Shutdown-Event, um den Thread zu beenden.
        """
        if not self.events.shutdown.is_set():
            self.events.shutdown.set()
            self.logger.debug(f"{self.__class__.__name__} - {self.name} called shutdown")

    def run(self):
        """
        Wartet auf Benutzereingabe ('q'), um das Abbruch-Event zu setzen.
        """
        user_input = input()
        if user_input == 'q':
            self.events.aborted.set()
        else:
            self.events.aborted.set()
            self.logger.debug(f"{self.__class__.__name__} - {self.name} Bitte q abbruch durchgeführt, bitte q beim nächsten mal verwenden")
        self.logger.debug(f"{self.__class__.__name__} - {self.name} user pressed {user_input}")
        time.sleep(0.1)
            
        self.events.shutdown.wait(timeout=None)

        if self.events.shutdown.is_set():
            self.events.shutdown.clear()
        
        if self.events.aborted.is_set():
            self.events.aborted.clear()
        self.logger.debug(f"{self.__class__.__name__} - {self.name} shutdown")
