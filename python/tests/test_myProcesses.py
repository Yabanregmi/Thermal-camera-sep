from typing import Dict, Type
import unittest
import multiprocessing
import time
from my_processes import MyEvent, MyQueue, ProcessManager, ServerProcess, MyTimerThread

class TestProcessManager(unittest.TestCase):
    def setUp(self):
        self.queue_server = MyQueue()
        self.queue_io = MyQueue()
        self.queue_db = MyQueue()
        self.event_shutdown = MyEvent()
        self.event_heartbeat = MyEvent()
        self.handler_process_server: ServerProcess = ServerProcess

    def test_process_lifecycle(self):

        # Prozess initialisieren
        name = "test_process"
        ProcessManager.init_process(
            name="test_process",
            queue_server=self.queue_server,
            queue_io=self.queue_io,
            queue_db=self.queue_db,
            event_shutdown=self.event_shutdown,
            event_heartbeat=self.event_heartbeat,
            handler_process=self.handler_process_server
        )

        # Prozess starten
        ProcessManager.start_process(name=name)
        time.sleep(0.2)  # Prozess kurz laufen lassen
        self.assertTrue(ProcessManager.is_alive(name=name))

        time.sleep(2)
        self.assertTrue(ProcessManager.check_heartbeat(name=name))

        # Prozess sauber beenden
        ProcessManager.shutdown_process(name=name)

        self.assertFalse(ProcessManager.is_alive(name=name))
