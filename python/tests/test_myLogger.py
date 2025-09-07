from multiprocessing import Process, current_process
from myLogger import MyLogger
import unittest
import logging
import threading
import time
import os
from pathlib import Path


class Test_myLogger(unittest.TestCase):
    def setUp(self):
        self.log_file = Path(__file__).parent / "test_Test_myLogger.txt"

        if self.log_file.exists():
            self.log_file.unlink()

        # Dictionary mit Level-Strings als Schlüssel und logging-Level als Wert
        self.testData = {
            "DEBUG": logging.DEBUG,
            "INFO": logging.INFO,
            "WARNING": logging.WARNING,
            "ERROR": logging.ERROR,
            "CRITICAL": logging.CRITICAL,
        }

    def tearDown(self):
        MyLogger.stopAll()  # Stoppt Listener und schließt Handler
        if self.log_file.exists():
            self.log_file.unlink()

    def test_valid_levels(self):
        for level_str, expected_level in self.testData.items():
            with self.subTest(level=level_str):
                logger = MyLogger(
                    "testLogger", sLevel=level_str, sFilePath=self.log_file
                )
                self.assertEqual(logger.level, expected_level)

                # Handler schließen und entfernen
                handlers = logger.handlers[:]
                for handler in handlers:
                    handler.close()
                    logger.removeHandler(handler)

    def test_invalid_level_raises(self):
        with self.assertRaises(ValueError):
            _ = MyLogger("testLogger", sLevel="INVALID")


class TestLoggerWithQueue(unittest.TestCase):
    def setUp(self):
        self.log_file = Path(__file__).parent / "test_TestLoggerWithQueue.txt"

        if self.log_file.exists():
            self.log_file.unlink()
        self.log_file.touch()

        # Logger vorbereiten (wichtig: muss im Hauptprozess initialisiert werden)
        self.logger = MyLogger(
            "queueLogger", sLevel="DEBUG", sFilePath=self.log_file)

    def worker_process(self, index: int):
        # Neue Logger-Instanz im Kindprozess (Filemode 'a' damit nicht überschrieben wird)
        logger = MyLogger(
            "queueLogger", sLevel="DEBUG", sFilePath=self.log_file)
        for i in range(5):
            logger.info(f"Process-{index}: log {i}")
            time.sleep(0.01)

    def test_multiprocess_logging(self):
        processes = [
            Process(target=self.worker_process, args=(i,)) for i in range(3)
        ]
        process_names = []

        for p in processes:
            p.start()
            process_names.append(p.name)

        for p in processes:
            p.join()

        # Listener etwas Zeit geben
        time.sleep(0.3)

        self.assertTrue(os.path.exists(self.log_file),
                        "Logdatei wurde nicht erstellt")

        with open(self.log_file, "r", encoding="utf-8") as f:
            content = f.read()

        # Prüfe, ob alle Prozessnamen im Log gefunden werden
        for name in process_names:
            self.assertIn(f"{name}", content)

        lines = content.strip().splitlines()
        self.assertGreaterEqual(len(lines), 15)  # 3 Prozesse x 5 Logs

    def tearDown(self):
        MyLogger.stopAll()
        if self.log_file.exists():
            self.log_file.unlink()
