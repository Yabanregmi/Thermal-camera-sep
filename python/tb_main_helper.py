from typing import Optional
from logging import Logger
from tb_logger import TbLogger
from config import SERVER_URL
from tb_relais import Tb_Relay
from tb_queue_test import Tb_QueueTestMain, Tb_QueueTestServer, Tb_QueueTestIr, Tb_QueueTestEvents
from tb_queues import MainQueues, SocketQueues, Tb_Queue
from tb_events import ServerEvents, IrEvents, UserInputsEvents, Tb_Event
from tb_server_process import Tb_ServerProcess
from tb_ir_process import Tb_IrProcess
from tb_user_input import Tb_UserInput
from tb_heartbeat import Tb_Heartbeat
import os


USE_MOCK_RELAIS = os.environ.get("USE_MOCK_RELAIS", "1") == "1"  # default mock during tests

if USE_MOCK_RELAIS:
    from tb_relais_mock import Tb_Relay
else:
    from tb_relais import Tb_Relay

class AppContext:
    _instance: Optional["AppContext"] = None
    logger_main : Logger

    def __new__(cls, logger: Logger) -> "AppContext":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
            cls._instance.logger_main = logger
        return cls._instance

    def __init__(self, logger: Logger):
        if self._initialized:
            raise RuntimeError("AppContext wurde bereits initialisiert")

        self.logger_main: Logger = logger

        try:
            self.events_server = self._init_events_server()
            self.events_ir = self._init_events_ir()
            self.events_user_input = self._init_events_user_input()

            self.heartbeat = self._init_heartbeat(event_heartbeat_server =self.events_server.heartbeat, event_heartbeat_ir = self.events_ir.heartbeat)
            self.main_queues = self._init_main_queues()
            self.socket_queues = self._init_socket_queues()
            
            self.server_process = self._init_server_process(main_queues=self.main_queues,socket_queues=self.socket_queues, events= self.events_server)
            self.ir_process = self._init_ir_process(main_queues=self.main_queues,socket_queues=self.socket_queues, events= self.events_ir)
            self.thread_user_input = self._init_user_input(events=self.events_user_input)
            self.relays = self._init_relais()

            self.events_queue_test_main = self._init_events_queue_test(name="Main")
            self.queue_test_main = self._init_queue_test_main(queue=self.main_queues.main,events=self.events_queue_test_main)

            self.events_queue_test_server = self._init_events_queue_test(name="Server")
            self.queue_test_server = self._init_queue_test_server(queue=self.main_queues.server,events=self.events_queue_test_server)

            self.events_queue_test_ir = self._init_events_queue_test(name="Ir")
            self.queue_test_ir = self._init_queue_test_ir(queue=self.main_queues.ir,events=self.events_queue_test_ir)
        except Exception as e:
            raise RuntimeError("Fehler bei der Initialisierung der Appself-Komponenten") from e

        self._initialized = True

    
    def start(self):
        try:
            self._start_server_process(self.server_process)
            self._start_ir_process(self.ir_process)
            self._start_user_input(self.thread_user_input)
        except Exception as e:
            raise RuntimeError("Fehler beim Starten der Appself-Komponenten") from e
        
    def shutdown_all(self):
        print("Appcontext del")
        self.relays.off_1()
        self.ir_process.shutdown()
        self.ir_process.join()
        self.server_process.shutdown()
        self.server_process.join()
        self.main_queues.main.shutdown()
        self.main_queues.server.shutdown()
        self.main_queues.ir.shutdown()
        self.main_queues.main.join()
        self.main_queues.server.join()
        self.main_queues.ir.join()
        self.thread_user_input.shutdown()
        self.thread_user_input.join()
        self.logger_main.debug("App stop")

    def _init_events_server(self) -> ServerEvents:
        try:
            return ServerEvents(name="Server process")
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren von Server Events") from e

    def _init_events_ir(self) -> IrEvents:
        try:
            return IrEvents(name="Ir process")
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren von IR Events") from e

    def _init_events_user_input(self) -> UserInputsEvents:
        try:
            return UserInputsEvents(name="User Input")
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren von User Input Events") from e

    def _init_events_queue_test(self, name:str) -> Tb_QueueTestEvents:
        try:
            return Tb_QueueTestEvents(name=name)
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren von Queue Test Events") from e

    def _init_heartbeat(self, event_heartbeat_server: Tb_Event,event_heartbeat_ir: Tb_Event) -> Tb_Heartbeat:
        try:
            logger_heartbeat = TbLogger.get_logger("logger_heartbeat")
            return Tb_Heartbeat(logger=logger_heartbeat, event_heartbeat_server = event_heartbeat_server, event_heartbeat_ir=event_heartbeat_ir)
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren des Heartbeats") from e

    def _init_main_queues(self) -> MainQueues:
        try:
            main_queues = MainQueues()
            logger_main = TbLogger.get_logger("logger_queue_main")
            logger_server = TbLogger.get_logger("logger_queue_server")
            logger_ir = TbLogger.get_logger("logger_queue_ir")
            main_queues.init(logger_main=logger_main, logger_server=logger_server, logger_ir=logger_ir)
            return main_queues
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren der MainQueues") from e
    
    def _init_socket_queues(self) -> SocketQueues:
        try:
            socket_queues = SocketQueues()
            logger_server_to_ir = TbLogger.get_logger("logger_server_to_ir")
            logger_ir_to_server = TbLogger.get_logger("logger_ir_to_server")

            socket_queues.init(logger_server_to_ir = logger_server_to_ir, logger_ir_to_server = logger_ir_to_server)
            return socket_queues
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren der MainQueues") from e

    def _init_server_process(self, main_queues: MainQueues, socket_queues : SocketQueues,events: ServerEvents) -> Tb_ServerProcess:
        try:
            logger_server = TbLogger.get_logger("logger_server_process")
            logger_backend = TbLogger.get_logger("logger_backend")
            return Tb_ServerProcess(name="server_process", logger=logger_server,logger_backend=logger_backend, url=SERVER_URL, events=events, main_queues=main_queues, socket_queues = socket_queues)
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren des Server-Prozesses") from e

    def _init_ir_process(self, main_queues: MainQueues, socket_queues : SocketQueues, events: IrEvents) -> Tb_IrProcess:
        try:
            logger_ir = TbLogger.get_logger("logger_ir_process")
            return Tb_IrProcess(name="ir_process", logger=logger_ir, events=events, main_queues=main_queues, socket_queues = socket_queues)
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren des Ir-Prozesses") from e

    def _init_user_input(self, events: UserInputsEvents) -> Tb_UserInput:
        try:
            logger = TbLogger.get_logger("logger_user_input")
            return Tb_UserInput(name="User_Input", logger=logger, events=events)
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren des User Inputs Thread") from e

    def _init_relais(self) -> Tb_Relay:
        try:
            logger = TbLogger.get_logger("logger_relais")
            return Tb_Relay(logger=logger, bus_nr=1)
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren der Relais") from e

    def _init_queue_test_main(self, queue : Tb_Queue, events: Tb_QueueTestEvents) -> Tb_QueueTestMain:
        try:
            logger = TbLogger.get_logger("logger_queue_test_main")
            return Tb_QueueTestMain(logger=logger, queue=queue, events=events)
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren des Queue-Test-Main") from e

    def _init_queue_test_server(self, queue : Tb_Queue, events: Tb_QueueTestEvents) -> Tb_QueueTestServer:
        try:
            logger = TbLogger.get_logger("logger_queue_test_server")
            return Tb_QueueTestServer(logger=logger, queue=queue, events=events)
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren des Queue-Test-Server") from e

    def _init_queue_test_ir(self, queue : Tb_Queue, events: Tb_QueueTestEvents) -> Tb_QueueTestIr:
        try:
            logger = TbLogger.get_logger("logger_queue_test_ir")
            return Tb_QueueTestIr(logger=logger, queue=queue, events=events)
        except Exception as e:
            raise ValueError("Fehler beim Initialisieren des Queue-Test-Ir") from e

    # --- private Start-Methoden ---
    def _start_server_process(self, server: Tb_ServerProcess):
        try:
            server.start()
        except Exception as e:
            raise RuntimeError("Fehler beim Starten des Server Prozesses") from e

    def _start_ir_process(self, ir: Tb_IrProcess):
        try:
            ir.start()
        except Exception as e:
            raise RuntimeError("Fehler beim Starten des Ir Prozesses") from e

    def _start_user_input(self, thread: Tb_UserInput):
        try:
            thread.start()
        except Exception as e:
            raise RuntimeError("Fehler beim Starten des User-Input") from e