import smbus2 as smbus
from typing import List
from logging import Logger
class Tb_Relay:
    # Konstanten
    NUM_RELAY_PORTS: int = 4
    DEVICE_ADDRESS: int = 0x24
    DEVICE_CONFIGURATION_REGISTER: int = 0x06
    DEVICE_OUTPUT_PORT_REGISTER: int = 0x02
    DEVICE_SET_PINS_AS_OUTPUTS: int = 0x00
    device_data: int = 0x00

    # Statusflags (global für alle Relais im System!)
    relay_1_is_set: bool = False
    relay_2_is_set: bool = False
    relay_3_is_set: bool = False
    relay_4_is_set: bool = False

    # Logger und Bus (müssen vor Benutzung gesetzt werden)
    logger: Logger
    bus: smbus.SMBus
    
    relais_names : List[str] = ["","","",""]
    
    def __init__(self, logger: Logger, bus_nr: int = 1, relais_names : List[str] = ["","","",""]) -> None:
        self.logger : Logger = logger
        self.bus : smbus.SMBus = smbus.SMBus(bus_nr)
        self.logger.warning(f"ADR: {self.DEVICE_ADDRESS}, Ports {self.NUM_RELAY_PORTS}")
        self._reset_register()
        
        
        if len(relais_names) > 4:
            self.relais_names = relais_names
        else:
            self.relais_names = ["Relais1","Relais2","Relais3","Relais4"]
        
        self.all_off()

    def _reset_register(self) -> None:
        self.logger.warning('Reset relay register')
        self.bus.write_byte_data(self.DEVICE_ADDRESS, self.DEVICE_CONFIGURATION_REGISTER, self.DEVICE_SET_PINS_AS_OUTPUTS)

    def on_1(self) -> None:
        if not self.relay_1_is_set:
            self._relay_on(relay_num=1, name=self.relais_names[0])
            self.relay_1_is_set = True

    def on_2(self) -> None:
        if not self.relay_2_is_set:
            self._relay_on(relay_num=2, name=self.relais_names[1])
            self.relay_2_is_set = True

    def on_3(self) -> None:
        if not self.relay_3_is_set:
            self._relay_on(relay_num=3, name=self.relais_names[2])
            self.relay_3_is_set = True

    def on_4(self) -> None:
        if not self.relay_4_is_set:
            self._relay_on(relay_num=4, name=self.relais_names[3])
            self.relay_4_is_set = True

    def off_1(self) -> None:
        if self.relay_1_is_set:
            self._relay_off(relay_num=1, name=self.relais_names[0])
            self.relay_1_is_set = False

    def off_2(self) -> None:
        if self.relay_2_is_set:
            self._relay_off(relay_num=2, name=self.relais_names[1])
            self.relay_2_is_set = False

    def off_3(self) -> None:
        if self.relay_3_is_set:
            self._relay_off(relay_num=3, name=self.relais_names[2])
            self.relay_3_is_set = False

    def off_4(self) -> None:
        if self.relay_4_is_set:
            self._relay_off(relay_num=4, name=self.relais_names[3])
            self.relay_4_is_set = False

    def all_on(self) -> None:
        self.on_1()
        self.on_2()
        self.on_3()
        self.on_4()

    def all_off(self) -> None:
        self.off_1()
        self.off_2()
        self.off_3()
        self.off_4()

    def _relay_on(self, relay_num: int, name : str) -> None:
        if 0 < relay_num <= self.NUM_RELAY_PORTS:
            self.logger.warning(f'Relay {name} ON')
            self.device_data |= (0x1 << (relay_num - 1))
            self.bus.write_byte_data(self.DEVICE_ADDRESS, self.DEVICE_OUTPUT_PORT_REGISTER, self.device_data)
        else:
            self.logger.warning(f'Invalid relay #: {name}')

    def _relay_off(self, relay_num: int, name : str) -> None:
        if 0 < relay_num <= self.NUM_RELAY_PORTS:
            self.logger.warning(f'Relay {name} OFF')
            self.device_data &= ~(0x1 << (relay_num - 1))
            self.bus.write_byte_data(self.DEVICE_ADDRESS, self.DEVICE_OUTPUT_PORT_REGISTER, self.device_data)
        else:
            self.logger.warning(f'Invalid relay #: {name}')

    def relay_toggle_port(self, relay_num: int, name : str) -> None:
        self.logger.warning(f'Toggling relay: {name}')
        if self.relay_get_port_status(relay_num):
            self._relay_off(relay_num, name)
        else:
            self._relay_on(relay_num, name)

    def relay_get_port_status(self, relay_num: int) -> bool:
        self.logger.warning(f'Checking status of relay {relay_num}')
        res = self.relay_get_port_data(relay_num)
        if res > 0:
            mask = 1 << (relay_num - 1)
            return (self.device_data & mask) == 0
        else:
            self.logger.warning("Specified relay port is invalid")
            return False
        
    def relay_get_port_data(self, relay_num: int) -> int:
        self.logger.warning(f'Reading relay status value for relay {relay_num}')
        if 0 < relay_num <= self.NUM_RELAY_PORTS:
            device_reg_data = self.bus.read_byte_data(self.DEVICE_ADDRESS, self.DEVICE_OUTPUT_PORT_REGISTER)
            return device_reg_data
        else:
            self.logger.warning("Specified relay port is invalid")
            return 0