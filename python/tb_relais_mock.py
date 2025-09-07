class Tb_Relay:
    def __init__(self, logger, bus_nr=1):
        self.logger = logger
        self.state = False
        self.logger.debug("Mock relay initialized")

    def on_1(self):
        if not self.state:
            self.state = True
            self.logger.debug("Relay 1 ON (mock)")

    def off_1(self):
        if self.state:
            self.state = False
            self.logger.debug("Relay 1 OFF (mock)")
