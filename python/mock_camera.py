# mock_camera.py
import numpy as np
import time
import random

class MockCameraController:
    def __init__(self):
        self.palette_width = 80
        self.palette_height = 60
        self.thermal_width = 80
        self.thermal_height = 60
        self.opened = True
        self._t0 = time.time()

    # keep your existing method
    def read_frame(self):
        rgb = np.full((self.palette_height, self.palette_width, 3), 64, dtype=np.uint8)
        if int(time.time() - self._t0) % 7 == 0:
            temp_c = 55.0
        else:
            temp_c = 22.0 + random.uniform(-0.5, 0.5)
        return rgb, temp_c

    # add this alias so IR code works unchanged
    def get_frame(self):
        return self.read_frame()

    # add a shutdown hook since IR code may call cam.shutdown()
    def shutdown(self):
        self.opened = False
