import ctypes as ct
import numpy as np
import os
from ctypes.util import find_library
import cv2
import random  # For simulated temperature

# --- Frame metadata structure for thermal SDK ---
class EvoIRFrameMetadata(ct.Structure):
    _fields_ = [
        ("counter", ct.c_uint),
        ("counterHW", ct.c_uint),
        ("timestamp", ct.c_longlong),
        ("timestampMedia", ct.c_longlong),
        ("flagState", ct.c_int),
        ("tempChip", ct.c_float),
        ("tempFlag", ct.c_float),
        ("tempBox", ct.c_float),
    ]

class CameraController:
    def __init__(self, use_webcam=False):
        self.use_webcam = use_webcam
        self.cap = None
        self.libir = None
        self.pathXml = b''
        self.metadata = EvoIRFrameMetadata()

        if self.use_webcam:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                raise RuntimeError("Webcam could not be opened.")
            print("Using webcam for mock input.")
        else:
            try:
                self._init_lib()      # Set XML path and load SDK
                self._init_camera()   # Initialize hardware using XML path
                print("CameraController initialized successfully.")
            except Exception as e:
                raise RuntimeError(f"CameraController init failed: {e}")

    def _init_lib(self):
        # 1) Load the native library
        if os.name == 'nt':
            try:
                self.libir = ct.CDLL(r'.\libirimager.dll')
            except OSError as e:
                raise RuntimeError(f"Failed to load libirimager.dll: {e}")
            # Default XML on Windows (can be overridden by OPTRIS_XML)
            default_xml = r'.\generic.xml'
        else:
            lib_path = find_library("irdirectsdk")
            if not lib_path:
                # Optional fallback path if your system installs it here
                candidate = "/usr/local/lib/libirdirectsdk.so"
                if os.path.isfile(candidate):
                    lib_path = candidate
                else:
                    raise RuntimeError("Could not find irdirectsdk library.")
            try:
                self.libir = ct.cdll.LoadLibrary(lib_path)
            except OSError as e:
                raise RuntimeError(f"Failed to load irdirectsdk: {e}")
            # Default XML on your Pi (can be overridden by OPTRIS_XML)
            default_xml = "/home/yonjanregmi/Thermal_cam/generic.xml"

        # 2) Resolve XML path (env override -> default)
        xml_str = os.environ.get("OPTRIS_XML", default_xml)

        # Validate on filesystem using str
        if not os.path.isfile(xml_str):
            raise RuntimeError(f"Missing XML config: {xml_str}")

        # Store as bytes for the C API
        self.pathXml = xml_str.encode("utf-8")

        # 3) Set ctypes signatures for the functions we call
        #    (prevents implicit conversions that caused earlier errors)
        self.libir.evo_irimager_usb_init.argtypes = [
            ct.c_char_p,  # xml path
            ct.c_char_p,  # format path (may be empty)
            ct.c_char_p,  # log path (may be empty)
        ]
        self.libir.evo_irimager_usb_init.restype = ct.c_int

        self.libir.evo_irimager_get_serial.argtypes = [ct.POINTER(ct.c_ulong)]
        self.libir.evo_irimager_get_serial.restype = ct.c_int

        self.libir.evo_irimager_get_thermal_image_size.argtypes = [
            ct.POINTER(ct.c_int), ct.POINTER(ct.c_int)
        ]
        self.libir.evo_irimager_get_thermal_image_size.restype = ct.c_int

        self.libir.evo_irimager_get_palette_image_size.argtypes = [
            ct.POINTER(ct.c_int), ct.POINTER(ct.c_int)
        ]
        self.libir.evo_irimager_get_palette_image_size.restype = ct.c_int

        self.libir.evo_irimager_get_thermal_palette_image_metadata.argtypes = [
            ct.c_int, ct.c_int, ct.POINTER(ct.c_ushort),
            ct.c_int, ct.c_int, ct.POINTER(ct.c_ubyte),
            ct.POINTER(EvoIRFrameMetadata),
        ]
        self.libir.evo_irimager_get_thermal_palette_image_metadata.restype = ct.c_int

        self.libir.evo_irimager_terminate.argtypes = []
        self.libir.evo_irimager_terminate.restype = None

    def _init_camera(self):
        self.pathFormat = b""
        self.pathLog = b""

        # Holders for sizes/serial
        self.palette_width, self.palette_height = ct.c_int(), ct.c_int()
        self.thermal_width, self.thermal_height = ct.c_int(), ct.c_int()
        self.serial = ct.c_ulong()

        print(f"Using config XML: {self.pathXml!r}")
        ret = self.libir.evo_irimager_usb_init(self.pathXml, self.pathFormat, self.pathLog)
        if ret != 0:
            raise RuntimeError(f"Failed to initialize camera: {ret}")

        # Query device info and image sizes
        self.libir.evo_irimager_get_serial(ct.byref(self.serial))
        print("Camera Serial:", self.serial.value)

        self.libir.evo_irimager_get_thermal_image_size(
            ct.byref(self.thermal_width), ct.byref(self.thermal_height)
        )
        self.libir.evo_irimager_get_palette_image_size(
            ct.byref(self.palette_width), ct.byref(self.palette_height)
        )

        if self.palette_width.value == 0 or self.palette_height.value == 0:
            self.libir.evo_irimager_terminate()
            raise RuntimeError("Palette image size is 0x0.")
        if self.thermal_width.value == 0 or self.thermal_height.value == 0:
            self.libir.evo_irimager_terminate()
            raise RuntimeError("Thermal image size is 0x0.")

        print(f"Palette Image Size: {self.palette_width.value}x{self.palette_height.value}")
        print(f"Thermal Image Size: {self.thermal_width.value}x{self.thermal_height.value}")

        # Allocate numpy buffers and bind to C pointers
        self.np_thermal = np.zeros(
            (self.thermal_height.value, self.thermal_width.value), dtype=np.uint16
        )
        self.npThermalPointer = self.np_thermal.ctypes.data_as(ct.POINTER(ct.c_ushort))

        self.np_img = np.zeros(
            (self.palette_height.value, self.palette_width.value, 3), dtype=np.uint8
        )
        self.npImagePointer = self.np_img.ctypes.data_as(ct.POINTER(ct.c_ubyte))
    def get_frame(self):
        if self.use_webcam:
            ret, frame = self.cap.read()
            if not ret:
                raise RuntimeError("Failed to read from webcam.")
            temp = random.uniform(25.0, 60.0)  # Simulated temperature
            return frame, temp

        # Real thermal camera frame
        ret = self.libir.evo_irimager_get_thermal_palette_image_metadata(
            self.thermal_width, self.thermal_height, self.npThermalPointer,
            self.palette_width, self.palette_height, self.npImagePointer,
            ct.byref(self.metadata)
        )
        if ret != 0:
            raise RuntimeError(f"Camera error: {ret}")

        rgb_img = cv2.cvtColor(self.np_img, cv2.COLOR_BGR2RGB)
        thermal_mean_raw = self.np_thermal.mean()
        mean_temp = thermal_mean_raw / 10.0 - 100.0
        return rgb_img, mean_temp

    def shutdown(self):
        if self.use_webcam and hasattr(self, 'cap'):
            self.cap.release()
        elif self.libir:
            self.libir.evo_irimager_terminate()
