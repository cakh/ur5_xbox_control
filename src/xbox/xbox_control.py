"""Xbox 360 / XInput gamepad reader (Windows, zero dependencies, ctypes)."""
import ctypes
import sys
from ctypes import wintypes
from dataclasses import dataclass

BTN_BACK = 0x0020
BTN_START = 0x0010
BTN_LB = 0x0100
BTN_RB = 0x0200
BTN_A = 0x1000
BTN_B = 0x2000
BTN_X = 0x4000
BTN_Y = 0x8000


class _Gamepad(ctypes.Structure):
    _fields_ = [("buttons", wintypes.WORD),
                ("lt", ctypes.c_ubyte), ("rt", ctypes.c_ubyte),
                ("lx", ctypes.c_short), ("ly", ctypes.c_short),
                ("rx", ctypes.c_short), ("ry", ctypes.c_short)]


class _State(ctypes.Structure):
    _fields_ = [("packet", wintypes.DWORD), ("pad", _Gamepad)]


@dataclass
class Pad:
    connected: bool = False
    lx: float = 0.0   # sticks -1..1, up/right positive
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    lt: float = 0.0   # triggers 0..1
    rt: float = 0.0
    buttons: int = 0

    def down(self, mask):
        return bool(self.buttons & mask)


class XboxController:
    def __init__(self, index=0):
        if sys.platform != "win32":
            raise RuntimeError("XInput reader is Windows-only; run this on Windows Python.")
        self.index = index
        for dll in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
            try:
                self._xi = ctypes.WinDLL(dll)
                break
            except OSError:
                continue
        else:
            raise RuntimeError("No XInput DLL found")

    def read(self) -> Pad:
        st = _State()
        if self._xi.XInputGetState(self.index, ctypes.byref(st)) != 0:
            return Pad(connected=False)
        g = st.pad
        return Pad(True, g.lx / 32768, g.ly / 32768, g.rx / 32768, g.ry / 32768,
                   g.lt / 255, g.rt / 255, g.buttons)


if __name__ == "__main__":
    import time
    joy = XboxController()
    while True:
        print(joy.read())
        time.sleep(0.1)
