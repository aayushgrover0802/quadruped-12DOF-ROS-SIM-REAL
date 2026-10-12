"""
esp32_link.py -- talks to the ESP32 over USB serial. No ROS here, so the same class
is used by the ROS node (hardware/bridge.py) and the setup tool (hardware/servo_tool.py).

A background thread reads the port all the time:
  * STATE frames  -> stored in `self.state` (+ optional callback)
  * LOG frames    -> passed to the `log` function
  * replies       -> put in a queue so request() can wait for them
"""
import queue
import threading
import time

from QuadPkg.hardware import protocol as P

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover
    serial = None


class Esp32Link:
    def __init__(self, port, baud=921600, log=print, on_state=None):
        if serial is None:
            raise RuntimeError("pyserial missing:  sudo apt install python3-serial   (or pip install pyserial)")
        self.log = log
        self.on_state = on_state
        # DTR/RTS are wired to the ESP32's reset/boot pins on most boards.
        # Set them BEFORE opening so the board is not held in reset / bootloader.
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, baud, 0.02
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.parser = P.FrameParser()
        self.replies = queue.Queue()
        self.state = None            # (t_ms, [ServoState]) latest
        self.state_time = 0.0        # time.monotonic() of the latest STATE
        self.n_state = 0
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ low level
    def _reader(self):
        while self._running:
            try:
                data = self.ser.read(512)
            except Exception as e:  # cable pulled etc.
                self.log(f"serial read error: {e}")
                time.sleep(0.2)
                continue
            if not data:
                continue
            for ftype, payload in self.parser.feed(data):
                if ftype == P.E_STATE:
                    self.state = P.decode_state(payload)
                    self.state_time = time.monotonic()
                    self.n_state += 1
                    if self.on_state:
                        self.on_state(*self.state)
                elif ftype == P.E_LOG:
                    self.log("[esp32] " + payload.decode(errors="replace"))
                else:
                    self.replies.put((ftype, payload))

    def send(self, frame):
        with self._lock:
            self.ser.write(frame)

    def request(self, frame, reply_type, timeout=1.0):
        """Send a frame and wait for the reply of `reply_type`. Returns its payload or None."""
        while not self.replies.empty():          # forget stale replies
            self.replies.get_nowait()
        self.send(frame)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                ftype, payload = self.replies.get(timeout=max(0.0, end - time.monotonic()))
            except queue.Empty:
                break
            if ftype == reply_type:
                return payload
        return None

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.ser.close()

    # ------------------------------------------------------------------ high level
    def hello(self, timeout=3.0):
        """Wait until the firmware answers (it may still be booting). Returns (proto, fw) or None."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            r = self.request(P.encode(P.HELLO), P.E_HELLO, timeout=0.3)
            if r is not None:
                if r[0] != P.PROTO_VERSION:
                    raise RuntimeError(f"firmware protocol v{r[0]}, this code speaks v{P.PROTO_VERSION}: "
                                       "re-flash firmware/esp32_servo_bridge")
                return r[0], r[1]
        return None

    def configure(self, ids, feedback_hz=50, watchdog_ms=500, watchdog_action=0):
        """Tell the ESP32 which servos to drive. Returns list[bool]: which ones answered."""
        r = self.request(P.build_config(ids, feedback_hz, watchdog_ms, watchdog_action),
                         P.E_CONFIG_ACK, timeout=2.0)
        return None if r is None else P.decode_config_ack(r)

    def set_targets(self, ticks, speed=0, acc=0):
        self.send(P.build_targets(ticks, speed, acc))

    def torque(self, on, mask=0xFFFF):
        return self.request(P.build_torque(mask, 1 if on else 0), P.E_TORQUE_ACK, timeout=1.0) is not None

    def reg_write(self, servo_id, addr, data):
        r = self.request(P.build_reg_write(servo_id, addr, data), P.E_REG_WRITE, timeout=1.0)
        return r is not None and r[2] == 1

    def reg_read(self, servo_id, addr, n):
        r = self.request(P.build_reg_read(servo_id, addr, n), P.E_REG_READ, timeout=1.0)
        if r is None or r[2] != 1:
            return None
        return bytes(r[4:4 + r[3]])

    def scan(self, first=1, last=253):
        r = self.request(P.build_scan(first, last), P.E_SCAN, timeout=5.0)
        return None if r is None else list(r[1:1 + r[0]])

    def wait_state(self, timeout=1.0, newer_than=None):
        """Block until a STATE frame arrives (newer than `newer_than` count). Returns it or None."""
        start_count = self.n_state if newer_than is None else newer_than
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.n_state > start_count:
                return self.state
            time.sleep(0.005)
        return None
