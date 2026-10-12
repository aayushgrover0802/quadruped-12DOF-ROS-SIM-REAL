"""
protocol.py -- the byte format spoken between the Jetson and the ESP32 firmware
(firmware/esp32_servo_bridge/esp32_servo_bridge.ino). Pure Python, no ROS, no serial.

Every message ("frame") looks like this:

    0xAA 0x55 | type | len | payload (len bytes) | crc8
     header     what   how     the data            checksum over type+len+payload
                 kind  long

All multi-byte numbers are little-endian (low byte first), like the servo itself.
"""
import struct
from dataclasses import dataclass

# ---- frame types: Jetson -> ESP32
HELLO, CONFIG, TARGETS, TORQUE, REG_WRITE, REG_READ, SCAN = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
# ---- frame types: ESP32 -> Jetson
E_HELLO, E_CONFIG_ACK, E_TORQUE_ACK, E_REG_WRITE, E_REG_READ, E_SCAN = 0x81, 0x82, 0x84, 0x85, 0x86, 0x87
E_STATE, E_LOG = 0x90, 0x9F

PROTO_VERSION = 1
MAX_SERVOS = 16

# ---- ST3215 register addresses (same numbers as in the firmware)
REG_ID = 5
REG_MIN_ANGLE = 9          # 2 bytes
REG_MAX_ANGLE = 11         # 2 bytes
REG_OFFSET = 31            # 2 bytes, sign bit 11 (set by "middle" calibration)
REG_TORQUE_ENABLE = 40
REG_LOCK = 55              # 0 = EEPROM writable, 1 = locked
REG_PRESENT_POS = 56

TICKS_PER_REV = 4096       # ST3215: 12-bit encoder, 360 degrees
CENTER_TICK = 2048         # the "middle position" you set in the web app


def crc8(data, crc=0):
    """CRC-8, polynomial 0x07 (same function as in the firmware)."""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode(ftype, payload=b""):
    payload = bytes(payload)
    if len(payload) > 255:
        raise ValueError("payload too long")
    body = bytes([ftype, len(payload)]) + payload
    return b"\xAA\x55" + body + bytes([crc8(body)])


class FrameParser:
    """Feed it raw bytes from the serial port; it returns complete, valid frames."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            start = self.buf.find(b"\xAA\x55")
            if start < 0:
                del self.buf[:-1]          # keep a trailing 0xAA, it may be a header start
                return frames
            del self.buf[:start]
            if len(self.buf) < 4:
                return frames
            n = self.buf[3]
            if len(self.buf) < 5 + n:
                return frames
            body = bytes(self.buf[2:4 + n])
            if crc8(body) == self.buf[4 + n]:
                frames.append((body[0], body[2:]))
                del self.buf[:5 + n]
            else:
                self.bad_crc += 1
                del self.buf[:2]           # skip this header, look for the next one


# ------------------------------------------------------------------ builders
def build_config(ids, feedback_hz=50, watchdog_ms=500, watchdog_action=0):
    if not 0 < len(ids) <= MAX_SERVOS:
        raise ValueError("need 1..16 servo ids")
    return encode(CONFIG, bytes([len(ids)] + [int(i) for i in ids])
                  + struct.pack("<BHB", int(feedback_hz), int(watchdog_ms), int(watchdog_action)))


def build_targets(ticks, speed=0, acc=0):
    """ticks: list of goal positions (raw servo ticks). speed 0 / acc 0 = servo maximum."""
    speeds = speed if isinstance(speed, (list, tuple)) else [speed] * len(ticks)
    accs = acc if isinstance(acc, (list, tuple)) else [acc] * len(ticks)
    out = bytes([len(ticks)])
    for t, s, a in zip(ticks, speeds, accs):
        out += struct.pack("<hHB", int(t), int(s), int(a))
    return encode(TARGETS, out)


def build_torque(mask, on):
    return encode(TORQUE, struct.pack("<HB", int(mask) & 0xFFFF, int(on)))


def build_reg_write(servo_id, addr, data):
    data = bytes(data)
    return encode(REG_WRITE, bytes([servo_id, addr, len(data)]) + data)


def build_reg_read(servo_id, addr, n):
    return encode(REG_READ, bytes([servo_id, addr, n]))


def build_scan(first=1, last=253):
    return encode(SCAN, bytes([first, last]))


# ------------------------------------------------------------------ decoders
@dataclass
class ServoState:
    online: bool        # answered in the last few polls
    fresh: bool         # answered in THIS poll
    torque: bool        # torque on (as requested by us)
    pos: int            # ticks, 0..4095 (2048 = middle)
    speed: int          # ticks/s (signed)
    load: int           # 0.1 % of max torque (signed)
    voltage: float      # volts
    temp: int           # deg C
    current_ma: float   # mA


def decode_state(payload):
    t_ms, n = struct.unpack_from("<IB", payload, 0)
    servos = []
    for i in range(n):
        flags, pos, spd, load, volt, temp, cur = struct.unpack_from("<BhhhBBh", payload, 5 + 11 * i)
        servos.append(ServoState(bool(flags & 1), bool(flags & 2), bool(flags & 4),
                                 pos, spd, load, volt / 10.0, temp, cur * 6.5))
    return t_ms, servos


def decode_config_ack(payload):
    n, lo, hi = payload[0], payload[1], payload[2]
    mask = lo | (hi << 8)
    return [bool(mask & (1 << i)) for i in range(n)]
