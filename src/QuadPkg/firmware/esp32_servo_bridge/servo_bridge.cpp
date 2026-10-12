/*
 * servo_bridge.cpp  --  firmware for the Waveshare "Servo Driver with ESP32" board
 * ===================================================================================
 *
 * JOB OF THIS BOARD: be a dumb, fast relay.  NO kinematics, NO calibration, NO gait.
 * All the thinking happens on the Jetson (or your laptop) in the ROS 2 workspace.
 *
 *     Jetson / laptop  <== USB serial (921600 baud) ==>  ESP32  <== servo bus (1 Mbps) ==>  12 x ST3215
 *       hardware_bridge node                            this file                         IDs set by you
 *
 * What it does:
 *   1. Receives frames from the Jetson:
 *        CONFIG   -> which servo IDs to talk to, how often to report, watchdog
 *        TARGETS  -> 12 goal positions (raw servo ticks 0..4095) + speed + acceleration
 *        TORQUE   -> switch servo torque on / off
 *        REG_READ / REG_WRITE / SCAN -> raw access, used by `servo_tool` for setup
 *   2. Sends all 12 TARGETS to the servos in ONE "sync write" packet (all move together).
 *   3. Reads every servo's position / speed / load / voltage / temperature / current
 *      `feedback_hz` times per second and sends one STATE frame back.
 *
 * Libraries needed: NONE (the servo protocol is implemented below, ~100 lines).
 * Board setting in Arduino IDE: "ESP32 Dev Module".  See firmware/README.md.
 *
 * NOTE: flashing this replaces Waveshare's demo firmware (the Wi-Fi web app).
 * The middle positions you set with the web app are stored INSIDE each servo,
 * so they are kept.  You can always flash the Waveshare demo back.
 */

#include <Arduino.h>

// ----------------------------------------------------------------- settings
#define FW_VERSION      1
#define PROTO_VERSION   1

#define HOST_BAUD       921600   // USB serial to the Jetson. Must match hardware.yaml `baud`
#define BUS_BAUD        1000000  // ST3215 default bus speed
#define BUS_RX_PIN      18       // S_RXD on the Waveshare board
#define BUS_TX_PIN      19       // S_TXD on the Waveshare board

#define MAX_SERVOS      16
#define READ_TIMEOUT_US 3000     // how long to wait for one servo to answer
#define SCAN_TIMEOUT_US 1500

// ----------------------------------------------------------------- ST3215 / STS register map
#define REG_TORQUE_ENABLE  40    // 0 = off, 1 = on, 128 = "current position becomes 2048"
#define REG_ACC            41    // 1 byte, then GOAL_POS(2) GOAL_TIME(2) GOAL_SPEED(2)
#define REG_PRESENT_POS    56    // 15 bytes from here: pos(2) speed(2) load(2) volt(1) temp(1) ... current@69(2)
#define FEEDBACK_LEN       15

#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_SYNC_WRITE 0x83
#define BROADCAST_ID    0xFE

// ----------------------------------------------------------------- host protocol
// Frame: 0xAA 0x55 | type | len | payload[len] | crc8(type, len, payload)
#define H_HELLO      0x01
#define H_CONFIG     0x02
#define H_TARGETS    0x03
#define H_TORQUE     0x04
#define H_REG_WRITE  0x05
#define H_REG_READ   0x06
#define H_SCAN       0x07

#define E_HELLO      0x81
#define E_CONFIG_ACK 0x82
#define E_TORQUE_ACK 0x84
#define E_REG_WRITE  0x85
#define E_REG_READ   0x86
#define E_SCAN       0x87
#define E_STATE      0x90
#define E_LOG        0x9F

HardwareSerial &Bus = Serial1;

// ----------------------------------------------------------------- state
struct Servo {
  uint8_t  id;
  bool     online;        // answered recently
  bool     fresh;         // answered in the latest poll cycle
  uint8_t  misses;        // consecutive missed reads
  bool     torque;        // what the HOST asked for
  int16_t  pos, speed, load, current;
  uint8_t  volt, temp;
};

Servo    servos[MAX_SERVOS];
uint8_t  nServos       = 0;          // 0 until the Jetson sends CONFIG
uint8_t  feedbackHz    = 50;
uint16_t watchdogMs    = 500;
uint8_t  watchdogAct   = 0;          // 0 = hold last position, 1 = torque off (robot goes limp)
uint32_t lastHostMs    = 0;
bool     watchdogFired = false;
uint32_t lastPollUs    = 0;
uint32_t pollCycle     = 0;

bool     polling       = false;      // true while pollFeedback() is running
uint8_t  pendingFrame[260];          // a slow frame that arrived mid-poll (handled right after)
int      pendingLen    = 0;

void serviceHost();                  // defined further down

uint8_t  lastSent[256];              // copy of the last bus packet (to ignore an echo)
int      lastSentLen = 0;

// ================================================================= small helpers
static uint8_t crc8(const uint8_t *d, int n, uint8_t crc = 0) {
  for (int i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static void put16(uint8_t *p, int16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (v >> (8 * i)) & 0xFF; }

// ST3215 stores negative numbers as "magnitude + sign bit" (not two's complement)
static int16_t fromSignMag(uint16_t raw, uint8_t signBit) {
  uint16_t mask = (uint16_t)1 << signBit;
  return (raw & mask) ? -(int16_t)(raw & (mask - 1)) : (int16_t)(raw & (mask - 1));
}
static uint16_t toSignMag(int16_t v) {
  return v < 0 ? (uint16_t)((-v) | 0x8000) : (uint16_t)v;
}

static void sendHost(uint8_t type, const uint8_t *payload, uint8_t len) {
  uint8_t head[4] = {0xAA, 0x55, type, len};
  uint8_t crc = crc8(&head[2], 2);
  crc = crc8(payload, len, crc);
  Serial.write(head, 4);
  if (len) Serial.write(payload, len);
  Serial.write(crc);
}

static void logHost(const char *msg) {
  sendHost(E_LOG, (const uint8_t *)msg, (uint8_t)min((int)strlen(msg), 200));
}

// ================================================================= servo bus (half duplex)
static void busWrite(uint8_t id, uint8_t inst, const uint8_t *params, int n) {
  uint8_t *p = lastSent;
  p[0] = 0xFF; p[1] = 0xFF; p[2] = id; p[3] = (uint8_t)(n + 2); p[4] = inst;
  uint8_t sum = id + p[3] + inst;
  for (int i = 0; i < n; i++) { p[5 + i] = params[i]; sum += params[i]; }
  p[5 + n] = ~sum;
  lastSentLen = n + 6;
  while (Bus.available()) Bus.read();          // drop any old bytes
  Bus.write(p, lastSentLen);
  Bus.flush();                                  // wait until really sent
}

// Read ONE valid packet (FF FF ID LEN ... CHK). Returns its length, or -1 on timeout.
static int busReadPacket(uint8_t *buf, int maxLen, uint32_t timeoutUs) {
  uint32_t t0 = micros();
  int idx = 0;
  uint8_t len = 0;
  while ((uint32_t)(micros() - t0) < timeoutUs) {
    if (!Bus.available()) continue;
    uint8_t b = Bus.read();
    if (idx == 0)      { if (b == 0xFF) buf[idx++] = b; }
    else if (idx == 1) { if (b == 0xFF) buf[idx++] = b; else idx = 0; }
    else if (idx == 2) { if (b != 0xFF) buf[idx++] = b; }      // extra 0xFF = still header
    else if (idx == 3) { len = b; if (len < 2 || len + 4 > maxLen) idx = 0; else buf[idx++] = b; }
    else {
      buf[idx++] = b;
      if (idx == 4 + len) {
        uint8_t sum = 0;
        for (int i = 2; i < idx - 1; i++) sum += buf[i];
        if ((uint8_t)~sum == buf[idx - 1]) return idx;
        idx = 0;                                                // bad checksum -> resync
      }
    }
  }
  return -1;
}

// Wait for the status packet of `id`. Skips our own echo if the board echoes TX.
// Returns number of parameter bytes copied to `out`, or -1.
static int busReadStatus(uint8_t id, uint8_t *out, int outMax, uint32_t timeoutUs) {
  uint8_t buf[80];
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < timeoutUs) {
    uint32_t used = micros() - t0;
    if (used >= timeoutUs) return -1;
    int n = busReadPacket(buf, sizeof(buf), timeoutUs - used);
    if (n < 0) return -1;
    if (n == lastSentLen && memcmp(buf, lastSent, n) == 0) continue;   // our own echo
    if (buf[2] != id) continue;
    int nParams = buf[3] - 2;                                           // minus ERR and CHK
    if (nParams > outMax) nParams = outMax;
    memcpy(out, &buf[5], nParams);
    return nParams;
  }
  return -1;
}

static bool busReadRegs(uint8_t id, uint8_t addr, uint8_t n, uint8_t *out, uint32_t timeoutUs = READ_TIMEOUT_US) {
  uint8_t prm[2] = {addr, n};
  busWrite(id, INST_READ, prm, 2);
  return busReadStatus(id, out, n, timeoutUs) == n;
}

static bool busWriteRegs(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t n) {
  uint8_t prm[64];
  if (n > 62) return false;
  prm[0] = addr;
  memcpy(&prm[1], data, n);
  busWrite(id, INST_WRITE, prm, n + 1);
  if (id == BROADCAST_ID) return true;                  // broadcast never answers
  uint8_t dummy[4];
  return busReadStatus(id, dummy, sizeof(dummy), READ_TIMEOUT_US) >= 0;
}

static bool busPing(uint8_t id, uint32_t timeoutUs) {
  busWrite(id, INST_PING, nullptr, 0);
  uint8_t dummy[4];
  return busReadStatus(id, dummy, sizeof(dummy), timeoutUs) >= 0;
}

// One packet that writes `dataLen` bytes at `addr` to many servos at once.
static void busSyncWrite(uint8_t addr, uint8_t dataLen, const uint8_t *ids, const uint8_t *data, int count) {
  static uint8_t prm[250];
  int k = 0;
  prm[k++] = addr;
  prm[k++] = dataLen;
  for (int i = 0; i < count; i++) {
    if (k + 1 + dataLen > (int)sizeof(prm)) break;
    prm[k++] = ids[i];
    memcpy(&prm[k], &data[i * dataLen], dataLen);
    k += dataLen;
  }
  busWrite(BROADCAST_ID, INST_SYNC_WRITE, prm, k);
}

// ================================================================= actions
static void setTorque(uint16_t mask, uint8_t value) {
  uint8_t ids[MAX_SERVOS], data[MAX_SERVOS * 7];
  int count = 0;
  if (value) {
    // Before switching ON: make the goal = where the servo IS, so nothing jumps.
    for (int i = 0; i < nServos; i++) {
      if (!(mask & (1u << i)) || !servos[i].online) continue;
      uint8_t *d = &data[count * 7];
      uint16_t p = toSignMag(servos[i].pos);
      d[0] = 0; d[1] = p & 0xFF; d[2] = p >> 8; d[3] = 0; d[4] = 0; d[5] = 0; d[6] = 0;
      ids[count++] = servos[i].id;
    }
    if (count) busSyncWrite(REG_ACC, 7, ids, data, count);
  }
  count = 0;
  for (int i = 0; i < nServos; i++) {
    if (!(mask & (1u << i))) continue;
    ids[count] = servos[i].id;
    data[count] = value;
    servos[i].torque = value != 0;
    count++;
  }
  if (count) busSyncWrite(REG_TORQUE_ENABLE, 1, ids, data, count);
}

static void handleFrame(uint8_t type, const uint8_t *p, uint8_t len);

static void pollFeedback() {
  polling = true;
  for (int i = 0; i < nServos; i++) {
    Servo &s = servos[i];
    s.fresh = false;
    // an offline servo is only retried every 10th cycle, so it can't slow the others down
    if (!s.online && s.misses >= 3 && ((pollCycle + i) % 10) != 0) continue;
    uint8_t r[FEEDBACK_LEN];
    if (busReadRegs(s.id, REG_PRESENT_POS, FEEDBACK_LEN, r)) {
      s.pos     = fromSignMag(r[0] | (r[1] << 8), 15);
      s.speed   = fromSignMag(r[2] | (r[3] << 8), 15);
      s.load    = fromSignMag(r[4] | (r[5] << 8), 10);
      s.volt    = r[6];
      s.temp    = r[7];
      s.current = fromSignMag(r[13] | (r[14] << 8), 15);
      s.online = s.fresh = true;
      s.misses = 0;
    } else if (s.misses < 255) {
      s.misses++;
      if (s.misses >= 3) s.online = false;
    }
    serviceHost();               // never let a TARGETS frame wait for a whole poll cycle
  }
  pollCycle++;
  polling = false;

  // STATE frame: t_ms(4) n(1) then per servo: flags pos speed load volt temp current (11 bytes)
  uint8_t out[5 + MAX_SERVOS * 11];
  put32(out, millis());
  out[4] = nServos;
  for (int i = 0; i < nServos; i++) {
    uint8_t *p = &out[5 + i * 11];
    Servo &s = servos[i];
    p[0] = (s.online ? 1 : 0) | (s.fresh ? 2 : 0) | (s.torque ? 4 : 0);
    put16(&p[1], s.pos);
    put16(&p[3], s.speed);
    put16(&p[5], s.load);
    p[7] = s.volt;
    p[8] = s.temp;
    put16(&p[9], s.current);
  }
  sendHost(E_STATE, out, 5 + nServos * 11);
}

// ================================================================= host frame handling
static void handleFrame(uint8_t type, const uint8_t *p, uint8_t len) {
  lastHostMs = millis();
  watchdogFired = false;

  switch (type) {
    case H_HELLO: {
      uint8_t out[3] = {PROTO_VERSION, FW_VERSION, nServos};
      sendHost(E_HELLO, out, 3);
      break;
    }
    case H_CONFIG: {   // n, ids[n], feedback_hz, watchdog_ms(2), watchdog_action
      if (len < 1 || p[0] > MAX_SERVOS || len < 1 + p[0] + 4) { logHost("bad CONFIG"); break; }
      nServos = p[0];
      for (int i = 0; i < nServos; i++) {
        servos[i] = Servo();
        servos[i].id = p[1 + i];
      }
      const uint8_t *q = &p[1 + nServos];
      feedbackHz  = q[0] ? q[0] : 50;
      watchdogMs  = q[1] | (q[2] << 8);
      watchdogAct = q[3];
      // ping everyone once and report who answered (bit i = servo i)
      uint16_t online = 0;
      for (int i = 0; i < nServos; i++) {
        if (busPing(servos[i].id, READ_TIMEOUT_US)) { online |= 1u << i; servos[i].online = true; }
        else servos[i].misses = 3;
      }
      uint8_t out[3] = {nServos, (uint8_t)(online & 0xFF), (uint8_t)(online >> 8)};
      sendHost(E_CONFIG_ACK, out, 3);
      break;
    }
    case H_TARGETS: {  // n, then per servo: pos(2, signed ticks) speed(2) acc(1)
      if (len < 1 || p[0] != nServos || len < 1 + 5 * nServos) break;
      uint8_t ids[MAX_SERVOS], data[MAX_SERVOS * 7];
      int count = 0;
      for (int i = 0; i < nServos; i++) {
        if (!servos[i].torque) continue;   // writing a goal would switch torque on -> only if asked
        const uint8_t *s = &p[1 + 5 * i];
        int16_t pos = (int16_t)(s[0] | (s[1] << 8));
        uint16_t pm = toSignMag(pos);
        uint8_t *d = &data[count * 7];
        d[0] = s[4];                        // acceleration
        d[1] = pm & 0xFF; d[2] = pm >> 8;   // goal position
        d[3] = 0; d[4] = 0;                 // goal time (unused)
        d[5] = s[2]; d[6] = s[3];           // goal speed
        ids[count++] = servos[i].id;
      }
      if (count) busSyncWrite(REG_ACC, 7, ids, data, count);
      break;
    }
    case H_TORQUE: {   // mask(2) value(1)
      if (len < 3) break;
      uint16_t mask = p[0] | (p[1] << 8);
      setTorque(mask, p[2]);
      uint8_t out[3] = {p[0], p[1], p[2]};
      sendHost(E_TORQUE_ACK, out, 3);
      break;
    }
    case H_REG_WRITE: { // id addr n data[n]
      if (len < 3 || len < 3 + p[2]) break;
      bool ok = busWriteRegs(p[0], p[1], &p[3], p[2]);
      uint8_t out[3] = {p[0], p[1], (uint8_t)ok};
      sendHost(E_REG_WRITE, out, 3);
      break;
    }
    case H_REG_READ: {  // id addr n
      if (len < 3 || p[2] > 64) break;
      uint8_t out[4 + 64];
      out[0] = p[0]; out[1] = p[1];
      bool ok = busReadRegs(p[0], p[1], p[2], &out[4]);
      out[2] = ok; out[3] = ok ? p[2] : 0;
      sendHost(E_REG_READ, out, 4 + out[3]);
      break;
    }
    case H_SCAN: {      // first(1) last(1) -> list of IDs that answer a ping
      uint8_t first = len >= 2 ? p[0] : 1, last = len >= 2 ? p[1] : 253;
      uint8_t out[255];
      int n = 0;
      for (int id = first; id <= last && id < 254 && n < 254; id++) {
        if (busPing((uint8_t)id, SCAN_TIMEOUT_US)) out[1 + n++] = (uint8_t)id;
      }
      out[0] = (uint8_t)n;
      sendHost(E_SCAN, out, 1 + n);
      break;
    }
    default:
      logHost("unknown frame type");
  }
}

// Byte-by-byte frame parser for the USB side
static uint8_t  rxBuf[260];
static int      rxIdx = 0;

void serviceHost() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (rxIdx == 0)      { if (b == 0xAA) rxBuf[rxIdx++] = b; }
    else if (rxIdx == 1) { if (b == 0x55) rxBuf[rxIdx++] = b; else rxIdx = (b == 0xAA) ? 1 : 0; }
    else {
      rxBuf[rxIdx++] = b;
      if (rxIdx >= 4) {
        uint8_t len = rxBuf[3];
        if (rxIdx == 5 + len) {
          uint8_t crc = crc8(&rxBuf[2], 2 + len);
          if (crc == rxBuf[4 + len]) {
            uint8_t type = rxBuf[2];
            if (polling && type != H_TARGETS && type != H_TORQUE && type != H_HELLO) {
              // CONFIG / SCAN / REG_* would disturb the poll in progress -> do it right after
              memcpy(pendingFrame, rxBuf, 5 + len);
              pendingLen = 5 + len;
            } else {
              handleFrame(type, &rxBuf[4], len);
            }
          }
          rxIdx = 0;
        }
      }
    }
  }
}

// ================================================================= Arduino entry points
void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(HOST_BAUD);
  Bus.setRxBufferSize(512);
  Bus.begin(BUS_BAUD, SERIAL_8N1, BUS_RX_PIN, BUS_TX_PIN);
  delay(200);
  lastHostMs = millis();
  logHost("esp32_servo_bridge ready, waiting for CONFIG");
}

void loop() {
  serviceHost();

  // Watchdog: the Jetson went quiet (crash, cable pulled...)
  if (nServos && !watchdogFired && watchdogMs && (millis() - lastHostMs) > watchdogMs) {
    watchdogFired = true;
    if (watchdogAct == 1) {
      setTorque(0xFFFF, 0);
      logHost("watchdog: no data from host -> torque OFF");
    } else {
      logHost("watchdog: no data from host -> holding last position");
    }
  }

  uint32_t period = 1000000UL / (feedbackHz ? feedbackHz : 50);
  if (nServos && (uint32_t)(micros() - lastPollUs) >= period) {
    lastPollUs = micros();
    pollFeedback();
  }
  if (pendingLen) {
    pendingLen = 0;
    handleFrame(pendingFrame[2], &pendingFrame[4], pendingFrame[3]);
  }
}
