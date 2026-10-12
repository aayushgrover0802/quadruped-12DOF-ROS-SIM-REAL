# ESP32 firmware: `esp32_servo_bridge`

This goes on the Waveshare **Servo Driver with ESP32** board. It is a plain relay:

```
Jetson / laptop  ==USB (921600 baud)==>  ESP32  ==servo bus (1 Mbps, GPIO 18/19)==>  12 x ST3215
```

The ESP32 does **no maths**. It forwards the 12 target positions the Jetson sends,
and sends back each servo's position, speed, load, voltage and temperature 50 times a second.
All kinematics, calibration and safety logic is in the ROS 2 package (`QuadPkg/hardware/`).

| File | What it is |
|---|---|
| `esp32_servo_bridge/esp32_servo_bridge.ino` | Open this in the Arduino IDE (it's only a comment) |
| `esp32_servo_bridge/servo_bridge.cpp` | The actual code. No extra libraries needed |
| `bin/esp32_servo_bridge_merged.bin` | Ready-built firmware, for flashing without the Arduino IDE (option B) |

> Flashing this **replaces Waveshare's demo firmware** (the Wi-Fi web app).
> The middle positions you already set are stored **inside each servo**, so they stay.
> If you ever need the web app again, re-flash Waveshare's demo from their wiki.

---

## Option A: Arduino IDE (easiest)

1. Install the **Arduino IDE 2**.
2. Add ESP32 support: *File → Preferences → Additional boards manager URLs*, paste
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`,
   then *Tools → Board → Boards Manager*, search **esp32** (by Espressif) and install it.
3. *File → Open* → `firmware/esp32_servo_bridge/esp32_servo_bridge.ino`.
   Both files in the folder open as tabs. That's expected.
4. *Tools → Board* → **ESP32 Dev Module**. *Tools → Port* → the board's port.
5. Click **Upload** (→ arrow).

Close the Arduino Serial Monitor afterwards. Only one program can use the port at a time.

## Option B: from the Jetson / laptop terminal, no IDE

```bash
pip3 install esptool
```
> **Breakdown**
> - `pip3`: Python's package installer.
> - `install esptool`: downloads Espressif's official flashing tool.

```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash 0x0 bin/esp32_servo_bridge_merged.bin
```
> **Breakdown**
> - `esptool.py`: the flashing program you just installed.
> - `--chip esp32`: the board has a classic ESP32 chip.
> - `--port /dev/ttyUSB0`: the USB port the board is plugged into. Find it with `ls /dev/ttyUSB* /dev/ttyACM*`.
> - `--baud 460800`: upload speed. If it fails, use `115200` (slower but more reliable).
> - `write_flash 0x0`: write starting at the very beginning of the chip's memory.
> - `bin/esp32_servo_bridge_merged.bin`: the ready-built file (bootloader + partition table + program in one). Run the command from inside the `firmware/` folder (`cd ~/quadruped_ws/src/QuadPkg/firmware`), because the path is relative.
>
> On newer esptool versions the command is `esptool` (no `.py`), and `write_flash` is `write-flash`.

If the upload hangs at `Connecting...`, hold the board's **BOOT** button until it starts.

---

## Check that it works

```bash
ros2 run QuadPkg servo_tool scan
```
> **Breakdown**
> - `ros2 run`: start a program from a ROS 2 package.
> - `QuadPkg`: your package.
> - `servo_tool`: the setup helper (`QuadPkg/hardware/servo_tool.py`).
> - `scan`: ask the ESP32 to ping every possible servo ID (1–253) and list the ones that answer.

You should see `found 12 servo(s): [1, 2, ..., 12]  matches hardware.yaml`.

## Frame format (only if you want to look under the hood)

`0xAA 0x55 | type | len | payload | crc8` (CRC-8, polynomial 0x07, over type+len+payload).
The Python side is `QuadPkg/hardware/protocol.py`. Both files list the same frame types.

| Jetson → ESP32 | | ESP32 → Jetson | |
|---|---|---|---|
| 0x01 HELLO | ping / heartbeat | 0x81 | protocol + firmware version |
| 0x02 CONFIG | servo IDs, feedback rate, watchdog | 0x82 | which servos answered |
| 0x03 TARGETS | 12 × (ticks, speed, acc) | – | (no reply, it's 50 Hz) |
| 0x04 TORQUE | on/off mask | 0x84 | ack |
| 0x05/0x06 REG_WRITE/READ | raw servo registers | 0x85/0x86 | result |
| 0x07 SCAN | ping a range of IDs | 0x87 | list of IDs |
| | | 0x90 STATE | all 12 servos, every 20 ms |
| | | 0x9F LOG | text message |

Watchdog: if the Jetson sends nothing for `watchdog_ms` (500 ms), the ESP32 either keeps
holding the last pose (default) or switches torque off (`watchdog_torque_off: true` in hardware.yaml).
