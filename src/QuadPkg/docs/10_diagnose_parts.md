# 10 · Diagnosing parts — one at a time, from power to legs

Work **top to bottom**: each part depends on the ones above it. Robot on a stand (feet off the ground)
unless a step says otherwise.

```
battery → power wiring → ESP32 board → USB → Jetson/laptop software
                       ↘ servo bus → 12 servos → horns/linkages → legs/feet
```

## 1. Battery and power

```bash
ros2 run QuadPkg servo_tool read
```
> **Breakdown**
> - The `volt` column is each servo's own supply reading. Stop the launch first (port in use).

| Look for | Healthy | Problem → cause |
|---|---|---|
| All 12 volts | within 0.2 V of each other, 11–12.6 V (3S) | One leg lower → bad connector/thin wire on that branch |
| Resting voltage | > 11.1 V | < 10.5 V → charge |
| Voltage while walking (`servo_monitor`, `V` column) | drops < 0.8 V | Big drops / `LOWV` / ESP32 reboots → battery C-rating too low, wire too thin, bad connector |

Brown-out signature: in the launch terminal `[esp32] esp32_servo_bridge ready, waiting for CONFIG` appears **in the middle**
of a run, `servo_monitor` shows `TQ0` on servos and `feedback is N s old`. → power, not software.

## 2. ESP32 board

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```
> **Breakdown**
> - `ls`: list files. Serial devices appear as `/dev/ttyUSB0` (CP210x/CH340 chips) or `/dev/ttyACM0`.
>   Nothing listed → cable is charge-only, board unpowered, or driver missing. Run it with the board unplugged, then plugged, to see which appears.

```bash
sudo dmesg -w
```
> **Breakdown**
> - `dmesg`: the Linux kernel's messages. `-w`: keep watching. Plug the board in: you should see
>   `cp210x converter now attached to ttyUSB0` (or `ch341`). Repeated `disconnect` lines = bad cable or power dips. Ctrl-C to stop.

Then `ros2 run QuadPkg servo_tool scan`: `ESP32 firmware v1` printed = board + firmware + USB OK.

## 3. USB link quality

```bash
ros2 topic hz /joint_states
```
> **Breakdown**
> - With the launch running: should be steady ≈ `feedback_hz` (50). Drops, gaps or "no new messages" = USB/ESP32 hiccups.

Fixes: short, good USB cable; no unpowered hub; on the Jetson avoid the same USB bus as a camera.

## 4. Servo bus and IDs

```bash
ros2 run QuadPkg servo_tool scan
```
> **Breakdown**
> - Lists every ID that answers and compares with `hardware.yaml`.

| Result | Cause | Fix |
|---|---|---|
| `found 0 servo(s)` | servo power off, bus cable not in the board's servo port, wrong bus pins in firmware | power/cable; firmware `BUS_RX_PIN 18 / BUS_TX_PIN 19` match the Waveshare board |
| A whole leg (3 IDs) missing | the daisy-chain cable into that leg | reseat/replace that cable |
| Only the **last** servo(s) of a chain missing | cable between two servos | — |
| One ID missing, an **unexpected** ID present | that servo has the wrong ID | `servo_tool setid WRONG RIGHT` |
| Same ID twice (one "missing", things act weird) | two servos share an ID → they answer together | unplug all but one, `setid`, repeat |
| IDs come and go | loose connector, under-voltage | wiggle-test each connector while `servo_monitor` runs (`OFF` flickers) |

## 5. Each servo

With the launch + `servo_monitor` running and motors **off**, move each joint by hand:
`meas°` should change smoothly and in the **right direction** (same signs as the jog table in [05](05_real_robot_bringup.md#3-check-every-motor-direction-robot-lifted-off-the-ground)).
Jumps or a frozen value = encoder/servo fault.

Then motors **on** (M), robot standing on the stand:

| `servo_monitor` shows | Diagnosis |
|---|---|
| `err°` < 1–2°, `load%` small | healthy |
| `err°` steady 3–10°, `load%` high | servo too weak for the load or the joint is binding — see §6 |
| `err°` grows and the joint runs to its end | **direction is wrong** for that joint → Ctrl-C/K immediately, fix `direction` |
| `ticks>` changes but `<ticks` doesn't, `TQ0` | servo not enabled / reset |
| `ticks>` changes but `<ticks` doesn't, torque on, load 100 % | joint mechanically blocked |
| `°C` one servo much hotter than its twins | that joint fights something (binding, wrong offset pushing the leg into another) |

One-joint test without the controller:

```bash
ros2 run QuadPkg servo_tool jog BR_KneeP 15
```
> **Breakdown**
> - Moves that single joint +15° and back. Watch the leg and compare with the expected motion it prints.

## 6. Horns, linkages, legs

Motors **off** (`servo_tool limp`), move each joint through its range by hand:
* should feel smooth, no tight spots, no click → a tight spot = rubbing print, bent linkage, screw too long;
* no play/backlash at the horn → loose horn screw = the measured angle lies (servo turns, leg doesn't);
* horn not one spline tooth off: `servo_tool calib` would show ~±100 ticks offset for that joint while its twins are ~0.

Compare the twin legs (FL vs FR, BL vs BR) in `servo_tool read` while posed standing: angles should match within ~2°.

## 7. Feet

Worn or hard feet slip → "walks on the spot". Rubber caps, and test on a rubber mat/carpet before smooth floors.

## 8. Software side, last

```bash
ros2 run QuadPkg diagnose
```
> **Breakdown**
> - Exactly 1 `/joint_command` publisher, ~100 messages per 2 s, controller state, worst tracking error per joint.

If everything above is healthy and it still won't walk properly → [11](11_why_not_walking.md).
