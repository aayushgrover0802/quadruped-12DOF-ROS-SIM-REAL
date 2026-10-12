# 09 · Warnings and errors reference

Every message the code prints, grouped by who prints it. Search this page for a few words of the message.
`[WARN]` = yellow, keeps running. `[ERROR]` = red. "crash" = the node stops.

## hardware_bridge (real robot)

| Message | What it means | What to do |
|---|---|---|
| `No answer from the ESP32 on /dev/ttyUSB0 @ 921600 baud` (crash) | Nothing on that port speaks our protocol | Wrong port (`ls /dev/ttyUSB* /dev/ttyACM*`, then `port:=…`); firmware not flashed; Arduino Serial Monitor or `servo_tool` still has the port open; board in bootloader (unplug/replug) |
| `firmware protocol vX, this code speaks v1` (crash) | ESP32 runs a different firmware version | Re-flash `firmware/` |
| `ESP32 did not acknowledge CONFIG` (crash) | ESP32 answered HELLO but not CONFIG | USB cable/hub flaky; re-plug; try a short cable straight into the Jetson |
| `pyserial missing` (crash) | Python serial library not installed | `sudo apt install python3-serial` |
| `Permission denied: '/dev/ttyUSB0'` (crash) | You're not in the `dialout` group | `sudo usermod -aG dialout $USER`, log out/in |
| `servos NOT answering: FL_KneeP(id 1), …` | Those IDs didn't reply at start. **Motors will refuse to enable** | Servo power on? Bus cable in? ID right? → [10 §4](10_diagnose_parts.md#4-servo-bus-and-ids) |
| `all 12 servos answered` | Good | — |
| `refusing to enable: not all servos are answering` | You pressed M, but a servo is missing | Same as above. `servo_monitor` shows which (`OFF`) |
| `refusing to enable: no feedback from the ESP32 yet` | No STATE frame received yet | Wait 1 s after start; if it persists the ESP32 isn't reading the servos (servo power off?) |
| `ESP32 did not confirm torque ON` | Torque command got no reply | USB problem, or ESP32 rebooting (brown-out). Check battery |
| `motors ENABLED -- gliding to the commanded pose` | Torque on, slow ramp for `enable_ramp_time` s | — |
| `motors DISABLED (limp)` | K pressed / `/quad/enable false` | Hold the robot |
| `no feedback from the ESP32 for N s` | STATE frames stopped | ESP32 reset (brown-out when motors pull current), USB unplugged, ESP32 crashed. Look for `[esp32] esp32_servo_bridge ready` reappearing = it rebooted → power problem |
| `servos not answering: [...]` (every 5 s) | A servo dropped out while running | Loose connector on that leg, overheated servo protection, voltage dip |
| `LOW BATTERY: N V` | Average servo supply below `min_voltage` | Charge. Under load the voltage dips — if it only appears while walking, the battery/wiring can't deliver the current |
| `HOT servos: FR_KneeP 68C …` | Above `max_temp_c` | Stop and let it cool. Repeated → that joint is overloaded ([08 §7](08_urdf_and_weight.md#7-weight-budget-how-much-can-it-carry)) or binding mechanically |
| `[esp32] watchdog: no data from host -> holding last position` | The Jetson went silent > `watchdog_ms` | Bridge crashed/frozen or USB dropped. The robot holds its pose |
| `[esp32] watchdog: no data from host -> torque OFF` | Same, with `watchdog_torque_off: true` | The robot went limp |
| `[esp32] bad CONFIG` / `unknown frame type` | Corrupted or mismatched frames | Firmware and Python out of sync → re-flash; noisy USB |
| `[esp32] esp32_servo_bridge ready, waiting for CONFIG` | ESP32 just booted | Normal once at start. **In the middle of a run = it reset** (power) |

## quad_controller

| Message | What it means | What to do |
|---|---|---|
| `Loaded …urdf + payload …: total mass X kg, COM at zero pose = (…)` | Info: the model it's using | Check the mass matches reality |
| `stance shifted ±N mm to centre feet under COM` | Info: auto foot centering | Large values → weight far forward/back |
| `Startup: blending from measured /joint_states to STAND` | Good: starts from where the legs really are | — |
| `Startup: blending from zero pose (no /joint_states)` | No feedback within `startup_wait` | Real robot: start the bridge **before** / with the controller (the launch file does). Sim: Isaac not playing or no joint-state publisher |
| `state -> STAND / WALK / SIT` | State changes | Never reaches WALK → [11](11_why_not_walking.md) |
| `N foot targets were out of reach (clamped)` | IK couldn't reach where the gait wanted the foot | `step_height`/`max_step_length` too big, `stand_height` too tall/short, or `stance_x_extra` too big. Shows as `LIM` in `servo_monitor` |
| `Trying to set parameter … of type INTEGER, expecting type DOUBLE` (crash) | YAML has `1` where `1.0` is needed | Add `.0` ([04](04_configuration.md#yaml-rules-that-bite)) |
| `<LEG>: HipRoll axis … is not along Y` (crash) | URDF changed shape | [08 §1](08_urdf_and_weight.md#1-what-the-urdf-must-contain) |
| `payload item 'X': need mass > 0, xyz[3], size[3]` (crash) | Typo in `payload.yaml` | Fix that block |

## servo_tool

| Message | What to do |
|---|---|
| `ESP32 does not answer. Is the firmware flashed? Right port? Robot launch stopped?` | Stop `robot.launch.py` (it holds the port), check port |
| `WARNING, not answering: …` | Those servos missing → [10 §4](10_diagnose_parts.md#4-servo-bus-and-ids) |
| `scan failed (no reply)` | ESP32 busy/rebooting; retry |
| `missing: [...] unexpected: [...]` (scan) | IDs on the bus differ from `hardware.yaml`. Unexpected ID = a servo with the wrong ID → `setid` |
| `offset … <-- large! wrong ID or pose?` (calib) | > 200 ticks (17°) off: robot not in the stand pose, IDs swapped, or a horn one spline tooth off |
| `ID N is already used by another servo` (setid) | Pick a free ID, or connect one servo at a time |

## diagnose

| Message | Meaning |
|---|---|
| `/joint_command publishers : 2 <-- PROBLEM` | Two controllers running (e.g. old launch still open, or `gait` + `controller`) — kill one |
| `/clock : NOT received` | Sim only: controller with `use_sim_time` waits forever. Real robot: ignore |
| `/joint_states : NOT received` | Bridge not running / Isaac joint-state publisher missing |
| tracking error `<-- LARGE` (> 0.15 rad ≈ 8.6°) | Joint can't follow: too weak, blocked, wrong direction |

## servo_monitor flags and feedback messages
See [07 · flags table](07_debug_topics.md#1-the-easy-way-servo_monitor). Feedback `message` endings:

| `<-- …` text | Meaning |
|---|---|
| `NOT ANSWERING` | servo silent; its numbers are stale |
| `torque OFF while motors enabled` | servo reset itself (brown-out / overload protection) or a torque command was lost |
| `HOT` / `low voltage` | over `max_temp_c` / under `min_voltage` |
| `not following (err ±N deg)` | \|sent − measured\| > `debug_error_warn_deg` |
| `load near 100% (stalling?)` | servo at its torque limit |
| `feedback is N s old` | ESP32 stopped sending STATE |
