# 07 · Servo debug topics: what you SEND vs. what comes BACK

`hardware_bridge` publishes three topics that exist only so **you** can look at them. Nothing subscribes to
them and they never move the robot.

| Topic | Type | Shows |
|---|---|---|
| `/debug/servo_command` | `diagnostic_msgs/DiagnosticArray` | **SENDING side**, per servo: what the controller asked, what survived the URDF limits and the slew limiter, and the exact ticks put on the wire. Or *why nothing is sent*. |
| `/debug/servo_feedback` | `diagnostic_msgs/DiagnosticArray` | **RETURN side**, per servo: ticks it reports, angle, error vs. what was sent, speed, load, voltage, temperature, current, online/torque flags, with a WARN/ERROR level |
| `/debug/joint_sent` | `sensor_msgs/JointState` | the sent angles (rad), every 50 Hz tick, same format as `/joint_states` → plotting, `lag_meter` |

### The path of one command (where each value is taken)

```
quad_controller ──/joint_command──► hardware_bridge
   "ctrl°"  controller_target_deg      │ 1. clamp to URDF limits      → after_limits_deg   (flag LIM)
                                       │ 2. slew limit max_velocity    → sent_deg           (flag SLEW)
                                       │ 3. rad → ticks (direction, offset)  → sent_ticks   "ticks>"
                                       ▼
                                    ESP32 ──► servo
                                    ESP32 ◄── servo  present position etc.
                                       │ ticks                          → ticks            "<ticks"
                                       │ ticks → rad                    → angle_deg        "meas°"
                                       │ sent − measured                → error_deg        "err°"
                                       ▼
                         /debug/servo_feedback   (+ /joint_states for the controller)
```

---

## 1. The easy way: `servo_monitor`

```bash
ros2 run QuadPkg servo_monitor
```
> **Breakdown**
> - Subscribes to the topics above plus `/joint_command`, `/joint_states`, `/quad/state`, `/quad/motors_enabled`,
>   `/quad/battery_voltage`, and redraws a table 4×/s. Read-only. Ctrl-C to quit.
> - Run it in its own terminal (or on your laptop while the bridge runs on the Jetson — same `ROS_DOMAIN_ID`).

Example (motors on, standing, one knee lagging, one servo dead):

```
servo_monitor  [REAL ROBOT (hardware_bridge debug topics)]   21:42:07
controller state: STAND   motors enabled: True   battery: 11.6 V
last msg  /joint_command  0.0s ago | /debug/servo_command  0.0s ago | /debug/servo_feedback  0.1s ago | /joint_states  0.0s ago

joint        id   ctrl°   sent° ticks> <ticks   meas°   err°  load%  °C     V  flags
------------------------------------------------------------------------------------
FL_HipRoll    3    +0.0    +0.0   2121   2121    +0.0   +0.0    +12  38  11.6
FR_HipRoll    6   +68.8   +17.2   2229   2229   +17.2   -0.0    +12  38  11.6  LIM SLEW
FR_KneeP      4    +0.0   +11.5   1872   2003    +0.0  +11.5    +12  38  11.6  ERR
BL_ThighP     8    +0.0    +0.0   2131      -       -      -      -   -     -  OFF
BR_HipRoll   12    +0.0    +0.0   2022   2022    +0.0   +0.0    +12  70  11.6  HOT
...
```

| Column | Meaning |
|---|---|
| `id` | servo bus ID (from `hardware.yaml`) |
| `ctrl°` | what `quad_controller` wants (`/joint_command`) |
| `sent°` | what the bridge actually sent after limits + slew. ≠ `ctrl°` → see flags |
| `ticks>` | raw position sent to the servo (0–4095, 2048 = middle) |
| `<ticks` | raw position the servo reports |
| `meas°` | `<ticks` converted back to a joint angle (0 = standing) |
| `err°` | `sent° − meas°`. Small (< 2°) = follows. Big and steady = too weak/blocked. Big and growing = wrong direction |
| `load%` | servo's own load estimate, ±100 % of max torque |
| `°C`, `V` | servo temperature and its supply voltage |

| Flag | Meaning | First thing to check |
|---|---|---|
| `OFF` | servo didn't answer the last polls; its numbers are hidden (stale) | cable, ID, power — [10](10_diagnose_parts.md#4-servo-bus-and-ids) |
| `TQ0` | servo reports torque OFF | motors not enabled (press M), or the servo reset (brown-out) |
| `LIM` | controller asked beyond the URDF joint limit, command was clamped | `stand_height`/`step_height` too big, or a wrong `direction` making IK ask for silly angles |
| `SLEW` | slew limiter is still catching up | normal for a few seconds after M (`enable_velocity`); constant during walking → `max_velocity` too low |
| `HOT` | above `max_temp_c` | stop, let it cool; leg overloaded |
| `LOWV` | below `min_voltage` | charge the battery; voltage sag under load |
| `ERR` | \|err°\| > `debug_error_warn_deg` (5°) | servo can't keep up: weight, friction, binding, low voltage, wrong direction |

Options:

```bash
ros2 run QuadPkg servo_monitor --view cmd
```
> **Breakdown**
> - Only the **sending** columns (`ctrl°`, `sent°`, `ticks>`). Use `--view fb` for only the **feedback** columns.

```bash
ros2 run QuadPkg servo_monitor --no-clear --rate 1 | tee servo_log.txt
```
> **Breakdown**
> - `--no-clear`: don't wipe the screen, just keep printing tables. `--rate 1`: one table per second.
> - `| tee servo_log.txt`: show it on screen **and** save it into `servo_log.txt`.

```bash
ros2 run QuadPkg servo_monitor --once
```
> **Breakdown**
> - Listen for 2 s, print one table, exit. Handy to paste into a message when asking for help.

---

## 1b. How late is each servo? `lag_meter`

`servo_monitor` shows single moments. To see the **timing**, record a few seconds of walking:

```bash
ros2 run QuadPkg lag_meter --seconds 10
```
> **Breakdown**
> - Start it, then press **W** in teleop and let the robot walk for the whole 10 s.
> - It compares `/debug/joint_sent` (sent, every 50 Hz tick) with `/joint_states` (measured) and finds, per joint,
>   the time shift where they match best.

| Column | Meaning | Good |
|---|---|---|
| `lag ms` | how late the servo follows | similar for left/right twins |
| `err now` | RMS error as `servo_monitor` sees it | small |
| `err @lag` | error left once the delay is removed (sag, friction, weakness) | < 2° |
| `reach %` | measured movement size / sent movement size | close to 100 % |

An **equal** delay on all joints does no harm: the controller doesn't use feedback while walking,
so the whole trot just happens a bit later. What hurts:
* **knees later than thighs**: the foot swings forward before it is lifted and is still up when it should land.
  Fix: `knee_lead` in `quad_params.yaml` (lag_meter prints the value). It sends only the knee angles that much earlier.
* **reach % well below 100 on thighs/knees**: the swing is too short for the servos and steps shrink.
  Fix: longer `gait_period`, lower `step_height`.

To check one servo on its own, with no ROS in between (robot lifted, launch stopped):

```bash
ros2 run QuadPkg servo_tool step FR_KneeP 10
```
> **Breakdown**
> - `step`: jumps that one joint 10° at once and times the response: when it starts moving, when it's half way, 90 %.
> - Under ~120 ms to 90 % = the servo is fast; extra lag while walking then comes from weight/voltage.
> - `--speed N --acc N`: try other values of the servo's speed/acceleration registers (0 = maximum).
* **`ODD LEG:` lines**: one joint is much later or shorter than its twin, so that leg has its own problem
  (power, connector, horn, binding). See [10](10_diagnose_parts.md).

---

## 2. The raw way: `ros2 topic echo`

```bash
ros2 topic echo /debug/servo_command
```
> **Breakdown**
> - Prints every command message: 12 entries, each with `level` (0 OK, 1 WARN, 2 ERROR), `name` (joint),
>   `message` (one-line summary) and `values` (key/value list). Ctrl-C to stop.

One entry looks like this:

```yaml
- level: "\x01"                   # WARN (exact look depends on your ROS version)
  name: FR_HipRoll
  message: 'sent +17.19 deg = 2229 ticks  [CLAMPED by URDF limit]  [slew-limited]'
  hardware_id: servo id 6
  values:
  - key: controller_target_deg     # what the controller asked
    value: '+68.75'
  - key: after_limits_deg          # after clamping to the URDF <limit>
    value: '+45.84'
  - key: sent_deg                  # after the slew limiter = what really went out
    value: '+17.19'
  - key: sent_ticks
    value: '2229'
  - key: clamped_by_limit
    value: 'True'
  - key: slew_limited
    value: 'True'
  - key: slew_limit_deg_s
    value: '458'
  - key: limit_low_deg
    value: '-45.8'
  - key: limit_high_deg
    value: '+45.8'
```
> **Breakdown**
> - `level` is a byte field, so ROS prints it oddly (e.g. `"\x01"` or `b'\x01'`): 0 = OK, 1 = WARN, 2 = ERROR.
> - When motors are off, `message` reads `NOT SENDING: motors OFF (press M in teleop)` or
>   `NOT SENDING: no /joint_command received (is quad_controller running?)`.

```bash
ros2 topic echo /debug/servo_feedback
```
> **Breakdown**
> - Per servo: `online`, `fresh_this_poll`, `torque_on`, `ticks`, `angle_deg`, `error_deg`, `speed_deg_s`,
>   `load_pct`, `voltage_v`, `temp_c`, `current_ma`, `feedback_age_s`. The `message` ends with `<-- …` listing any problem.

```bash
ros2 topic echo --once /debug/servo_feedback > feedback.yaml
```
> **Breakdown**
> - `--once`: one message only. `> feedback.yaml`: save it into a file.

```bash
ros2 topic echo /joint_states --field position
```
> **Breakdown**
> - `--field position`: print only that part of the message (12 measured angles in radians, `joint_order`).

---

## 3. Plotting sent vs. measured (PlotJuggler)

```bash
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
```
> **Breakdown**
> - Installs PlotJuggler with its ROS 2 plugin (a live graph tool).

```bash
ros2 run plotjuggler plotjuggler
```
> **Breakdown**
> - Opens the window. *Streaming → ROS2 Topic Subscriber → Start*, tick `/joint_command`, `/debug/joint_sent`,
>   `/joint_states`. Drag e.g. `/debug/joint_sent/FR_KneeP/position` and `/joint_states/FR_KneeP/position`
>   into the same plot. The gap between the lines = tracking error; a time shift = servo lag.

For smooth curves raise the debug rate before launching (edit `hardware.yaml`, rebuild):

```yaml
    debug_rate_hz: 50.0
```
> **Breakdown**
> - Publishes the debug topics every control tick instead of 10×/s. Costs a bit of CPU on the Jetson.

---

## 4. Record a test, look at it later

```bash
ros2 bag record -o trot_01 /cmd_vel /quad/state /joint_command /debug/joint_sent /joint_states /debug/servo_command /debug/servo_feedback /quad/battery_voltage /quad/servo_temperature
```
> **Breakdown**
> - Saves all those topics into folder `trot_01/` until Ctrl-C. Afterwards you can open it in PlotJuggler
>   (*Data → Load*), or replay it with **the robot off**: `ros2 bag play trot_01` + `servo_monitor`.

---

## 5. Turning it off / down

In `config/hardware.yaml`:

```yaml
    debug_topics: false        # no debug topics at all
    debug_rate_hz: 10.0        # or keep them, just slower/faster
    debug_error_warn_deg: 5.0  # when ERR / WARN kicks in
```
> **Breakdown**
> - `debug_topics` and `debug_rate_hz` are read at start (rebuild + restart). `debug_error_warn_deg` can be changed live:
>   `ros2 param set /hardware_bridge debug_error_warn_deg 3.0`.

## 6. In Isaac Sim
There is no `hardware_bridge`, so no `/debug/servo_*` topics. `servo_monitor` notices and compares
`/joint_command` with `/joint_states` directly (header says `SIM / no bridge`).
