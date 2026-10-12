# 04 · Configuration reference

Three files in `src/QuadPkg/config/`. **After editing any of them: rebuild** —

```bash
cd ~/quadruped_ws && colcon build --packages-select QuadPkg && source install/setup.bash
```
> **Breakdown**
> - Nodes read the **installed** copy in `install/QuadPkg/share/QuadPkg/config/`, not the one you edited in `src/`.
>   The build copies it over. Then restart the launch.

### YAML rules that bite

```yaml
quad_controller:          # must equal the node name
  ros__parameters:        # two underscores; everything below is a parameter
    stand_height: 0.155   # decimals stay decimals
    gait_period: 1.0      # write 1.0, NOT 1  (ROS 2 refuses int where float was declared)
    servo_id: [3, 2, 1]   # lists of whole numbers: no ".0"
```
> **Breakdown**
> - Line 1: the block is applied to the node with this name (`quad_controller`, `hardware_bridge`).
> - Line 2: `ros__parameters` is a fixed keyword.
> - Lines 3–5: ROS 2 checks types. A float parameter written as `1` crashes the node at start with
>   *"Trying to set parameter … to … of type INTEGER, expecting type DOUBLE"*. Indentation is 2 spaces, never tabs.

---

## 1. `config/quad_params.yaml` — how it walks

Read by `quad_controller` (sim **and** real). Read once at start.

| Parameter | Value | What it does | Raise it → | Lower it → |
|---|---|---|---|---|
| `rate_hz` | 50.0 | Control loop frequency | smoother, more CPU | jerkier; keep = bridge `rate_hz` |
| `cmd_timeout` | 0.5 | s without `/cmd_vel` before it stops walking | keeps walking longer after teleop dies | stops sooner; must be > teleop period 0.1 s |
| `startup_wait` | 3.0 | s to wait for first `/joint_states` | — | starts blind from zero pose if no feedback |
| `stand_height` | 0.155 | body height above feet (m), = joint-zero pose | taller, needs more knee torque, less reach for steps | crouched, more torque margin; **changing it moves the stand pose away from servo middle** |
| `sit_height` | 0.05 | body height when sitting (m) | — | must stay reachable (`check_robot` tells you) |
| `stance_x_extra` | 0.0 | extra sideways foot spacing (m) | wider = stable, more HipRoll torque | narrower = less HipRoll torque, tips easier |
| `stance_y_shift` | *(not set = auto)* | fore/aft foot shift; auto puts feet centre under the COM | only set by hand if auto is wrong | — |
| `height_transition_time` | 2.5 | s for sit ↔ stand | gentler | faster, more peak torque |
| `startup_time` | 3.0 | s to blend from measured pose to start pose | gentler | — |
| `start_mode` | "stand" | pose after startup: `"stand"` or `"sit"` | — | — |
| `gait_period` | 0.85 | s for one full step cycle (all 4 legs) | slower steps, more tipping per step | quicker steps, needs faster servos |
| `duty_factor` | 0.65 | fraction of the cycle each foot is on the ground (> 0.5 = moments with 4 feet down) | more stable, shorter swing (faster leg moves) | less stable |
| `step_height` | 0.02 | foot lift (m) | clears carpet/bumps, more torque | may scuff and trip |
| `max_step_length` | 0.06 | clamp on foot travel per step (m) | longer strides | short strides |
| `max_vx` | 0.10 | max forward speed (m/s) | faster | — |
| `max_vy` | 0.04 | max sideways speed (m/s); must be > `cmd_deadband` | — | — |
| `max_wz` | 0.30 | max turn rate (rad/s) | — | — |
| `max_lin_accel` | 0.3 | m/s² speed ramp | snappier, more tipping | smoother starts/stops |
| `max_ang_accel` | 1.5 | rad/s² turn ramp | — | — |
| `cmd_deadband` | 0.02 | speeds below this count as "stop" (m/s) | ignores tiny commands | tiny commands start a trot that does nothing |

Rule of thumb when it walks badly: **slower first** — `gait_period: 1.0`, `max_vx: 0.06`, `step_height: 0.025`.

---

## 2. `config/hardware.yaml` — real robot only

Read by `hardware_bridge` and `servo_tool`. Index *i* in every list belongs to `joint_order[i]`.

### Connection
| Parameter | Value | Meaning |
|---|---|---|
| `port` | "/dev/ttyUSB0" | ESP32 serial port (`ls /dev/ttyUSB* /dev/ttyACM*`) |
| `baud` | 921600 | must equal `HOST_BAUD` in the firmware |
| `feedback_hz` | 50 | how often the ESP32 reads all servos and sends STATE |
| `watchdog_ms` | 500 | ESP32 acts if the Jetson is silent this long |
| `watchdog_torque_off` | false | false = hold last pose, true = go limp |

### Servo mapping
| Parameter | Meaning | How to find it |
|---|---|---|
| `joint_order` | fixed order of the 12 joints for every list below | don't change |
| `servo_id` | bus ID of each joint's servo. Your wiring: FL = 3,2,1 (hip,thigh,knee), FR = 6,5,4, BL = 9,8,7, BR = 12,11,10 | `servo_tool scan` |
| `direction` | +1.0 / -1.0: which way the servo turns for a + joint angle | `servo_tool jog <joint> 10` |
| `offset_ticks` | correction so the standing pose = 0 rad (1 tick = 0.088°) | `servo_tool calib --write …` |
| `gear_ratio` | 1.0 unless there's a belt/gear between servo and joint | — |
| `tick_min` / `tick_max` | hard clamp on any tick ever sent | 0 / 4095 |

### Motion safety
| Parameter | Value | Meaning | Live-settable? |
|---|---|---|---|
| `rate_hz` | 50.0 | how often targets go to the ESP32 | no |
| `max_velocity` | 8.0 | rad/s slew limit per joint while running | yes |
| `enable_velocity` | 0.6 | rad/s slew limit during the first seconds after enable | yes |
| `enable_ramp_time` | 4.0 | s of slow mode after enable | yes |
| `servo_speed` / `servo_acc` | 0 / 0 | ST3215 speed/acceleration registers (0 = max; the Jetson already smooths) | yes |
| `auto_enable` | false | torque on at start | no |
| `torque_off_on_exit` | false | Ctrl-C: false = keep holding, true = limp | yes |

### Health warnings
| Parameter | Value | Meaning |
|---|---|---|
| `max_temp_c` | 65 | warn above this servo temperature (°C) |
| `min_voltage` | 10.5 | warn below this supply voltage (3S LiPo: 12.6 full, 11.1 nominal, ~10.5 time to charge) |

### Debug topics (new)
| Parameter | Value | Meaning |
|---|---|---|
| `debug_topics` | true | publish `/debug/servo_command`, `/debug/servo_feedback`, `/debug/joint_sent` |
| `debug_rate_hz` | 10.0 | how often. 10 = readable; 50 = every tick (plots, bags) |
| `debug_error_warn_deg` | 5.0 | feedback status turns WARN when \|sent − measured\| > this |

---

## 3. `config/payload.yaml` — extra weight on the body

Read by `quad_controller` (COM → foot placement), `export_urdf` (Isaac) and `check_robot`.
Full guide with examples: [08 · URDF and weight](08_urdf_and_weight.md).

```yaml
payload:
  - name: battery              # any label; becomes link "payload_battery"
    enabled: true              # false = ignored, block kept
    mass: 0.65                 # kg
    xyz: [0.0, 0.0, 0.020]     # centre, metres, base_link: +X right, +Y forward, +Z up
    size: [0.045, 0.140, 0.035]  # outer box (m), only for rotational inertia + Isaac's grey box
```
> **Breakdown**
> - `payload:` the list. Each `- name:` starts one item.
> - `mass` in **kilograms** (170 g → `0.17`).
> - `xyz`: where the item's centre sits. `(0, 0, z)` = middle of the body between the 4 hips.
>   Body shell ≈ X −0.099…+0.099, Y −0.170…+0.190, Z −0.005…+0.080.
> - `size`: rough box. A guess is fine.
