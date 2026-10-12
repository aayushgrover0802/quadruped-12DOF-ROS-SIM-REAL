# 05 · Real robot bring-up

Do these once, in order. Your laptop can stand in for the Jetson: plug the ESP32 board's USB into
the laptop and every command is identical.

**Before step 1:** run `ros2 run QuadPkg check_robot` (no hardware needed) and read any FAIL/WARN — see [08](08_urdf_and_weight.md).

## 0. Build
See [02 · Build and install](02_build_and_install.md). Short version:

```bash
cd ~/quadruped_ws && colcon build --packages-select QuadPkg && source install/setup.bash
```
> **Breakdown**
> - Go to the workspace, build this package, load it into this terminal. Repeat after every edit.

## 1. Flash the ESP32
Follow `firmware/README.md` (Arduino IDE, or one `esptool.py` command with `firmware/bin/esp32_servo_bridge_merged.bin`).

## 2. Check the servos answer

```bash
ros2 run QuadPkg servo_tool scan
```
> **Breakdown**
> - Lists every servo ID on the bus and compares with `servo_id` in `config/hardware.yaml`.

Expected (your wiring): **1,2,3 = FL knee, thigh, hip · 4,5,6 = FR · 7,8,9 = BL · 10,11,12 = BR**.
A wrong ID → `ros2 run QuadPkg servo_tool setid OLD NEW`. Missing IDs → [10 · Diagnosing parts](10_diagnose_parts.md#4-servo-bus-and-ids).

```bash
ros2 run QuadPkg servo_tool read
```
> **Breakdown**
> - Table of all 12: raw ticks, angle (0 = standing), temperature, voltage, load, torque state.

With the robot posed standing, every angle should be within a few degrees of 0.

## 3. Check every motor direction (robot lifted off the ground!)

```bash
ros2 run QuadPkg servo_tool jog FL_ThighP 10
```
> **Breakdown**
> - Moves ONE joint +10° and back, and prints what you **should** see (e.g. "whole leg swings FORWARD").
> - Joint names: `FL/FR/BL/BR` + `_HipRoll` / `_ThighP` / `_KneeP`.

Wrong way → change that joint's entry in `direction:` from `1.0` to `-1.0` (or back). Left/right servos are
mirrored, so several need `-1.0`. Do all 12, rebuild, re-check any you changed.

Expected motion for a **positive** jog, same for all four legs:

| Joint | Positive means |
|---|---|
| `_HipRoll` | foot swings toward the robot's **right** side |
| `_ThighP` | whole leg swings **forward** |
| `_KneeP` | knee **bends more**, foot comes closer to the body |

## 4. Fine-tune the zero (optional)

```bash
ros2 run QuadPkg servo_tool limp
```
> **Breakdown**
> - Torque off on all 12 so you can pose the legs by hand. Support the robot!

```bash
ros2 run QuadPkg servo_tool calib --write ~/quadruped_ws/src/QuadPkg/config/hardware.yaml
```
> **Breakdown**
> - Hold the standing pose, press ENTER; it averages 10 readings and writes `offset_ticks` into your **source** yaml.
> - Any offset > 200 ticks (17°) is flagged: wrong ID, wrong pose, or a horn mounted one spline off. Rebuild afterwards.

## 5. First stand — with the debug view open

Terminal 1:
```bash
ros2 launch QuadPkg robot.launch.py
```
> **Breakdown**
> - Starts `hardware_bridge` (talks to the ESP32) and `quad_controller`. Motors stay OFF.
> - Read the first lines: "all 12 servos answered", "ESP32 firmware v1", "Loaded … total mass 3.84 kg".

Terminal 2:
```bash
ros2 run QuadPkg servo_monitor
```
> **Breakdown**
> - Live table of what is sent to each servo and what it reports back. Right now it should say
>   **"NOT SENDING: motors OFF"** and show the real angles in the `meas°` column. See [07](07_debug_topics.md).

Terminal 3:
```bash
ros2 run QuadPkg teleop
```
> **Breakdown**
> - Keyboard driver. Press **M**: motors on, the robot glides to standing at `enable_velocity` (0.6 rad/s).
>   Watch Terminal 2: `sent°` moves toward `ctrl°`, `err°` stays small, no flags.

## 6. First steps
Hold the robot just above the ground (feet barely touching), press **W** once (0.05 m/s). Watch the legs cycle
diagonally (FL+BR together, FR+BL together). Then set it down, **SPACE** to stop, **W** again.
If something's off: [11 · Why isn't it walking?](11_why_not_walking.md).

**K** = limp at any time (hold the robot).

> Don't run Isaac Sim on the same `ROS_DOMAIN_ID` while driving the real robot — both would publish `/joint_states`.

## Torque reality check
`check_robot` estimates (one foot carrying 60 % of 3.84 kg in trot) ≈ **2.2 Nm on the knees, 1.4 Nm on the hip-roll**
at the standing pose. ST3215 stall ≈ 2.7 Nm @ 11.1 V, 2.9 Nm @ 12 V, ≈ 1.9 Nm @ 7.4 V. So the knees run near
80 % of stall: walk slowly, keep the battery charged and watch `/quad/servo_temperature`.
`python3 tools/mujoco_check.py --torque 1.9` shows what happens on a weaker supply.
