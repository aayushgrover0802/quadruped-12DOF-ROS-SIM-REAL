# QuadPkg: quadruped trot controller (Isaac Sim + real robot)

One ROS 2 package drives the robot in **Isaac Sim** and on the **real robot** (Jetson + ESP32 + 12 × ST3215)
with exactly the same controller code. **Full documentation: [`docs/`](docs/README.md).**

```
  teleop ──/cmd_vel, /quad/mode, /quad/enable──►  quad_controller ──/joint_command──┬──► Isaac Sim
                                                                                     └──► hardware_bridge ──USB──► ESP32 ──► 12 servos
                                                                                           ├─► /debug/servo_command   (what is SENT)
                                                                                           └─► /debug/servo_feedback  (what comes BACK)
```

## Quick start

```bash
cd ~/quadruped_ws && colcon build --packages-select QuadPkg && source install/setup.bash
```
> **Breakdown**
> - Build the package and load it into this terminal. Repeat after every edit (nodes read the installed copy).

| Goal | Command | Docs |
|---|---|---|
| Check the robot model (mass, COM, torque) | `ros2 run QuadPkg check_robot` | [08](docs/08_urdf_and_weight.md) |
| Run the real robot | `ros2 launch QuadPkg robot.launch.py` | [05](docs/05_real_robot_bringup.md) |
| Run with Isaac Sim | `ros2 launch QuadPkg sim.launch.py` | [06](docs/06_isaac_sim.md) |
| Drive | `ros2 run QuadPkg teleop` (M motors on, T stand, W/X/A/D/Q/E move, SPACE stop, K limp) | [03](docs/03_commands.md) |
| See sent vs. returned per servo | `ros2 run QuadPkg servo_monitor` | [07](docs/07_debug_topics.md) |
| Setup / calibrate servos | `ros2 run QuadPkg servo_tool scan\|read\|jog\|calib…` | [05](docs/05_real_robot_bringup.md) |
| Health report | `ros2 run QuadPkg diagnose` | [11](docs/11_why_not_walking.md) |

## Docs index

| | |
|---|---|
| [01 Project structure](docs/01_project_structure.md) | [07 Servo debug topics](docs/07_debug_topics.md) |
| [02 Build and install](docs/02_build_and_install.md) | [08 URDF and weight](docs/08_urdf_and_weight.md) |
| [03 Command cheat sheet](docs/03_commands.md) | [09 Warnings reference](docs/09_warnings_reference.md) |
| [04 Configuration reference](docs/04_configuration.md) | [10 Diagnosing parts](docs/10_diagnose_parts.md) |
| [05 Real robot bring-up](docs/05_real_robot_bringup.md) | [11 Why isn't it walking?](docs/11_why_not_walking.md) |
| [06 Isaac Sim](docs/06_isaac_sim.md) | [firmware/README](firmware/README.md) |

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | teleop → controller |
| `/quad/mode` | std_msgs/String (`sit` / `stand`) | teleop → controller |
| `/quad/state` | std_msgs/String | controller → you |
| `/joint_command` | sensor_msgs/JointState | controller → Isaac / bridge |
| `/joint_states` | sensor_msgs/JointState | Isaac / bridge → controller (bridge: effort = load %) |
| `/quad/enable` | std_msgs/Bool | you / teleop → bridge (motors off until `true`) |
| `/quad/motors_enabled` | std_msgs/Bool | bridge → you |
| `/quad/battery_voltage` | std_msgs/Float32 | bridge → you |
| `/quad/servo_temperature` | std_msgs/Float32MultiArray | bridge → you (°C, joint order) |
| `/debug/servo_command` | diagnostic_msgs/DiagnosticArray | bridge → you: per servo, what is sent and why |
| `/debug/servo_feedback` | diagnostic_msgs/DiagnosticArray | bridge → you: per servo, what comes back + problems |
| `/debug/joint_sent` | sensor_msgs/JointState | bridge → you: sent angles, for plotting vs `/joint_states` |

## History

* **Oct 2026, v0.2 (this version):** folder clean-up (`urdf/`, `isaac/`, `legacy/`, `QuadPkg/debug/`), `docs/` folder,
  servo command/feedback debug topics + `servo_monitor`, `check_robot` model checker, `stance_x_extra` sign bug fixed.
  See [docs/01](docs/01_project_structure.md#what-changed-in-the-clean-up-oct-2026).
* **Oct 2026, second update:** URDF re-zeroed (0 = standing), start in STAND, `payload.yaml` (2.0 kg), ESP32 firmware +
  `hardware_bridge` + `servo_tool`, teleop M/K.
* **Oct 2026, first update:** closed-form IK (verified against Pinocchio < 0.01 mm), `/cmd_vel` gait with cycloid swing,
  60 % duty trot, velocity ramps, COM-centred feet, `/clock` following, URDF hip masses fixed, Isaac damping fixed.
