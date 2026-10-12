# QuadPkg documentation

Every command in these pages has a **Breakdown** box under it that explains each part.
Read them in order the first time; afterwards use the "I want to…" table.

| # | Page | What's in it |
|---|---|---|
| 01 | [Project structure](01_project_structure.md) | Which folder holds what, and what got moved in the clean-up |
| 02 | [Build and install](02_build_and_install.md) | First-time setup, building, sourcing, clean rebuilds |
| 03 | [Command cheat sheet](03_commands.md) | **Every** `ros2 run` / `ros2 launch` / `ros2 topic` command you'll use |
| 04 | [Configuration reference](04_configuration.md) | Every setting in `quad_params.yaml`, `hardware.yaml`, `payload.yaml` |
| 05 | [Real robot bring-up](05_real_robot_bringup.md) | Flash, scan, directions, calibrate, first stand, first walk |
| 06 | [Isaac Sim](06_isaac_sim.md) | Scene, re-importing the robot, running the sim |
| 07 | [Servo debug topics](07_debug_topics.md) | **What you send vs. what the servos send back**, live table, plotting, recording |
| 08 | [URDF and weight](08_urdf_and_weight.md) | What the URDF must look like, **adding / removing weight**, `check_robot` |
| 09 | [Warnings reference](09_warnings_reference.md) | Every warning/error the code prints: what it means, what to do |
| 10 | [Diagnosing parts](10_diagnose_parts.md) | Battery, ESP32, USB, servo bus, each servo, each leg — test one at a time |
| 11 | [Why isn't it walking?](11_why_not_walking.md) | Step-by-step fault finding from "nothing moves" to "walks badly" |

## I want to…

| …do this | go to |
|---|---|
| see the exact command each servo receives and what it reports back | [07](07_debug_topics.md) → `ros2 run QuadPkg servo_monitor` |
| add a lidar / camera / bigger battery | [08 → Adding weight](08_urdf_and_weight.md#3-adding-weight) |
| take something off the robot | [08 → Removing weight](08_urdf_and_weight.md#4-removing-weight) |
| know if the servos are strong enough for the new weight | [08 → check_robot](08_urdf_and_weight.md#2-check_robot-the-model-checker) |
| understand a red/yellow message in the terminal | [09](09_warnings_reference.md) |
| find which part is broken | [10](10_diagnose_parts.md) |
| fix "it stands but doesn't walk" | [11](11_why_not_walking.md) |
| change walking speed / step height | [04 → quad_params.yaml](04_configuration.md#1-configquad_paramsyaml--how-it-walks) |

## The system in one picture

```
                    /cmd_vel, /quad/mode, /quad/enable
  teleop ────────────────────────────────►  quad_controller   (same node for sim and real)
                                                  │ /joint_command (12 joint angles, rad, 0 = STANDING)
                         ┌────────────────────────┴──────────────────────────┐
                         ▼                                                   ▼
             Isaac Sim Action Graph                             hardware_bridge  (Jetson / laptop)
          (ROS2 Subscribe Joint State →                 rad ⇄ ticks, limits, slew rate, health
           Articulation Controller)                         │   ├─► /debug/servo_command  (what is SENT)
                         │ /clock, /joint_states            │   └─► /debug/servo_feedback (what comes BACK)
                         ▼                                  │ USB serial 921600
                  back to controller                        ▼
                                                   ESP32 (firmware/esp32_servo_bridge)  dumb relay
                                                            │ servo bus 1 Mbps
                                                            ▼
                                                     12 x ST3215 servos
```

## Conventions used everywhere

* Body frame (URDF `base_link`): **+X = right, +Y = forward, +Z = up**, metres.
* `/cmd_vel` uses the ROS convention: `linear.x` forward, `linear.y` left, `angular.z` turn left.
* Joint angles are radians, **0 = standing pose** (the URDF was re-zeroed for this).
* Joint names: `FL/FR/BL/BR` + `_HipRoll` / `_ThighP` / `_KneeP`.
  * HipRoll **+** → foot moves to the robot's right · ThighP **+** → leg swings forward · KneeP **+** → knee bends more.
* Servo ticks: 0…4095 per turn, **2048 = middle**, 1 tick = 0.088°.
  `ticks = 2048 + offset_ticks + direction × gear_ratio × angle / (2π/4096)`.
