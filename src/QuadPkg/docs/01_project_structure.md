# 01 · Project structure

```
quadruped_ws/                         ← the workspace (you run colcon build HERE)
├── .gitignore                        ← keeps build/ install/ log/ out of git
└── src/
    └── QuadPkg/                      ← the ROS 2 package
        ├── QuadPkg/                  ← Python code (installed by colcon)
        │   ├── kinematics/leg_kinematics.py   closed-form FK/IK, reads the URDF, numpy only
        │   ├── gait/controller.py             gait brain (sit/stand/trot), NO ROS, unit-tested
        │   ├── gait/gait.py                   ROS node  quad_controller
        │   ├── description/robot_description.py  URDF + payload.yaml → one robot model
        │   ├── description/export_urdf.py     `export_urdf`: URDF with payload for Isaac
        │   ├── teleop/keyboard_teleop.py      `teleop`: keyboard → /cmd_vel, /quad/mode, /quad/enable
        │   ├── hardware/bridge.py             ROS node  hardware_bridge (+ debug topics)
        │   ├── hardware/servo_map.py          radians ⇄ ticks
        │   ├── hardware/protocol.py           USB frame format (no ROS)
        │   ├── hardware/esp32_link.py         serial connection + reader thread (no ROS)
        │   ├── hardware/servo_tool.py         `servo_tool`: scan/read/jog/calib/setid…
        │   └── debug/
        │       ├── diagnose.py                `diagnose`: publishers, /clock, tracking error
        │       ├── servo_monitor.py           `servo_monitor`: live table SENT vs FEEDBACK
        │       ├── monitor_table.py           the table maker (no ROS, tested)
        │       ├── check_robot.py             `check_robot`: mass/COM/torque/URDF checks
        │       ├── lag_meter.py               `lag_meter`: how late / how far each servo follows
        │       └── lag_math.py                its maths (no ROS, tested)
        ├── config/                   ← settings (installed; REBUILD after editing)
        │   ├── quad_params.yaml               how it walks
        │   ├── hardware.yaml                  serial port, servo IDs, directions, offsets, safety, debug
        │   └── payload.yaml                   extra weight on the body
        ├── launch/
        │   ├── sim.launch.py                  controller only (Isaac provides the robot)
        │   └── robot.launch.py                hardware_bridge + controller
        ├── urdf/                     ← robot model (installed to share/QuadPkg/urdf)
        │   ├── QuadURDFcad4_0.urdf            USED: joint 0 = standing
        │   ├── QuadURDFcad4_0_cadzero.urdf    original SolidWorks export (CAD zero), kept as source
        │   └── meshes/*.STL
        ├── isaac/                    ← Isaac Sim files (NOT installed, opened from Isaac)
        │   ├── scenes/NewQuad.usd             the scene you open
        │   ├── robot_usd/QuadURDFcad4_0/      robot USD the scene references
        │   └── scripts/setup_isaac_scene.py   run once in Isaac's Script Editor
        ├── firmware/                 ← ESP32 code
        │   ├── README.md
        │   ├── esp32_servo_bridge/            Arduino sketch
        │   └── bin/esp32_servo_bridge_merged.bin   ready-built firmware
        ├── tools/                    ← one-off scripts run with python3 (not ROS)
        │   ├── rezero_urdf.py                 CAD-zero URDF → standing-zero URDF
        │   └── mujoco_check.py                physics check without Isaac
        ├── test/                     ← python3 -m pytest test/
        ├── docs/                     ← you are here
        ├── legacy/                   ← old Pinocchio IK + old Isaac snippets (reference only)
        ├── setup.py, setup.cfg, package.xml, resource/
        └── README.md
```

## What changed in the clean-up (Oct 2026)

| Before | Now | Why |
|---|---|---|
| `QuadPkg/parser/*.urdf`, `parser/meshes/` | `urdf/` | The robot model isn't Python code; it was hiding inside the Python package |
| `QuadPkg/parser/QuadURDFcad4_0/` (USD) | `isaac/robot_usd/QuadURDFcad4_0/` | Isaac files together. `NewQuad.usd`'s reference was updated to the new path and checked |
| `QuadPkg/isaac_tools/scenes/NewQuad.usd` | `isaac/scenes/NewQuad.usd` | same |
| `QuadPkg/isaac_tools/setup_isaac_scene.py` | `isaac/scripts/setup_isaac_scene.py` | only runs inside Isaac, not a ROS node |
| `QuadPkg/test_fk.py`, `test_ik.py`, `verify_ik_sim.py`, `kinematics/fk.py`, `kinematics/ik.py`, `parser/parser.py`, `results/robot_descriptor.yaml` | `legacy/pinocchio_ik/` | Replaced by `leg_kinematics.py`; nothing used them; they needed Pinocchio |
| `isaac_tools/verify_fk.py`, `scenes/quad4_test_setup.usd` | `legacy/isaac/` | Old; the test scene pointed at a folder that no longer exists |
| `QuadPkg/tools_ros/diagnose.py` | `QuadPkg/debug/diagnose.py` | all debugging tools in one place. Command name unchanged: `ros2 run QuadPkg diagnose` |
| `firmware/esp32_servo_bridge_merged.bin` | `firmware/bin/` | binaries apart from source |
| installed to `share/QuadPkg/parser/` | installed to `share/QuadPkg/urdf/` | matches the folder name |
| — | `QuadPkg/debug/servo_monitor.py`, `check_robot.py` | new debugging tools |
| — | `/debug/servo_command`, `/debug/servo_feedback`, `/debug/joint_sent` | new topics from `hardware_bridge` |
| `stance_x_extra` negative = still pushed feet OUT (bug) | negative now pulls feet IN | `math.copysign` was dropping the sign; fixed in `gait/controller.py` |

> Because install paths changed, do **one clean build** after unpacking (see [02](02_build_and_install.md#clean-rebuild)).
> Otherwise an old `install/QuadPkg/share/QuadPkg/parser/` folder hangs around (harmless, but confusing).

All `ros2 run` / `ros2 launch` command names are the same as before, plus two new ones:
`servo_monitor` and `check_robot`. (`diagnose` moved folders but keeps its name.)
