# 03 · Command cheat sheet

Every command assumes a terminal where you've already done
`cd ~/quadruped_ws && source install/setup.bash`.

**Contents:** [Start the robot](#start-the-robot) · [Drive without the keyboard](#drive-without-the-keyboard) ·
[Setup: servo_tool](#setup-servo_tool) · [Model: check_robot / export_urdf](#model-check_robot--export_urdf) ·
[Debug](#debug) · [Look at topics, nodes, parameters](#look-at-topics-nodes-parameters) ·
[Record and replay](#record-and-replay) · [Offline tools](#offline-tools-no-ros)

---

## Start the robot

```bash
ros2 launch QuadPkg robot.launch.py
```
> **Breakdown**
> - `ros2 launch`: start several nodes from a recipe file.
> - `QuadPkg`: the package. `robot.launch.py`: starts `hardware_bridge` + `quad_controller` for the **real robot**.
> - Motors stay **OFF** until you press **M** in teleop (or publish `/quad/enable`).

```bash
ros2 launch QuadPkg robot.launch.py port:=/dev/ttyACM0 auto_enable:=true payload_file:=/home/$USER/my_payload.yaml
```
> **Breakdown**
> - `name:=value`: a launch argument; overrides the default for this run only.
> - `port:=/dev/ttyACM0`: ESP32 on a different USB name (find it with `ls /dev/ttyUSB* /dev/ttyACM*`).
> - `auto_enable:=true`: torque on automatically 1 s after start. **Only after calibration!**
> - `payload_file:=…`: use another payload list (e.g. for an experiment) instead of the installed one.
>   Use a full path; `~` is not expanded inside `:=`.

```bash
ros2 launch QuadPkg sim.launch.py
```
> **Breakdown**
> - `sim.launch.py`: starts only `quad_controller`, following Isaac Sim's `/clock`. Isaac itself plays the robot.
> - Add `use_sim_time:=false` if your Isaac scene has no clock publisher (then the gait runs on wall time).

```bash
ros2 run QuadPkg teleop
```
> **Breakdown**
> - `ros2 run`: start **one** program from a package. `teleop`: keyboard driver; needs its own terminal.
> - Keys: **M** motors on · **K** motors off (limp, hold the robot!) · **T** stand · **G** sit ·
>   **W/X** faster forward/back · **A/D** strafe left/right · **Q/E** turn left/right · **SPACE** stop.
> - Each W/X/A/D press adds 0.05 m/s; Q/E adds 0.25 rad/s. The controller clips to `max_vx/max_vy/max_wz`.

```bash
ros2 run QuadPkg teleop --ros-args -p lin_step:=0.02 -p ang_step:=0.1
```
> **Breakdown**
> - `--ros-args`: everything after this is for ROS, not for the program.
> - `-p name:=value`: set a node **p**arameter. Smaller steps = finer speed control.

## Drive without the keyboard

```bash
ros2 topic pub --once /quad/enable std_msgs/msg/Bool "{data: true}"
```
> **Breakdown**
> - `ros2 topic pub`: send a message on a topic from the terminal.
> - `--once`: send one message and quit.
> - `/quad/enable`: the bridge's motor switch. `std_msgs/msg/Bool`: the message type (true/false).
> - `"{data: true}"`: the message content, in YAML. `false` = limp.

```bash
ros2 topic pub --once /quad/mode std_msgs/msg/String "{data: stand}"
```
> **Breakdown**
> - `/quad/mode`: ask the controller for `stand` or `sit`.

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```
> **Breakdown**
> - `-r 10`: repeat at 10 Hz until Ctrl-C. Needed: the controller stops walking if `/cmd_vel` is silent for `cmd_timeout` (0.5 s).
> - `geometry_msgs/msg/Twist`: velocity message. `linear.x: 0.05` = walk forward at 5 cm/s.
>   `linear.y` = sideways (left +), `angular.z` = turn (left +). Unset fields are 0.
> - Ctrl-C stops publishing → the robot stops after 0.5 s.

## Setup: servo_tool

Stop `robot.launch.py` first — only one program can hold the serial port.

```bash
ros2 run QuadPkg servo_tool scan
```
> **Breakdown**
> - `scan`: ping IDs 1–253, list who answers, compare with `servo_id` in `hardware.yaml`.

```bash
ros2 run QuadPkg servo_tool read
```
> **Breakdown**
> - `read`: one table of all 12 joints: raw ticks, angle (0 = standing), °C, volts, load %, torque on/off.

```bash
ros2 run QuadPkg servo_tool limp
```
> **Breakdown**
> - `limp`: torque OFF on all 12 — legs move freely by hand. **Support the robot.**

```bash
ros2 run QuadPkg servo_tool hold
```
> **Breakdown**
> - `hold`: torque ON, holding wherever the legs are right now.

```bash
ros2 run QuadPkg servo_tool stand
```
> **Breakdown**
> - `stand`: glide every joint to 0 (= standing pose) over 3 s, without ROS/controller. Good first motor test.

```bash
ros2 run QuadPkg servo_tool jog FL_KneeP 10
```
> **Breakdown**
> - `jog`: move **one** joint by +10° and back, printing which way it **should** move.
> - Wrong way → flip that joint's `direction` in `config/hardware.yaml` (1.0 ↔ -1.0), rebuild.
> - Allowed range ±30°. Lift the robot off the ground first.

```bash
ros2 run QuadPkg servo_tool calib --write ~/quadruped_ws/src/QuadPkg/config/hardware.yaml
```
> **Breakdown**
> - `calib`: you hold the robot in the standing pose, it averages 10 readings, prints `offset_ticks`.
> - `--write FILE`: puts the numbers straight into the `offset_ticks:` line of the **source** file. Rebuild afterwards.

```bash
ros2 run QuadPkg servo_tool middle all
```
> **Breakdown**
> - `middle`: make the **current** position the servo's new 2048 (stored inside the servo, like the web-app button).
> - `all` or one ID (`middle 4`). Asks you to type `yes`. Afterwards set those `offset_ticks` to 0.

```bash
ros2 run QuadPkg servo_tool setid 1 13
```
> **Breakdown**
> - `setid OLD NEW`: change a servo's ID (stored in its EEPROM). Connect only that servo if two share an ID.

```bash
ros2 run QuadPkg servo_tool --port /dev/ttyACM0 --config ~/quadruped_ws/src/QuadPkg/config/hardware.yaml read
```
> **Breakdown**
> - `--port`: override the yaml's serial port. `--config`: use this hardware.yaml (here the **source** one,
>   so you test an edit before rebuilding). Options go **before** the command word (`read`).

## Model: check_robot / export_urdf

```bash
ros2 run QuadPkg check_robot
```
> **Breakdown**
> - `check_robot`: checks URDF + payload + gait settings without any robot: structure, mass table, centre of mass,
>   payload placement, estimated torque per joint vs. servo stall torque, stand-pose reachability. See [08](08_urdf_and_weight.md).

```bash
ros2 run QuadPkg check_robot --add lidar 0.17 0 0.08 0.10 --remove jetson --stall 2.9
```
> **Breakdown**
> - `--add NAME KG X Y Z`: "what if I add this?" without editing `payload.yaml` (metres, +X right, +Y forward, +Z up).
> - `--remove NAME`: "what if I take this off?". Both can repeat.
> - `--stall 2.9`: your servo's stall torque in Nm at your battery voltage (default 2.72 = ST3215 @ 11.1 V).

```bash
ros2 run QuadPkg export_urdf ~/QuadRobot_with_payload.urdf
```
> **Breakdown**
> - `export_urdf`: writes ONE URDF = robot + everything in `payload.yaml`, with absolute mesh paths, for Isaac's importer.
> - Prints total mass and centre of mass. Re-run and re-import whenever `payload.yaml` changes.

## Debug

```bash
ros2 run QuadPkg servo_monitor
```
> **Breakdown**
> - `servo_monitor`: live table, one row per servo: controller wish → sent angle → sent ticks → returned ticks →
>   measured angle → error, load, °C, V and flags. Read-only, safe to run anytime. See [07](07_debug_topics.md).

```bash
ros2 run QuadPkg servo_monitor --view cmd
```
> **Breakdown**
> - `--view cmd`: only the **sending** side. `--view fb`: only the **feedback** side. Default `both`.

```bash
ros2 run QuadPkg servo_monitor --once > snapshot.txt
```
> **Breakdown**
> - `--once`: listen 2 s, print one table, quit. `> snapshot.txt`: save it into a file instead of the screen.

```bash
ros2 run QuadPkg lag_meter --seconds 10
```
> **Breakdown**
> - `lag_meter`: listens for 10 s while you walk the robot forward, then prints per servo: how many ms late it follows
>   the command, how much of the movement it really makes (`reach %`), and a suggested `gait_period`. Read-only.
> - It also names any joint that is clearly later/shorter than its left-right twin (a problem in that leg).

```bash
ros2 run QuadPkg diagnose
```
> **Breakdown**
> - `diagnose`: every 2 s: how many nodes publish `/joint_command` (must be 1), is `/clock` ticking,
>   controller state, worst tracking error per joint. Works in sim and on the real robot.

## Look at topics, nodes, parameters

```bash
ros2 node list
```
> **Breakdown**
> - Lists running nodes. Real robot: `/hardware_bridge`, `/quad_controller`, `/keyboard_teleop` (+ `/servo_monitor`, `/quad_diagnose` if running).

```bash
ros2 topic list
```
> **Breakdown**
> - Lists every topic that currently has a publisher or subscriber.

```bash
ros2 topic echo /quad/state
```
> **Breakdown**
> - `echo`: print every message on a topic. `/quad/state`: STARTUP / SIT / STAND / WALK.

```bash
ros2 topic echo --once /debug/servo_feedback
```
> **Breakdown**
> - Print **one** feedback message (all 12 servos with every value) and quit.

```bash
ros2 topic hz /joint_command
```
> **Breakdown**
> - `hz`: measure how often messages arrive. Expect ~50 Hz for `/joint_command` and `/joint_states`.

```bash
ros2 topic info -v /joint_states
```
> **Breakdown**
> - `info`: who publishes / subscribes. `-v`: verbose (node names, QoS).
>   On the real robot exactly **one** publisher (`hardware_bridge`). Two = Isaac is also running on your domain.

```bash
ros2 param list /hardware_bridge
```
> **Breakdown**
> - Show every parameter the running node has.

```bash
ros2 param get /hardware_bridge offset_ticks
```
> **Breakdown**
> - `get`: print the value the node is **actually** using (catches "I edited the yaml but forgot to rebuild").

```bash
ros2 param set /hardware_bridge max_velocity 4.0
```
> **Breakdown**
> - `set`: change a parameter while running. Works for values the node re-reads every tick:
>   `max_velocity`, `enable_velocity`, `enable_ramp_time`, `servo_speed`, `servo_acc`, `max_temp_c`, `min_voltage`,
>   `debug_error_warn_deg`. Everything else (IDs, directions, offsets, gait parameters) is read only at start →
>   edit the yaml, rebuild, restart. Live changes are lost on restart.

```bash
ros2 param dump /quad_controller
```
> **Breakdown**
> - `dump`: print all current parameters of the node as YAML (copy into `quad_params.yaml` if you like them).

## Record and replay

```bash
ros2 bag record -o walk_test /cmd_vel /quad/state /joint_command /debug/joint_sent /joint_states /debug/servo_command /debug/servo_feedback /quad/battery_voltage
```
> **Breakdown**
> - `ros2 bag record`: save the listed topics to disk while you test. Ctrl-C to stop.
> - `-o walk_test`: name of the folder it creates.
> - The topic list: commands in, what was sent, what came back, battery. Enough to debug a walk afterwards.
> - For smooth plots set `debug_rate_hz: 50.0` in `hardware.yaml` first (10 Hz default is for reading).

```bash
ros2 bag play walk_test
```
> **Breakdown**
> - Re-publishes the recording. **Don't** run it while the robot is live (it would publish `/cmd_vel` and `/joint_command`!).
>   Look at it with `servo_monitor` or PlotJuggler instead.

## Offline tools (no ROS)

```bash
cd ~/quadruped_ws/src/QuadPkg && python3 tools/mujoco_check.py --torque 1.9
```
> **Breakdown**
> - `tools/mujoco_check.py`: run the same gait in the MuJoCo physics engine (needs `pip install mujoco`).
> - `--torque 1.9`: limit the simulated servos to 1.9 Nm (e.g. 7.4 V battery) to see if it still walks.

```bash
cd ~/quadruped_ws/src/QuadPkg && python3 tools/rezero_urdf.py urdf/QuadURDFcad4_0_cadzero.urdf urdf/QuadURDFcad4_0.urdf --stand-height 0.155 --y-shift -0.016
```
> **Breakdown**
> - `rezero_urdf.py`: rewrites the URDF so all joints at 0 = standing pose. Only needed after a **new CAD export**
>   or a new stand height. Then also re-set the servo middles and Isaac import. See [08](08_urdf_and_weight.md#6-new-cad-export).
