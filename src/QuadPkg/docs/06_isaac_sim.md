# 06 · Isaac Sim

The scene is `src/QuadPkg/isaac/scenes/NewQuad.usd`. It references the robot at
`isaac/robot_usd/QuadURDFcad4_0/QuadURDFcad4_0.usd` (relative path, so keep the `isaac/` folder together).

## Run the simulation

1. Isaac Sim → *File → Open* → `isaac/scenes/NewQuad.usd` → **Play**.
2. Terminal 1:

```bash
ros2 launch QuadPkg sim.launch.py
```
> **Breakdown**
> - Starts only `quad_controller`, following Isaac's `/clock` (`use_sim_time:=true`).
> - Isaac's Action Graph subscribes to `/joint_command` and publishes `/joint_states` + `/clock`.

3. Terminal 2:

```bash
ros2 run QuadPkg teleop
```
> **Breakdown**
> - Keyboard driving. **M/K** do nothing in sim (no hardware_bridge) — that's fine.

4. Optional Terminal 3:

```bash
ros2 run QuadPkg servo_monitor
```
> **Breakdown**
> - In sim there are no `/debug/servo_*` topics, so the table compares `/joint_command` (ctrl°) with
>   `/joint_states` (meas°) directly. Big `err°` in sim = drive stiffness/torque too low or the robot is stuck.

## After changing payload.yaml or the URDF: re-import the robot (one time)

```bash
ros2 run QuadPkg export_urdf ~/QuadRobot_with_payload.urdf
```
> **Breakdown**
> - Writes one URDF = robot (zero = standing) + payload boxes, with absolute mesh paths.
> - `~/QuadRobot_with_payload.urdf`: output file in your home folder. Prints total mass + COM.

1. Open `isaac/scenes/NewQuad.usd`, delete the old robot prim `/World/QuadURDFcad4_0`.
2. *File → Import* → the exported URDF. Keep the prim name **QuadURDFcad4_0** (the Action Graph and
   `setup_isaac_scene.py` use that path). "Fix Base Link" **off**; joint drive type **position**.
   Tip: set the importer's output folder to `src/QuadPkg/isaac/robot_usd/` so the new USD lives with the project.
3. *Window → Script Editor → File → Open* → `isaac/scripts/setup_isaac_scene.py` → **Run** (Ctrl+Enter).
   It sets drive stiffness 40 Nm/rad, damping 0.8, torque limit 4 Nm, physics 200 Hz, foot friction,
   equal hip masses, and adds the `/clock` + `/joint_states` publishers.
4. *File → Save*.

> Isaac's torque limit (4 Nm) is higher than the real ST3215 (~2.7 Nm @ 11.1 V). If it walks in sim but not
> on the robot, set `MAX_TORQUE_NM = 2.7` in `setup_isaac_scene.py`, re-run it, and test again — the sim then
> shows the real robot's limits.

## Sim-specific checks

```bash
ros2 run QuadPkg diagnose
```
> **Breakdown**
> - Prints whether `/clock` is ticking and the sim-time/real-time ratio (Isaac often runs at 0.3–0.8×),
>   that exactly one node publishes `/joint_command`, and per-joint tracking error.

| Symptom in sim | Likely cause |
|---|---|
| controller stuck in `STARTUP`, no movement | `/clock` missing → re-run `setup_isaac_scene.py`, or launch with `use_sim_time:=false` |
| robot explodes / flies on Play | robot imported with "Fix Base Link" or a self-colliding pose; re-import |
| legs bounce / oscillate | drive damping too low → re-run `setup_isaac_scene.py` |
| robot sinks / knees buckle | drive stiffness/torque too low or payload too heavy for the set limit |
| walks on the spot | foot friction too low or `step_height` too small; `diagnose` errors > 0.15 rad = drive too weak |
