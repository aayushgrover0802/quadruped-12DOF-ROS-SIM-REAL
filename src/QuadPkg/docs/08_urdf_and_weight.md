# 08 · URDF requirements, adding / removing weight

**Contents:** [1. What the URDF must contain](#1-what-the-urdf-must-contain) ·
[2. check_robot](#2-check_robot-the-model-checker) · [3. Adding weight](#3-adding-weight) ·
[4. Removing weight](#4-removing-weight) · [5. A printed part got heavier/lighter](#5-a-printed-part-or-servo-changed-weight) ·
[6. New CAD export](#6-new-cad-export) · [7. Weight budget](#7-weight-budget-how-much-can-it-carry) ·
[8. check_robot warnings explained](#8-check_robot-warnings-explained)

There are **two places** weight lives:

| Where | For what | Edit when |
|---|---|---|
| `urdf/QuadURDFcad4_0.urdf` | the printed body, legs, servos — from CAD | a **part of the robot itself** changes |
| `config/payload.yaml` | things **bolted onto** the body: battery, Jetson, wiring, lidar… | you add/remove/move equipment |

Every node merges the two (`description/robot_description.py`), so the controller, `export_urdf` (Isaac)
and `check_robot` always agree. **Prefer `payload.yaml`** — you never touch the URDF for equipment.

---

## 1. What the URDF must contain

The controller (`kinematics/leg_kinematics.py`) reads the URDF directly and **refuses to start** if these
aren't true. `check_robot` section 1 checks them for you.

| Requirement | Your file | Why |
|---|---|---|
| Root link named `base_link` | ✔ | all positions are measured from it |
| 12 revolute joints named `<LEG>_HipRoll`, `<LEG>_ThighP`, `<LEG>_KneeP` with LEG = FL, FR, BL, BR | ✔ | the code looks them up by name |
| Chain per leg: `base_link → <LEG>_Hip → <LEG>_Thigh → <LEG>_Knee → <LEG>_Foot` | ✔ | |
| A link `<LEG>_Foot` on a **fixed** joint at the tip of each leg | ✔ (`<LEG>_FixedFoot`) | its origin is the foot contact point used by IK |
| HipRoll axis along the body's forward axis (±Y); ThighP and KneeP axes along the side axis (±X) | ✔ | closed-form IK assumes a "roll + 2-link planar" leg |
| Every joint has `<limit lower=… upper=…>` | ✔ | commands are clamped to these (flag `LIM` in the debug view) |
| Every link has `<inertial>` with `mass > 0` and positive inertia | ✔ (17 links) | centre of mass → foot placement; Isaac/MuJoCo need it |
| **All joints at 0 = standing pose** | ✔ (re-zeroed) | must match servo middle (tick 2048) set with the robot standing |
| Mesh paths relative (`./meshes/…`) | ✔ | so the package can be installed anywhere |
| Frame: +X right, +Y forward, +Z up | ✔ | payload `xyz`, `/cmd_vel` conversion and directions depend on it |

The anatomy of one link (what you'd edit for a weight change):

```xml
<link name="FR_Knee">
  <inertial>
    <origin xyz="0.0326 -0.0213 -0.0596" rpy="0 0 0" />   <!-- centre of mass of this part -->
    <mass value="0.0529" />                                 <!-- kilograms -->
    <inertia ixx="…" ixy="…" ixz="…" iyy="…" iyz="…" izz="…" />  <!-- how hard it is to spin, kg·m² -->
  </inertial>
  <visual> … mesh … </visual>
  <collision> … mesh … </collision>
</link>
```
> **Breakdown**
> - `<link name>`: one rigid part. `<inertial>`: everything about its weight.
> - `<origin xyz>`: where the part's centre of mass is, in the part's own frame (metres).
> - `<mass value>`: kilograms. `<inertia>`: rotational inertia; scale it together with the mass (section 5).
> - `<visual>`/`<collision>`: the shape for drawing/contact. Weight changes don't touch these.

---

## 2. check_robot: the model checker

Run it **every time** you change weight or the URDF, before powering the robot. No robot or ROS graph needed.

```bash
ros2 run QuadPkg check_robot
```
> **Breakdown**
> - Reads the **installed** URDF, `payload.yaml` and `quad_params.yaml` (so rebuild first after edits).
> - Exit code 0 = all OK, 1 = warnings, 2 = errors.

```bash
cd ~/quadruped_ws/src/QuadPkg && python3 -m QuadPkg.debug.check_robot
```
> **Breakdown**
> - Same tool straight from the **source** folder, no rebuild: it picks up `urdf/` and `config/` next to it.
>   Handy while you're still editing. (`python3 -m` = run a module of the package as a program.)

What it prints today (your robot with battery + Jetson + wiring):

```
=== 2. Mass table
         body (base_link)        0.917 kg
         leg FL / FR / BL / BR   0.232 kg each
         payload battery         0.650 kg  at (+0.000, +0.000, +0.020) m
         payload jetson          0.650 kg  at (+0.000, +0.000, +0.060) m
         payload wiring_buffer   0.700 kg  at (+0.000, +0.000, +0.040) m
         TOTAL                   3.843 kg   (payload 2.000 kg)
=== 3. Centre of mass (standing pose)
  [ OK ] COM sideways offset -0.1 mm (< 5 mm)
  [ OK ] COM fore/aft offset -0.0 mm after auto-shift
=== 5. Estimated holding torque per joint (trot, one foot carrying 60% of 3.84 kg)
         FR_HipRoll     1.41 Nm     52 %  <-- high
         FR_KneeP       2.17 Nm     80 %  <-- TOO HIGH
  [FAIL] BR_KneeP needs 80% of stall torque just to hold the body
```

**Read that last line seriously:** with 2 kg of payload the knees work at ~80 % of the ST3215's stall torque
(2.72 Nm @ 11.1 V) every time a foot carries the body. Servos in that zone get hot, sag a few degrees and
can't lift the foot cleanly — a classic cause of "stands but won't walk". See [section 7](#7-weight-budget-how-much-can-it-carry).

Try changes **before** editing anything:

```bash
ros2 run QuadPkg check_robot --add lidar 0.17 0 0.08 0.10
```
> **Breakdown**
> - `--add NAME KG X Y Z`: pretend a 170 g item sits 8 cm forward, 10 cm up (on top of the body). Repeatable.

```bash
ros2 run QuadPkg check_robot --remove wiring_buffer --add wiring 0.35 0 0 0.04
```
> **Breakdown**
> - `--remove NAME`: pretend an item from `payload.yaml` is gone. Here: "what if the 0.70 kg wiring guess is really 0.35 kg?"

```bash
ros2 run QuadPkg check_robot --stall 2.9
```
> **Breakdown**
> - `--stall`: your servo's stall torque in Nm. ST3215 ≈ 2.9 @ 12 V, 2.72 @ 11.1 V, ≈ 1.9 @ 7.4 V.
>   Use the voltage your battery has **under load** (watch `/quad/battery_voltage`).

---

## 3. Adding weight

Example: a 170 g lidar on top of the body, 8 cm forward.

**Step 1 — weigh it.** Kitchen scale, include its bracket and screws. Write it in **kg**: 170 g → `0.17`.

**Step 2 — measure where its centre is**, relative to `base_link`'s origin:
the origin is the **middle point between the 4 hip joints** (left/right and front/back), and
**32 mm below** the hip-joint axles.

| Axis | Direction | Your body's range |
|---|---|---|
| X | + = robot's **right** | shell −0.099 … +0.099 m, hips at ±0.071 |
| Y | + = **forward** | shell −0.170 … +0.190 m, hips at ±0.075 |
| Z | + = **up** | shell −0.005 … +0.080 m (top of shell ≈ 0.08) |

Lidar 8 cm forward of the middle, its centre ~2 cm above the 8 cm-high shell top → `xyz: [0.0, 0.08, 0.10]`.

**Step 3 — test it without editing:**

```bash
ros2 run QuadPkg check_robot --add lidar 0.17 0 0.08 0.10
```
> **Breakdown**
> - Shows the new total mass, where the COM moved, how far the feet must shift, and the new joint torques.

**Step 4 — add it to `config/payload.yaml`:**

```yaml
  - name: lidar
    enabled: true
    mass: 0.17
    xyz: [0.0, 0.08, 0.10]
    size: [0.07, 0.10, 0.06]
```
> **Breakdown**
> - Indent exactly like the other items (2 spaces, then `- name:`).
> - `size`: rough outer box in metres (x width, y length, z height). Only used for rotational inertia and Isaac's grey box.

**Step 5 — rebuild and check:**

```bash
cd ~/quadruped_ws && colcon build --packages-select QuadPkg && source install/setup.bash && ros2 run QuadPkg check_robot
```
> **Breakdown**
> - Build (copies the new payload.yaml into `install/`), load it, run the checker on the installed files.

**Step 6 — what happens automatically:** at start the controller logs
`Loaded … total mass 4.01 kg, COM at zero pose = (…)` and `stance shifted +x.x mm to centre feet under COM`.
It moves all four feet so their centre is under the new COM. **No servo recalibration needed.**

**Step 7 — Isaac Sim** (if you use it): `ros2 run QuadPkg export_urdf ~/QuadRobot_with_payload.urdf` and re-import ([06](06_isaac_sim.md)).

Rules of thumb:
* Keep `x` ≈ 0. A side-heavy robot tips to that side every trot step (check_robot warns above 5 mm COM offset).
* Heavy things **low** and near the middle. Every 1 cm higher makes trotting wobblier.
* A big fore/aft offset (feet shift > 30 mm) means the stand pose drifts away from servo middle — consider moving the item
  or re-running the re-zero (section 6).

---

## 4. Removing weight

Temporary (keep the numbers for later):

```yaml
  - name: jetson
    enabled: false        # ← ignored everywhere, block kept
    mass: 0.65
    ...
```
> **Breakdown**
> - `enabled: false`: the item is skipped by the controller, `export_urdf` and `check_robot`. Flip back to `true` to restore.

Permanent: delete the whole block (from `- name:` down to its `size:` line). Then rebuild (+ Isaac re-export).

Test first without touching the file:

```bash
ros2 run QuadPkg check_robot --remove jetson
```
> **Breakdown**
> - Pretends `jetson` isn't there and shows the new mass, COM and torques.

Bare robot, no payload at all (e.g. bench testing the legs):

```bash
ros2 launch QuadPkg robot.launch.py payload_file:=/dev/null
```
> **Breakdown**
> - `/dev/null` is an always-empty file → the payload list is empty → controller uses the bare 1.84 kg robot.
>   (An empty `payload_file:=` means "use the default file", so `/dev/null` is the trick to say "none".)

> Measuring is better than guessing: the `wiring_buffer` item (0.70 kg) is a **safety-margin guess**. Weigh the
> finished robot on a scale, subtract 1.84 kg (bare URDF) and the battery + Jetson, and set `wiring_buffer`
> to what's really left. The controller's foot placement is only as good as these numbers.

---

## 5. A printed part or servo changed weight

E.g. you reprinted `FR_Thigh` with more infill and it now weighs 95 g instead of 73.9 g.

1. Open `urdf/QuadURDFcad4_0.urdf`, find `<link name="FR_Thigh">`.
2. Change `<mass value="…"/>` to the new kg value.
3. Scale **all six** `<inertia>` numbers by `new_mass / old_mass` (here 0.095 / 0.0739 = 1.286) — the shape didn't
   change, so inertia grows in proportion. Leave `<origin>` alone unless the weight moved (e.g. a metal insert at one end).
4. Make the **same** edit in `urdf/QuadURDFcad4_0_cadzero.urdf` (the re-zero tool only rotates joint frames, so
   `<inertial>` blocks are identical in both files — keep them in sync for the next re-zero).
5. Do the same for the matching part on the other legs if you reprinted them too (check_robot warns if legs differ > 10 g).
6. Rebuild, `check_robot`, Isaac re-export.

Small helper to scale inertia (prints the new line, you paste it):

```bash
python3 -c "r=0.095/0.0739; print(' '.join(f'{k}=\"{v*r:.6g}\"' for k,v in dict(ixx=1e-5,ixy=0,ixz=0,iyy=2e-5,iyz=0,izz=3e-5).items()))"
```
> **Breakdown**
> - `python3 -c "…"`: run the Python code in quotes.
> - `r=0.095/0.0739`: the scale factor (new mass / old mass).
> - `dict(ixx=…, …)`: **replace these six numbers** with the ones from your `<inertia>` line.
> - It prints `ixx="…" ixy="…" …` scaled by `r`, ready to paste.

---

## 6. New CAD export

Only if the geometry changed (new leg lengths, new body). Order matters:

```bash
cd ~/quadruped_ws/src/QuadPkg && cp ~/Downloads/NewExport.urdf urdf/QuadURDFcad4_0_cadzero.urdf
```
> **Breakdown**
> - `cp SOURCE DEST`: copy the fresh SolidWorks export over the CAD-zero file. Copy its `meshes/*.STL` into `urdf/meshes/` too.
> - Keep link/joint names as in section 1, or the controller won't start.

```bash
python3 tools/rezero_urdf.py urdf/QuadURDFcad4_0_cadzero.urdf urdf/QuadURDFcad4_0.urdf --stand-height 0.155 --y-shift -0.016
```
> **Breakdown**
> - Rewrites the used URDF so all joints = 0 is the standing pose 0.155 m tall, feet shifted 16 mm back under the COM.
> - **Always** feed it the CAD-zero file, never its own output (it would shift twice).
> - Re-measure `--y-shift` with `check_robot` (section 3 "auto stance shift") if the payload changed a lot.

Then: rebuild → `check_robot` (section 1 + 6 must be OK) → stand the robot in the new pose and set every servo's
middle (`servo_tool middle all`, then `offset_ticks` all 0) → `jog` each joint to re-check directions → Isaac re-import.

---

## 7. Weight budget: how much can it carry?

Knee torque grows **linearly** with total mass. From `check_robot` at stand height 0.155 m:

| Total mass | Knee (worst) | % of 2.72 Nm stall | Verdict |
|---|---|---|---|
| 1.84 kg (bare) | 1.05 Nm | 38 % | comfortable |
| 2.8 kg | ≈ 1.6 Nm | ≈ 58 % | OK for short walks |
| 3.84 kg (now) | 2.17 Nm | 80 % | overloaded: heat, sag, weak steps |

≈ **0.057 Nm per extra 100 g** on each knee (~2 % of stall).

What helps, from most to least effective:

| Change | Effect (check_robot numbers) |
|---|---|
| Lighter payload (measure `wiring_buffer`, lighter battery) | −0.057 Nm knee per 100 g |
| 12 V supply instead of 11.1 V (stall ~2.9 Nm) | 80 % → 75 % |
| Stand taller: `stand_height` 0.175 | knee 2.17 → 1.90 Nm (straighter knee = shorter lever). Needs a re-zero (section 6) or the stand pose drifts 6–7° from servo middle |
| Narrower stance: `stance_x_extra: -0.02` | HipRoll 1.41 → 0.96 Nm (52 % → 35 %); slightly less sideways stability |
| ✗ Crouching lower | makes the knee **worse** (0.13 m → 2.44 Nm, 90 %) |

---

## 8. check_robot warnings explained

| Message | Meaning | Fix |
|---|---|---|
| `joint X missing` / `link <LEG>_Foot missing` | URDF names don't match | rename in the URDF (section 1) |
| `axis layout not supported` | a CAD axis isn't along X/Y | fix the joint `<axis>`/frames in CAD, re-export |
| `link X has mass 0` | CAD export forgot a mass | weigh the part or copy from its mirror twin |
| `inertia breaks the triangle rule` | physically impossible inertia | re-export or scale a twin part's inertia |
| `legs differ in mass by N g` | one leg heavier than another | usually a CAD mistake; compare `<mass>` of matching links |
| `COM is N mm to the right/left` | robot is side-heavy | move payload toward x = 0 |
| `COM is N mm ahead/behind … after auto-shift` | auto-centering couldn't fix it | rebuild; make sure `stance_y_shift` isn't set in quad_params |
| `feet must shift N mm` (> 30) | weight far forward/back | move heavy items toward y = 0 or re-zero |
| `COM is high compared to stance width` | top-heavy | mount heavy parts lower, widen stance |
| `'X' centre is outside the body footprint` | xyz probably in mm, or wrong sign | metres; +X right, +Y forward |
| `'X' weighs N kg -- typo?` | > 1.5 kg item | mass in kg, not g |
| `<joint> needs N% of stall torque` | servo overloaded | section 7 |
| `stand pose is N deg away from joint zero` | stand_height/payload changed since the re-zero | re-zero (section 6) or accept that servos aren't at 2048 when standing |
| `sit_height … out of reach` | legs can't fold that far | raise `sit_height` |
| `<joint> at stand is only N deg from its URDF limit` | little room to move | check the URDF `<limit>` |
