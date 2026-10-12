# 11 · Why isn't it walking?

Open these three terminals first; almost every answer below comes from them.

```bash
ros2 launch QuadPkg robot.launch.py        # T1 (sim: ros2 launch QuadPkg sim.launch.py)
```
```bash
ros2 run QuadPkg servo_monitor             # T2: per servo, sent vs. measured
```
```bash
ros2 run QuadPkg teleop                    # T3: keyboard
```
> **Breakdown**
> - T1 runs the robot and prints warnings ([09](09_warnings_reference.md)).
> - T2 header shows **controller state**, **motors enabled**, battery and how old each topic is; the table shows each servo.
> - T3 drives. Remember: teleop must keep running — if it's closed, `/cmd_vel` stops and the robot stops after 0.5 s.

Then find your symptom:

| # | Symptom | Jump to |
|---|---|---|
| A | Nothing moves at all | [A](#a-nothing-moves-at-all) |
| B | Stands, but W does nothing (state never `WALK`) | [B](#b-it-stands-but-never-goes-into-walk) |
| C | State is `WALK`, legs move, body doesn't go anywhere | [C](#c-legs-step-but-it-walks-on-the-spot) |
| D | Falls over / tips sideways while stepping | [D](#d-tips-over-or-falls-while-trotting) |
| E | Walks, but drifts, turns, or one leg looks wrong | [E](#e-walks-but-crooked-or-one-leg-is-odd) |
| F | Shakes, jitters, buzzes | [F](#f-shaking-jitter-buzzing) |
| G | Walks for a while, then sags / stops | [G](#g-works-for-a-while-then-gets-weak-or-stops) |
| S | Isaac Sim only problems | [06 · Sim-specific checks](06_isaac_sim.md#sim-specific-checks) |

---

## A. Nothing moves at all

1. **Is the bridge running?** T1 must show `all 12 servos answered` and `hardware_bridge ready`.
   Crash at start → [09 · hardware_bridge](09_warnings_reference.md#hardware_bridge-real-robot).
2. **Motors enabled?** T2 header `motors enabled: False` and `!! NOT SENDING: motors OFF` → press **M** in teleop.
   - T1 says `refusing to enable: not all servos are answering` → find the `OFF` row in T2 → [10 §4](10_diagnose_parts.md#4-servo-bus-and-ids).
3. **Is the controller publishing?** T2 `!! NOT SENDING: no /joint_command received` → `quad_controller` crashed (scroll up in T1) or isn't running.
   ```bash
   ros2 topic hz /joint_command
   ```
   > **Breakdown**
   > - Must show ~50 Hz. Nothing → controller not running / stuck.
4. **Sent but the servos don't move?** `ticks>` changing, `<ticks` frozen → flags tell you: `TQ0` (torque off, power reset), load 100 % (blocked), `LOWV`.
5. Still stuck → [10](10_diagnose_parts.md) from the top.

## B. It stands but never goes into WALK

The controller only starts trotting when **all** of these are true (see `gait/controller.py`, step 3):

| Condition | How to check | Fix |
|---|---|---|
| mode is `stand` (not `sit`) | T2 state `STAND`, not `SIT` | press **T** |
| sit→stand transition finished | state `STAND` (not changing height) | wait `height_transition_time` (2.5 s) |
| startup finished | state not `STARTUP` | wait `startup_time` (3 s). Stuck in STARTUP in sim = no `/clock` |
| a velocity command above `cmd_deadband` (0.02) | `ros2 topic echo /cmd_vel` while pressing W | W adds 0.05 m/s → ok. If you set `lin_step` < 0.02 one press isn't enough |
| `/cmd_vel` keeps arriving (< `cmd_timeout` 0.5 s apart) | `ros2 topic hz /cmd_vel` ≈ 10 Hz | teleop window must stay open; on two machines check `ROS_DOMAIN_ID` matches |

```bash
ros2 topic echo /quad/state
```
> **Breakdown**
> - Shows the state the controller is in, 50×/s. Press W in teleop and watch for `WALK`.

```bash
ros2 topic info /cmd_vel
```
> **Breakdown**
> - `Publisher count` must be ≥ 1 (teleop) and `Subscription count` ≥ 1 (controller). 0 subscribers = controller on another domain/machine not visible.

If state goes `WALK` but legs don't move → the bridge isn't sending ([A](#a-nothing-moves-at-all) step 2–4).

## C. Legs step but it walks on the spot

Most common real-robot problem. In order of likelihood:

1. **Servos can't keep up / too weak.** T2: `err°` on knees > 5° during swing, `ERR` flags, `load%` near ±100,
   knees sag. `check_robot` says knees need ~80 % of stall → **the robot is too heavy for the gait**
   ([08 §7](08_urdf_and_weight.md#7-weight-budget-how-much-can-it-carry)). Try, one at a time:
   - slower gait: `gait_period: 1.0`, `max_vx: 0.06`;
   - remove payload for a test: `ros2 launch QuadPkg robot.launch.py payload_file:=/dev/null` **and** physically take it off;
   - charged battery / 12 V.
2. **Foot doesn't lift.** Watch one leg: in swing the foot should rise `step_height` (2 cm). If it scuffs → `step_height: 0.025–0.03`
   (costs torque), or knees sagging (point 1).
3. **Slipping feet.** Smooth floor + hard feet → rubber feet or test on a mat.
4. **Steps too small.** At `max_vx 0.10` the step is ≈ 55 mm; at 0.05 m/s ≈ 28 mm — sagging servos can eat that.
   Press W twice (0.10 m/s) before judging.
5. **Slew limiter too tight.** T2 `SLEW` flag constantly during walking → `ros2 param set /hardware_bridge max_velocity 8.0` (default).
   (`SLEW` is normal only in the first `enable_ramp_time` seconds after M.)
6. **Clamped commands.** T1 `foot targets were out of reach` or T2 `LIM` → `step_height`/`stand_height` too big for the leg.

## D. Tips over or falls while trotting

| Clue | Cause | Fix |
|---|---|---|
| Always tips to the **same side** | COM not over the centreline, or a HipRoll `direction` wrong | `check_robot` COM section; `servo_tool jog <LEG>_HipRoll 10` on all four |
| Tips forward/back | payload mass/position in `payload.yaml` doesn't match reality → feet centred under the wrong COM | weigh the robot, fix `payload.yaml` ([08 §3](08_urdf_and_weight.md#3-adding-weight)) |
| Rocks diagonally each step | normal a bit for a trot; too much = too slow a swing or too high COM | `gait_period` 0.85 → 0.7 (faster swing) **only if servos keep up**; `duty_factor` 0.65–0.7; mount weight lower |
| Falls when stopping/starting | accelerations too high | `max_lin_accel: 0.2` |
| One leg collapses when it lands | that knee too weak / hot / wrong offset | T2: that row's `err°`, `°C`, `load%` |

## E. Walks, but crooked or one leg is odd

| Clue | Cause | Fix |
|---|---|---|
| Curves although only W pressed | unequal legs: offsets wrong, one leg slipping, one servo weaker | `servo_tool read` in the stand pose: all angles ≈ 0? Re-`calib`. Compare twins' `err°` while walking |
| One leg moves **backwards** relative to the others | that leg's `ThighP` direction is wrong | `servo_tool jog <LEG>_ThighP 10` |
| One leg's foot goes up when it should go down | `KneeP` direction wrong | `jog <LEG>_KneeP 10` |
| Robot stands crooked (one corner lower) | offsets | `servo_tool limp` → pose → `servo_tool calib --write …` |
| One leg sits a few mm higher/lower | horn one spline off, or offset | [10 §6](10_diagnose_parts.md#6-horns-linkages-legs) |

## F. Shaking, jitter, buzzing

| Clue | Cause | Fix |
|---|---|---|
| Buzzing while standing still, `load%` jumping | servo hunting; voltage noise | charged battery; `servo_acc: 50` (smoother acceleration on the servo); check frame stiffness |
| Whole body shakes in rhythm | gait too fast for the servos | `gait_period: 1.0` |
| Jerks every few seconds | USB hiccups / dropped `/joint_command` | `ros2 topic hz /joint_states` steady? CPU load on the Jetson (`htop`) |
| Twitch on M | normal glide uses `enable_velocity`; a **jump** means feedback ≠ reality | check `meas°` vs. how the legs really look before pressing M |

## G. Works for a while, then gets weak or stops

| Clue in T1/T2 | Cause | Fix |
|---|---|---|
| `HOT` flags, temperature rising on knees | overloaded servos | rest, less weight, slower gait ([08 §7](08_urdf_and_weight.md#7-weight-budget-how-much-can-it-carry)) |
| `LOW BATTERY`, `LOWV` | battery empty / sagging | charge |
| `[esp32] … ready` mid-run, `TQ0`, `feedback is N s old` | ESP32 brown-out reset | power wiring, separate regulator for the ESP32, bigger battery C-rating |
| `servos not answering` mid-run | loose connector shaken out, servo overload protection | wiggle test ([10 §4](10_diagnose_parts.md#4-servo-bus-and-ids)) |

---

## Collect evidence before changing things

```bash
ros2 bag record -o problem_01 /cmd_vel /quad/state /joint_command /debug/joint_sent /joint_states /debug/servo_command /debug/servo_feedback /quad/battery_voltage /quad/servo_temperature
```
> **Breakdown**
> - Records one failed attempt so you can look at it calmly afterwards (PlotJuggler, or replay with the robot off).
>   Change **one** setting at a time and record again — then you know which change helped.

```bash
ros2 run QuadPkg servo_monitor --once > problem_01.txt && ros2 run QuadPkg check_robot >> problem_01.txt
```
> **Breakdown**
> - `>` writes the servo table into a file, `>>` **appends** the model check to the same file. One file with
>   the full picture — easy to share when asking for help.
