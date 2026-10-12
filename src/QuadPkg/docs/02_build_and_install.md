# 02 · Build and install

Works the same on your laptop and on the Jetson. Assumes ROS 2 (Humble or newer) is installed.

## One-time system setup

```bash
sudo apt install python3-serial python3-numpy python3-yaml ros-$ROS_DISTRO-diagnostic-msgs
```
> **Breakdown**
> - `sudo`: run as administrator (installing system packages needs it).
> - `apt install`: Ubuntu's package installer.
> - `python3-serial`: pyserial, the library that opens `/dev/ttyUSB0` to talk to the ESP32.
> - `python3-numpy`, `python3-yaml`: maths and YAML-file reading, used by every node.
> - `ros-$ROS_DISTRO-diagnostic-msgs`: the message type used by the new `/debug/servo_*` topics.
>   `$ROS_DISTRO` is filled in automatically (e.g. `humble`) once ROS is sourced. It is usually already installed; apt will just say so.

```bash
sudo usermod -aG dialout $USER
```
> **Breakdown**
> - `usermod`: change a user account. `-aG dialout`: **a**dd to the **G**roup called `dialout`.
> - `dialout` is the group allowed to use serial ports. `$USER` = you.
> - **Log out and back in** (or reboot) once, otherwise you get `Permission denied: '/dev/ttyUSB0'`.

```bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
```
> **Breakdown**
> - `echo "…"`: print the text in quotes.
> - `>> ~/.bashrc`: instead of printing, **append** it to `.bashrc`, the file every new terminal runs at start.
> - Result: every new terminal knows where ROS 2 is. Do this once. (If you typed `$ROS_DISTRO` before sourcing ROS, it's empty — write `humble` directly.)

## Unpacking this workspace

```bash
cd ~ && unzip quadruped_ws.zip
```
> **Breakdown**
> - `cd ~`: go to your home folder.
> - `&&`: run the next command only if this one worked.
> - `unzip quadruped_ws.zip`: extract → you get `~/quadruped_ws/src/QuadPkg/…`.
>   If you already have a `~/quadruped_ws`, rename the old one first (`mv ~/quadruped_ws ~/quadruped_ws_old`) so nothing mixes.

## Build

```bash
cd ~/quadruped_ws && colcon build --packages-select QuadPkg && source install/setup.bash
```
> **Breakdown**
> - `cd ~/quadruped_ws`: the workspace folder, the one that **contains** `src/`. Never build inside `src/`.
> - `colcon build`: ROS 2's build tool. For a Python package it copies code, URDF, meshes, configs and docs into `install/`.
> - `--packages-select QuadPkg`: build only this package.
> - `source install/setup.bash`: tell **this terminal** about the freshly built package. Needed in **every new terminal**.
>
> Rebuild after **any** change to a `.yaml`, `.urdf`, launch file or Python file. Nodes read the **installed** copy, not `src/`.

Optional: build with `--symlink-install` so Python edits take effect without rebuilding:

```bash
colcon build --packages-select QuadPkg --symlink-install
```
> **Breakdown**
> - `--symlink-install`: instead of copying files into `install/`, make shortcuts (symlinks) to `src/`.
>   Python and YAML edits then apply on the next `ros2 run` without a rebuild.
>   Adding a **new** file or a new `ros2 run` name still needs a rebuild.

## Clean rebuild

Do this once after unpacking this version (install paths changed), or whenever things act strangely.

```bash
cd ~/quadruped_ws && rm -rf build install log && colcon build --packages-select QuadPkg && source install/setup.bash
```
> **Breakdown**
> - `rm -rf build install log`: delete the three generated folders. **r**ecursive (whole folder), **f**orce (no questions).
>   Only ever run this inside `~/quadruped_ws` — never in `src/`, which holds your code.
> - The rest: a normal build, starting from nothing.

## Check the build worked

```bash
ros2 pkg executables QuadPkg
```
> **Breakdown**
> - `ros2 pkg executables`: list every program in a package that `ros2 run` can start.
> - Expect: `check_robot controller diagnose export_urdf gait hardware_bridge servo_monitor servo_tool teleop`.
>   If `servo_monitor` is missing, you are running an old build → rebuild and `source install/setup.bash`.

```bash
cd ~/quadruped_ws/src/QuadPkg && python3 -m pytest test/test_kinematics_and_gait.py test/test_real_robot.py test/test_debug_tools.py -v
```
> **Breakdown**
> - `python3 -m pytest`: run the automated tests. The three files: the project's own tests. `-v`: print each test's name.
> - They need no ROS and no robot: IK/FK, gait, zero = stand, payload, USB frame format, ticks maths, debug table, `check_robot`.
> - The other files in `test/` (`test_flake8`, `test_pep257`, `test_copyright`) are ROS code-style checks that need
>   the `ament_*` packages; `colcon test --packages-select QuadPkg` runs them.

## Two machines (Jetson on the robot, laptop for teleop/debug)

```bash
export ROS_DOMAIN_ID=7
```
> **Breakdown**
> - `export`: set a setting (environment variable) for this terminal and the programs it starts.
> - `ROS_DOMAIN_ID=7`: ROS 2 machines only see each other if this number matches (0–101). Pick any number,
>   use the **same** on Jetson and laptop, and a **different** one for Isaac Sim experiments, so the sim's
>   `/joint_states` never reaches the real robot. Add the line to `~/.bashrc` to make it permanent.
