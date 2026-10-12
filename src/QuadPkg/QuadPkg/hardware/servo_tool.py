"""
servo_tool.py -- setup & calibration helper for the real robot (no ROS needed).

    ros2 run QuadPkg servo_tool <command> [options]

Commands
  scan                  list every servo ID that answers on the bus
  read                  table of all 12 joints: ticks, angle, temperature, voltage
  limp                  torque OFF on all 12 (move the legs by hand)
  hold                  torque ON on all 12, holding where they are now
  stand                 glide all joints to 0 rad (= standing pose) in 3 s
  jog JOINT DEG         move ONE joint by DEG degrees and back -> check `direction`
  step JOINT DEG        jump ONE joint by DEG at once, time how fast the servo gets there
                        [--speed N --acc N to test the servo's speed/acceleration registers]
  calib [--write FILE]  robot held in the standing pose -> measure offset_ticks
  middle ID|all         make the CURRENT position the servo's new middle (2048),
                        same as the web app button (stored inside the servo)
  setid OLD NEW         change a servo's ID (stored inside the servo)

Options
  --config FILE   hardware.yaml to use (default: the installed one)
  --port PORT     override the serial port from the yaml

IMPORTANT: stop the robot launch first -- only one program can use the serial port.
"""
import argparse
import os
import re
import sys
import time

import numpy as np

from QuadPkg.hardware import protocol as P
from QuadPkg.hardware.esp32_link import Esp32Link
from QuadPkg.hardware.servo_map import ServoMap, load_hardware_yaml

MOTION_HINT = {
    "HipRoll": "foot swings toward the robot's RIGHT side (same for all four legs)",
    "ThighP": "whole leg swings FORWARD",
    "KneeP": "lower leg swings forward/up: the knee BENDS MORE, foot comes closer to the body",
}


def default_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("QuadPkg"), "config", "hardware.yaml")
    except Exception:
        return os.path.join(os.path.dirname(__file__), "..", "..", "config", "hardware.yaml")


class Tool:
    def __init__(self, cfg_path, port=None):
        self.cfg_path = cfg_path
        self.p = load_hardware_yaml(cfg_path)
        self.map = ServoMap.from_params(self.p)
        port = port or self.p.get("port", "/dev/ttyUSB0")
        baud = int(self.p.get("baud", 921600))
        print(f"config: {cfg_path}\nport:   {port} @ {baud}")
        self.link = Esp32Link(port, baud, log=print)
        hello = self.link.hello(timeout=4.0)
        if hello is None:
            sys.exit("ESP32 does not answer. Is the firmware flashed? Right port? Robot launch stopped?")
        print(f"ESP32 firmware v{hello[1]}")

    def configure(self):
        online = self.link.configure(self.map.ids, 50, 0, 0)   # watchdog off while tooling
        if online is None:
            sys.exit("ESP32 did not acknowledge CONFIG")
        bad = [f"{j}(id {i})" for j, i, ok in zip(self.map.order, self.map.ids, online) if not ok]
        if bad:
            print("WARNING, not answering: " + ", ".join(bad))
        return online

    def state(self, n_avg=1):
        """Average n STATE frames -> (ticks array, servo list of last frame)."""
        acc, last = [], None
        for _ in range(n_avg):
            st = self.link.wait_state(timeout=1.0)
            if st is None:
                sys.exit("no feedback from the ESP32")
            last = st[1]
            acc.append([s.pos for s in last])
        return np.mean(acc, axis=0), last

    def glide(self, start_q, end_q, seconds):
        """Stream a smooth joint-space move at 50 Hz."""
        n = max(1, int(seconds * 50))
        for k in range(1, n + 1):
            s = k / n
            s = s * s * (3 - 2 * s)
            q = (1 - s) * start_q + s * end_q
            self.link.set_targets(self.map.joint_to_ticks(q).tolist())
            time.sleep(0.02)

    # ------------------------------------------------------------------ commands
    def cmd_scan(self, a):
        print("scanning IDs 1..253 (takes ~1 s)...")
        ids = self.link.scan(1, 253)
        if ids is None:
            sys.exit("scan failed (no reply)")
        print(f"found {len(ids)} servo(s): {ids}")
        expected = set(self.map.ids)
        if set(ids) != expected:
            print(f"  expected (hardware.yaml): {sorted(expected)}")
            print(f"  missing: {sorted(expected - set(ids))}   unexpected: {sorted(set(ids) - expected)}")
        else:
            print("  matches hardware.yaml")

    def cmd_read(self, a):
        online = self.configure()
        ticks, servos = self.state(3)
        q = self.map.ticks_to_joint(ticks)
        print(f"\n{'joint':12s} {'id':>3s} {'ticks':>6s} {'angle':>8s} {'temp':>5s} {'volt':>5s} {'load%':>6s} torque")
        for i, j in enumerate(self.map.order):
            s = servos[i]
            if not online[i] and not s.online:
                print(f"{j:12s} {self.map.ids[i]:3d}   --- not answering ---")
                continue
            reg = self.link.reg_read(self.map.ids[i], P.REG_TORQUE_ENABLE, 1)
            torque = "?" if reg is None else ("on" if reg[0] == 1 else "off")
            print(f"{j:12s} {self.map.ids[i]:3d} {ticks[i]:6.0f} {np.degrees(q[i]):+7.1f}d "
                  f"{s.temp:4d}C {s.voltage:4.1f}V {s.load / 10:+6.1f} {torque}")
        print("\nangle 0 = standing pose (servo middle 2048 + offset_ticks).")

    def cmd_limp(self, a):
        self.configure()
        self.link.torque(False)
        print("torque OFF on all 12 -- legs are free to move by hand")

    def cmd_hold(self, a):
        self.configure()
        self.state()
        self.link.torque(True)
        print("torque ON, holding the current pose")

    def cmd_stand(self, a):
        self.configure()
        ticks, _ = self.state(2)
        self.link.torque(True)
        start = self.map.ticks_to_joint(ticks)
        print("gliding to the standing pose (all joints 0) in 3 s... hold the robot!")
        self.glide(start, np.zeros(len(start)), 3.0)
        print("done (torque stays ON)")

    def cmd_jog(self, a):
        if a.joint not in self.map.order:
            sys.exit(f"unknown joint {a.joint}; choose from {self.map.order}")
        if abs(a.deg) > 30:
            sys.exit("keep jogs within +-30 degrees")
        i = self.map.index(a.joint)
        self.configure()
        ticks, _ = self.state(2)
        self.link.torque(True)
        start = self.map.ticks_to_joint(ticks)
        end = start.copy()
        end[i] += np.radians(a.deg)
        kind = a.joint.split("_")[1]
        sign = "POSITIVE" if a.deg > 0 else "NEGATIVE"
        hint = MOTION_HINT[kind] if a.deg > 0 else "the OPPOSITE of: " + MOTION_HINT[kind]
        print(f"{a.joint} (id {self.map.ids[i]}) {sign} {abs(a.deg):.0f} deg. EXPECTED: {hint}")
        self.glide(start, end, 1.0)
        time.sleep(1.0)
        self.glide(end, start, 1.0)
        print(f"If it moved the WRONG way, flip direction[{i}] for {a.joint} in hardware.yaml "
              f"(currently {self.map.direction[i]:+.0f}).")

    def cmd_step(self, a):
        """Instant step on one joint, record its response from the 50 Hz feedback."""
        if a.joint not in self.map.order:
            sys.exit(f"unknown joint {a.joint}; choose from {self.map.order}")
        if not 2 <= abs(a.deg) <= 20:
            sys.exit("use a step between 2 and 20 degrees")
        i = self.map.index(a.joint)
        self.configure()
        ticks, _ = self.state(2)
        start = self.map.ticks_to_joint(ticks)
        end = start.copy()
        end[i] += np.radians(a.deg)
        t_start, t_end = self.map.joint_to_ticks(start).tolist(), self.map.joint_to_ticks(end).tolist()
        self.link.torque(True)
        print(f"{a.joint} (id {self.map.ids[i]}): step {a.deg:+.0f} deg, speed={a.speed} acc={a.acc}. "
              "Robot LIFTED, legs free!")
        for _ in range(25):                                   # 0.5 s: hold where it is
            self.link.set_targets(t_start, a.speed, a.acc)
            time.sleep(0.02)
        q0 = self.map.ticks_to_joint([s.pos for s in self.link.state[1]])[i]
        samples, t0 = [], time.monotonic()
        last = self.link.n_state
        while time.monotonic() - t0 < 0.8:                    # 0.8 s: the step
            self.link.set_targets(t_end, a.speed, a.acc)
            st = self.link.wait_state(timeout=0.05, newer_than=last)
            if st is not None:
                last = self.link.n_state
                samples.append((time.monotonic() - t0, self.map.ticks_to_joint([s.pos for s in st[1]])[i]))
        self.glide(end, start, 1.0)                           # back where it was
        if len(samples) < 10:
            sys.exit("too little feedback during the step")
        target = q0 + np.radians(a.deg)
        frac = [(t, (q - q0) / (target - q0)) for t, q in samples]

        def first(level):
            return next((t for t, f in frac if f >= level), None)

        def ms(t):
            return "never" if t is None else f"{t * 1000:.0f} ms"
        t10, t50, t90 = first(0.1), first(0.5), first(0.9)
        final = np.degrees(samples[-1][1] - q0)
        print(f"  starts moving (10%): {ms(t10)}   half way (50%): {ms(t50)}   "
              f"almost there (90%): {ms(t90)}   ended at {final:+.1f} of {a.deg:+.0f} deg")
        print("  (includes ~10-30 ms of USB/feedback delay)")
        if t90 is None:
            print("  -> never got within 90%: blocked, too weak, or speed/acc registers limiting it")
        elif t90 < 0.12:
            print("  -> servo is FAST. If lag_meter shows much more while walking, the extra lag comes from "
                  "load (weight) or voltage sag, not the servo itself.")
        else:
            print("  -> servo is SLOW even without load: try --acc 0 / --speed 0 vs other values, "
                  "check voltage, compare with the same joint on another leg.")

    def cmd_calib(self, a):
        self.configure()
        print("Hold the robot in the STANDING pose (all joints 0, like in Isaac at start).")
        print("Tip: run `servo_tool limp` first, pose the legs by hand, then run calib.")
        input("Press ENTER when the robot is in the standing pose... ")
        ticks, _ = self.state(10)
        offsets = [int(round(t - P.CENTER_TICK)) for t in ticks]
        for j, o in zip(self.map.order, offsets):
            flag = "   <-- large! wrong ID or pose?" if abs(o) > 200 else ""
            print(f"  {j:12s} offset {o:+5d} ticks ({o * 360 / 4096:+6.1f} deg){flag}")
        line = "offset_ticks: [" + ",  ".join(", ".join(str(o) for o in offsets[k:k + 3])
                                              for k in range(0, len(offsets), 3)) + "]"
        print("\n    " + line)
        if a.write:
            text = open(a.write, encoding="utf-8").read()
            new, n = re.subn(r"offset_ticks:\s*\[[^\]]*\]", line, text)
            if n != 1:
                sys.exit(f"could not find a single `offset_ticks: [...]` line in {a.write}")
            open(a.write, "w", encoding="utf-8").write(new)
            print(f"written to {a.write}. Now:  colcon build --packages-select QuadPkg")
        else:
            print("(not saved -- add  --write ~/<your_ws>/src/QuadPkg/config/hardware.yaml  to save)")

    def cmd_middle(self, a):
        ids = self.map.ids if a.target == "all" else [int(a.target)]
        print(f"This makes the CURRENT position of servo(s) {ids} their new middle (2048).")
        print("The robot must be in the STANDING pose. Torque will be switched off.")
        if input("type yes to continue: ").strip().lower() != "yes":
            return
        for sid in ids:
            ok = self.link.reg_write(sid, P.REG_TORQUE_ENABLE, [128])
            print(f"  id {sid}: {'ok' if ok else 'NO REPLY'}")
        print("done. Set offset_ticks to 0 for these joints in hardware.yaml.")

    def cmd_setid(self, a):
        if not (1 <= a.new <= 253):
            sys.exit("new ID must be 1..253")
        if self.link.reg_read(a.new, P.REG_ID, 1) is not None:
            sys.exit(f"ID {a.new} is already used by another servo on the bus")
        if self.link.reg_read(a.old, P.REG_ID, 1) is None:
            sys.exit(f"no servo with ID {a.old}")
        self.link.reg_write(a.old, P.REG_LOCK, [0])          # unlock EEPROM
        self.link.reg_write(a.old, P.REG_ID, [a.new])        # (servo answers with its OLD id)
        time.sleep(0.05)
        self.link.reg_write(a.new, P.REG_LOCK, [1])          # lock again
        ok = self.link.reg_read(a.new, P.REG_ID, 1)
        print(f"ID {a.old} -> {a.new}: {'ok' if ok and ok[0] == a.new else 'FAILED'}")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="servo_tool", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=default_config())
    ap.add_argument("--port", default=None)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("scan", "read", "limp", "hold", "stand"):
        sub.add_parser(name)
    j = sub.add_parser("jog")
    j.add_argument("joint")
    j.add_argument("deg", type=float)
    st = sub.add_parser("step")
    st.add_argument("joint")
    st.add_argument("deg", type=float)
    st.add_argument("--speed", type=int, default=0, help="goal speed register (0 = max)")
    st.add_argument("--acc", type=int, default=0, help="acceleration register (0 = max)")
    c = sub.add_parser("calib")
    c.add_argument("--write", default=None)
    m = sub.add_parser("middle")
    m.add_argument("target")
    s = sub.add_parser("setid")
    s.add_argument("old", type=int)
    s.add_argument("new", type=int)
    # ros2 run passes --ros-args ...; ignore anything after it
    argv = sys.argv[1:] if argv is None else argv
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    a = ap.parse_args(argv)

    tool = Tool(a.config, a.port)
    try:
        getattr(tool, "cmd_" + a.cmd)(a)
    finally:
        tool.link.close()


if __name__ == "__main__":
    main()
