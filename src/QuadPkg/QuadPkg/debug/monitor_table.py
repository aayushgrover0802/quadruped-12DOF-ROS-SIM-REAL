"""
monitor_table.py -- the text table used by `servo_monitor`. No ROS imports, so it is unit-tested
in test/test_debug_tools.py.
"""
import math

ORDER = ["FL_HipRoll", "FL_ThighP", "FL_KneeP", "FR_HipRoll", "FR_ThighP", "FR_KneeP",
         "BL_HipRoll", "BL_ThighP", "BL_KneeP", "BR_HipRoll", "BR_ThighP", "BR_KneeP"]


def _status_dict(msg):
    """DiagnosticArray -> {joint: {key: value, '_msg': message, '_hw': hardware_id}}"""
    out = {}
    for st in msg.status:
        d = {kv.key: kv.value for kv in st.values}
        d["_msg"], d["_hw"], d["_level"] = st.message, st.hardware_id, st.level
        out[st.name] = d
    return out


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _f(v, width, fmt="{:+.1f}"):
    v = _num(v)
    return ("-" if v is None else fmt.format(v)).rjust(width)


def build_table(view, ctrl, cmd, fb, meas, header_lines=(), err_warn=5.0, max_temp=65, min_volt=10.5):
    """Pure function (no ROS) so it can be unit-tested. Returns the table as text."""
    rows = list(header_lines)
    joints = ORDER if not ctrl else [j for j in ORDER if j in ctrl] or list(ctrl)
    show_cmd, show_fb = view in ("both", "cmd"), view in ("both", "fb")
    head = f"{'joint':11s} {'id':>3s}"
    if show_cmd:
        head += f" {'ctrl°':>7s} {'sent°':>7s} {'ticks>':>6s}"
    if show_fb:
        head += f" {'<ticks':>6s} {'meas°':>7s}"
    if show_cmd and show_fb:
        head += f" {'err°':>6s}"
    if show_fb:
        head += f" {'load%':>6s} {'°C':>3s} {'V':>5s}"
    head += "  flags"
    rows += [head, "-" * len(head)]
    for j in joints:
        c, f = cmd.get(j, {}), fb.get(j, {})
        sid = (c.get("_hw") or f.get("_hw") or "").replace("servo id ", "")
        ctrl_deg = math.degrees(ctrl[j]) if j in ctrl else None
        meas_deg = _num(f.get("angle_deg"))
        if meas_deg is None and j in meas:                 # Isaac: no bridge, use /joint_states
            meas_deg = math.degrees(meas[j])
        sent_deg = _num(c.get("sent_deg"))
        err = None
        if sent_deg is not None and meas_deg is not None:
            err = sent_deg - meas_deg
        elif ctrl_deg is not None and meas_deg is not None and not cmd:
            err = ctrl_deg - meas_deg                        # sim: compare with the controller directly
        flags = []
        if f and f.get("online") == "False":
            flags.append("OFF")
        if f and f.get("torque_on") == "False":
            flags.append("TQ0")
        if c.get("clamped_by_limit") == "True":
            flags.append("LIM")
        if c.get("slew_limited") == "True":
            flags.append("SLEW")
        if (_num(f.get("temp_c")) or 0) > max_temp:
            flags.append("HOT")
        if f and (_num(f.get("voltage_v")) or 99) < min_volt:
            flags.append("LOWV")
        if err is not None and abs(err) > err_warn:
            flags.append("ERR")
        if f and f.get("online") == "False":          # stale numbers from a dead servo: hide them
            f = {k: v for k, v in f.items() if k in ("online", "_hw", "_msg", "_level")}
            meas_deg, err = None, None
        line = f"{j:11s} {sid:>3s}"
        if show_cmd:
            line += f" {_f(ctrl_deg, 7)} {_f(sent_deg, 7)} {_f(c.get('sent_ticks'), 6, '{:.0f}')}"
        if show_fb:
            line += f" {_f(f.get('ticks'), 6, '{:.0f}')} {_f(meas_deg, 7)}"
        if show_cmd and show_fb:
            line += f" {_f(err, 6)}"
        if show_fb:
            line += (f" {_f(f.get('load_pct'), 6, '{:+.0f}')} {_f(f.get('temp_c'), 3, '{:.0f}')}"
                     f" {_f(f.get('voltage_v'), 5, '{:.1f}')}")
        line += "  " + " ".join(flags)
        rows.append(line)
    return "\n".join(rows)
