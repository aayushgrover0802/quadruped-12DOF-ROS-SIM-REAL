"""
lag_math.py -- "how late is each servo?" maths for `lag_meter`. No ROS, unit-tested.

Idea: the measured angle should look like the sent angle, just shifted later in time.
Try every shift L from 0 to max_lag and keep the one where  meas(t)  best matches  sent(t - L).
"""
import numpy as np


def estimate_lag(t_cmd, x_cmd, t_meas, x_meas, max_lag=0.4, step=0.005, min_p2p=0.035):
    """Return dict(lag, err_now, err_at_lag, amp_ratio, ok) for ONE joint.

    t_*: seconds (same clock), x_*: radians. min_p2p: joint must move at least this much (rad, ~2 deg).
    lag        : best time shift [s] (servo is this much late)
    err_now    : RMS |sent - meas| with no shift [rad]   (what servo_monitor shows as err)
    err_at_lag : RMS after shifting by `lag` [rad]       (what is left = sag / amplitude loss)
    amp_ratio  : measured swing size / sent swing size  (1.0 = reaches the full movement)
    """
    t_cmd, x_cmd = np.asarray(t_cmd, float), np.asarray(x_cmd, float)
    t_meas, x_meas = np.asarray(t_meas, float), np.asarray(x_meas, float)
    out = {"lag": float("nan"), "err_now": float("nan"), "err_at_lag": float("nan"),
           "amp_ratio": float("nan"), "ok": False}
    if len(t_cmd) < 20 or len(t_meas) < 20:
        return out
    p2p_cmd = np.ptp(x_cmd)
    if p2p_cmd < min_p2p:
        return out
    # only judge measured samples whose shifted command time is inside the recording
    keep = (t_meas - max_lag >= t_cmd[0]) & (t_meas <= t_cmd[-1])
    tm, xm = t_meas[keep], x_meas[keep]
    if len(tm) < 20:
        return out
    lags = np.arange(0.0, max_lag + 1e-9, step)
    errs = np.array([np.sqrt(np.mean((xm - np.interp(tm - L, t_cmd, x_cmd)) ** 2)) for L in lags])
    k = int(np.argmin(errs))
    out.update(lag=float(lags[k]), err_now=float(errs[0]), err_at_lag=float(errs[k]),
               amp_ratio=float(np.ptp(xm) / p2p_cmd), ok=True)
    return out


TWIN = {"FL": "FR", "FR": "FL", "BL": "BR", "BR": "BL"}     # left/right partner of each leg


def odd_joints(results, min_extra_lag=0.04, max_reach_gap=0.2):
    """Joints that behave clearly worse than their left/right twin -> list of text lines."""
    lines = []
    for j, r in results.items():
        twin = TWIN[j[:2]] + j[2:]
        t = results.get(twin)
        if not r["ok"] or not t or not t["ok"]:
            continue
        if r["lag"] - t["lag"] >= min_extra_lag:
            lines.append(f"{j} is {1000 * (r['lag'] - t['lag']):.0f} ms later than {twin}")
        if t["amp_ratio"] - r["amp_ratio"] >= max_reach_gap:
            lines.append(f"{j} reaches only {100 * r['amp_ratio']:.0f}% of its movement ({twin}: "
                         f"{100 * t['amp_ratio']:.0f}%)")
    return lines


def recommend_gait(results, gait_period, duty_factor, max_fraction=0.2, feedback_latency=0.02):
    """Swing time needed so the worst thigh/knee lag is <= max_fraction of the swing.

    Returns (worst_lag, swing_now, suggested_gait_period) or None if nothing moved.
    A pure, EQUAL delay on every joint would not matter (the controller is open-loop);
    what hurts is that a slow servo cannot finish a short swing, so the step shrinks and
    the foot lands late compared with the other legs. A longer swing fixes that.
    """
    lags = [r["lag"] - feedback_latency for j, r in results.items()
            if r["ok"] and ("ThighP" in j or "KneeP" in j)]
    if not lags:
        return None
    worst = max(0.0, max(lags))
    swing_now = (1.0 - duty_factor) * gait_period
    need_swing = worst / max_fraction
    period = max(gait_period, need_swing / (1.0 - duty_factor))
    return worst, swing_now, round(period + 0.049, 1)       # round UP to 0.1 s
