"""INI proposal and report.md from a session. Nothing is written to the INI."""

import math
import os

from . import halio

KEYS = [("Pgain", "P"), ("Igain", "I"), ("Dgain", "D"), ("FF0", "FF0"),
        ("FF1", "FF1"), ("FF2", "FF2"), ("FF3", "FF3")]


def nice(x, sig=4):
    """x to sig significant digits, without exponent unless it is tiny."""
    if x == 0:
        return "0"
    r = float("%.*g" % (sig, x))
    if abs(r) < 1e-4:
        return "%.*g" % (sig, r)
    return ("%f" % r).rstrip("0").rstrip(".")


def ceil_sig(x, sig=2):
    if x <= 0:
        return 0.0
    p = 10 ** (math.floor(math.log10(x)) - sig + 1)
    return math.ceil(x / p) * p


def proposal(t):
    """List of (section, key, ini value, proposed, note) for joint t.joint."""
    ini, sec, st = t.ini, t.sec, t.state
    live = t.gains()
    rows = []

    def cur(s, k):
        v = ini.find(s, k)
        return v if v is not None else "-"

    plant = st.get("plant")
    g = plant["g"] if plant else 1.0
    fix_scale = bool(t.stepgen and plant and abs(g - 1.0) > 0.01 and
                     ini.find(sec, "STEP_SCALE") is not None)
    k = g if fix_scale else 1.0
    new_ss, ss_note = None, ""
    if fix_scale:
        ss = float(ini.find(sec, "STEP_SCALE"))
        new_ss = ss / g
        es = ini.find(sec, "ENCODER_SCALE")
        if es and abs(abs(new_ss) / abs(float(es)) - 1.0) < 0.005:
            # within 0.5% of 1 step = 1 encoder count: that is the real ratio
            new_ss = abs(float(es)) * (1 if ss > 0 else -1)
            ss_note = " (rounded to |ENCODER_SCALE|, measured %.5g)" % (ss / g)
        k = ss / new_ss
    for pin, key in KEYS:
        new = live[pin] * k
        old = cur(sec, key)
        try:
            same = abs(float(old) - new) <= 1e-9 * max(1.0, abs(new))
        except ValueError:
            same = False
        if not same and not (old == "-" and new == 0):
            rows.append((sec, key, old, nice(new, 5), ""))
    if fix_scale:
        rows.append((sec, "STEP_SCALE", ini.find(sec, "STEP_SCALE"), nice(new_ss, 6),
                     "measured: the drive moves %.4fx what stepgen thinks%s; the gains above "
                     "are scaled by %.5f to match" % (g, ss_note, k)))
        acc = float(ini.find(sec, "STEPGEN_MAX_ACC") or 0)
        amax = float(ini.find(sec, "MAX_ACCELERATION") or 0)
        if acc and amax and acc < 1.25 * amax:
            rows.append((sec, "STEPGEN_MAX_ACC", ini.find(sec, "STEPGEN_MAX_ACC"),
                         nice(ceil_sig(1.25 * amax), 3),
                         "with the corrected scale this is in real mm/s^2; keep >= 25% above "
                         "MAX_ACCELERATION so the PID corrections are not clipped"))
    v = st.get("verify", {}).get("verify")
    if v:
        m = v["metrics"]
        fe = max(ceil_sig(3 * m["max_e"]), 0.1)
        mfe = min(max(ceil_sig(3 * m.get("max_e_slow", 0)), 0.02), fe)
        rows.append((sec, "FERROR", cur(sec, "FERROR"), nice(fe, 2),
                     "3x the %.4f mm measured at full speed" % m["max_e"]))
        rows.append((sec, "MIN_FERROR", cur(sec, "MIN_FERROR"), nice(mfe, 2),
                     "3x the %.4f mm measured at low speed" % m.get("max_e_slow", 0)))
    J = st.get("jerk")
    if J:
        ax = "AXIS_" + t.m.letter
        rows.append(("TRAJ", "PLANNER_TYPE", cur("TRAJ", "PLANNER_TYPE"), "1", "S-curve"))
        rows.append(("TRAJ", "MAX_LINEAR_JERK", cur("TRAJ", "MAX_LINEAR_JERK"), nice(J, 3),
                     "the lowest jerk of all axes"))
        rows.append((ax, "MAX_JERK", cur(ax, "MAX_JERK"), nice(J, 3), ""))
        rows.append((sec, "MAX_JERK", cur(sec, "MAX_JERK"), nice(J, 3), ""))
    return rows, fix_scale


def notes(t):
    out = []
    for p in halio.pins_matching("hm2_"):
        if p.endswith(".encoder.timer-number"):
            if halio.getp(p) < 0:
                out.append("`%s` is -1: the encoders are read when hm2 read runs, not on the "
                           "DPLL timer the stepgens use. `setp %s 1` in the HAL file makes the "
                           "position samples evenly spaced (less velocity noise)." % (p, p))
    return out


def write(t):
    rows, fix_scale = proposal(t)
    st = t.state
    lines = ["# Autotune %s, joint %d (axis %s)" % (
        t.ini.find("EMC", "MACHINE"), t.joint, t.m.letter), ""]
    plant = st.get("plant")
    if plant:
        lines += ["Drive model: gain %.4f, delay %.2f ms, lag %.2f ms, tau %.2f ms (fit r2 %.3f)."
                  % (plant["g"], plant["delay"] * 1e3, plant["T"] * 1e3, plant["tau"] * 1e3,
                     plant["r2"]), ""]
    b = st.get("baseline")
    v = st.get("verify", {}).get("verify")
    if b:
        lines.append("| | max error | in accel | constant v | rms moving |")
        lines.append("|---|---|---|---|---|")
        lines.append("| before (original gains) | %.4f | %.4f | %.4f | %.4f |" % (
            b["max_e"], b["max_e_accel"], b["max_e_constv"], b["rms_moving"]))
        if v:
            m = v["metrics"]
            lines.append("| after (verify) | %.4f | %.4f | %.4f | %.4f |" % (
                m["max_e"], m["max_e_accel"], m["max_e_constv"], m["rms_moving"]))
        lines += ["", "mm. Before = the ident run (window and speeds may differ from verify).", ""]
    lines += ["## Proposed INI changes", "",
              "| section | key | INI now | proposed | note |", "|---|---|---|---|---|"]
    for r in rows:
        lines.append("| [%s] | %s | %s | %s | %s |" % r)
    if fix_scale:
        lines += ["", "If you keep STEP_SCALE as it is, use the live gains instead "
                  "(they are what was tested): " +
                  ", ".join("%s=%s" % (k, nice(t.gains()[p], 5)) for p, k in KEYS)]
    n = notes(t)
    if n:
        lines += ["", "## Notes", ""] + ["- " + x for x in n]
    lines += ["", "## History", "", "| time | phase | max error | rms | detail |",
              "|---|---|---|---|---|"]
    for h in st["history"]:
        m = h.get("metrics", {})
        det = ""
        if h["phase"] == "ff":
            gg = h["gains"]
            det = "FF1 %.5f FF2 %.6f FF3 %.3g" % (gg["FF1"], gg["FF2"], gg["FF3"])
        elif h["phase"] == "p":
            det = "P %.0f, osc %d, overshoot %.0f%%" % (h["P"], h["ring"]["osc"],
                                                          100 * h["ring"]["overshoot"])
        elif h["phase"] == "jerk":
            det = "jerk %.0f, move time %.3f s" % (h["jerk"], m["move_time"])
        elif "label" in h:
            det = h["label"]
        lines.append("| %s | %s | %.4f | %.4f | %s |" % (
            h["time"], h["phase"], m.get("max_e", 0), m.get("rms_moving", 0), det))
    path = os.path.join(t.dir, "report.md")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n  Proposed INI changes (not applied):")
    for r in rows:
        print("    [%s] %-16s %-10s -> %-10s %s" % r)
    for x in n:
        print("  note: " + x.replace("`", ""))
    print("  report: %s" % path)
    return path
