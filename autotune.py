#!/usr/bin/env python3
"""lcnc-autotune: servo tuning for LinuxCNC joints with a velocity-mode PID
(stepgen velocity-cmd or an analog velocity drive).

Run it next to a running LinuxCNC (machine ON, homed, idle), in a shell where
the LinuxCNC environment is set (scripts/rip-environment for run-in-place).

  ./autotune.py --ini CONFIG.ini --joint 0 --window 60 120 check
  ./autotune.py --ini CONFIG.ini --joint 0 --window 60 120 auto
  ./autotune.py ... drive --label "P2-02=80"     compare a drive setting
  ./autotune.py ... jerk                         S-curve jerk scan
  ./autotune.py ... report                       INI proposal, no motion
  ./autotune.py ... restore                      put the original gains back

Phases: check ident drive ff p verify jerk auto report restore
"""

import argparse
import sys

import linuxcnc

from lcnc_autotune import report
from lcnc_autotune.tuner import Abort, Tuner

PHASES = ["check", "ident", "drive", "ff", "p", "verify", "jerk", "auto", "report", "restore"]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("phases", nargs="+", choices=PHASES)
    ap.add_argument("--ini", required=True, help="the INI file LinuxCNC is running with")
    ap.add_argument("--joint", type=int, required=True)
    ap.add_argument("--window", type=float, nargs=2, metavar=("LO", "HI"),
                    help="travel window, machine coordinates of the joint's axis")
    ap.add_argument("--vmax", type=float, help="cap test velocity (mm/s), default INI axis max")
    ap.add_argument("--amax", type=float, help="cap test accel (mm/s^2), default INI axis max")
    ap.add_argument("--ferror-limit", type=float, default=1.0,
                    help="FERROR/MIN_FERROR used while tuning, mm (default 1.0)")
    ap.add_argument("--p-max", type=float, default=5000.0, help="never test P above this")
    ap.add_argument("--iterations", type=int, default=6, help="max ff iterations")
    ap.add_argument("--tol", type=float, default=0.02,
                    help="ff converged when every correction is below this fraction")
    ap.add_argument("--ff3", action="store_true", help="also tune FF3 (jerk feedforward)")
    ap.add_argument("--jerk", type=float, default=0.0,
                    help="run the phase with the S-curve planner at this jerk (mm/s^3)")
    ap.add_argument("--jerks", type=float, nargs="+", help="jerk values for the jerk phase")
    ap.add_argument("--resolution", type=float, default=None,
                    help="feedback resolution mm/count (default 1/|ENCODER_SCALE| or 0.0005)")
    ap.add_argument("--label", help="label for a drive run")
    ap.add_argument("--session-dir", help="where captures and state go (default runs/<machine>-jointN)")
    ap.add_argument("--yes", "-y", action="store_true", help="do not ask before each phase")
    o = ap.parse_args()

    ini = linuxcnc.ini(o.ini)
    if o.resolution is None:
        es = ini.find("JOINT_%d" % o.joint, "ENCODER_SCALE")
        o.resolution = 1.0 / abs(float(es)) if es and float(es) else 0.0005
    t = Tuner(ini, o.ini, o.joint, o)
    try:
        for ph in o.phases:
            print("\n=== %s ===" % ph)
            if ph == "check":
                t.check()
            elif ph == "report":
                report.write(t)
            elif ph == "restore":
                orig = t.state.get("original")
                if not orig:
                    print("  nothing to restore (no session yet)")
                else:
                    t.set_gains(orig)
                    print("  original gains restored: %s" % orig)
            elif ph == "auto":
                t.session(auto, t)
                report.write(t)
            else:
                t.session(getattr(t, "phase_" + ph))
    except Abort as e:
        print("stopped: %s" % e)
        return 1
    return 0


def auto(t):
    t.phase_ident()
    t.phase_ff()
    t.phase_p()
    print("\n  refining feedforward at the new P")
    it = t.o.iterations
    t.o.iterations = max(2, it // 2)
    t.phase_ff()
    t.o.iterations = it
    t.phase_verify()


if __name__ == "__main__":
    sys.exit(main())
