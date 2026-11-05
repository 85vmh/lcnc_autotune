"""The tuning phases. Each phase runs test moves on one joint, captures them,
analyses them and stores what it learned in the session (state.json), so the
phases can be run one at a time, days apart, or all together with 'auto'."""

import datetime
import json
import os
import time

import numpy as np

from . import analysis, halio, plots
from .capture import Capture
from .machine import Machine, MachineError

GAINS = ["Pgain", "Igain", "Dgain", "FF0", "FF1", "FF2", "FF3"]
INI_KEY = {"Pgain": "P", "Igain": "I", "Dgain": "D", "FF0": "FF0", "FF1": "FF1",
           "FF2": "FF2", "FF3": "FF3"}


class Abort(RuntimeError):
    pass


def ask(prompt, yes):
    if yes:
        print(prompt + " [auto-yes]")
        return
    a = input(prompt + " [Enter = continue, q = quit] ").strip().lower()
    if a.startswith("q"):
        raise Abort("stopped by operator")


class Tuner:
    def __init__(self, ini, ini_path, joint, opts):
        self.ini = ini
        self.ini_path = ini_path
        self.joint = joint
        self.o = opts
        self.m = Machine(ini, joint)
        self.dt = float(ini.find("EMCMOT", "SERVO_PERIOD") or 1000000) * 1e-9
        self.sec = "JOINT_%d" % joint
        self.pid = self._find_pid()
        self.stepgen = self._find_stepgen()
        machine = (ini.find("EMC", "MACHINE") or "machine").replace(" ", "_")
        self.dir = opts.session_dir or os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "runs", "%s-joint%d" % (machine, joint))
        os.makedirs(self.dir, exist_ok=True)
        self.state_path = os.path.join(self.dir, "state.json")
        self.state = self._load_state()
        vmax = float(ini.find("AXIS_" + self.m.letter, "MAX_VELOCITY") or
                     ini.find(self.sec, "MAX_VELOCITY") or 10)
        amax = float(ini.find("AXIS_" + self.m.letter, "MAX_ACCELERATION") or
                     ini.find(self.sec, "MAX_ACCELERATION") or 100)
        self.vmax = min(vmax, opts.vmax or vmax)
        self.amax = min(amax, opts.amax or amax)

    # ------------------------------------------------------------ discovery
    def _find_pid(self):
        """The pid component whose command is this joint's motor-pos-cmd."""
        sig = halio.signal_of("joint.%d.motor-pos-cmd" % self.joint)
        if sig is None:
            raise halio.HalError("joint.%d.motor-pos-cmd is not connected" % self.joint)
        _, readers = halio.signal_pins(sig)
        for p in readers:
            if p.endswith(".command") and halio.exists(p[:-len("command")] + "Pgain"):
                return p[:-len(".command")]
        raise halio.HalError("no pid reads %s (signal %s)" % (
            "joint.%d.motor-pos-cmd" % self.joint, sig))

    def _find_stepgen(self):
        sig = halio.signal_of(self.pid + ".output")
        if sig is None:
            return None
        _, readers = halio.signal_pins(sig)
        for p in readers:
            if ".stepgen." in p and p.endswith(".velocity-cmd"):
                return p[:-len(".velocity-cmd")]
        return None

    def gains(self):
        return {k: halio.getp("%s.%s" % (self.pid, k)) for k in GAINS}

    def set_gains(self, g):
        for k, v in g.items():
            halio.setp("%s.%s" % (self.pid, k), float(v))

    # ------------------------------------------------------------ state
    def _load_state(self):
        if os.path.exists(self.state_path):
            with open(self.state_path) as f:
                return json.load(f)
        return {"history": []}

    def save(self):
        with open(self.state_path, "w") as f:
            json.dump(self.state, f, indent=1)

    def log(self, phase, **kw):
        kw.update(phase=phase, time=datetime.datetime.now().isoformat(timespec="seconds"))
        self.state["history"].append(kw)
        self.save()

    # ------------------------------------------------------------ checks
    def check(self, verbose=True):
        lo, hi = self.window()
        smin, smax = self.m.soft_limits()
        problems = self.m.problems()
        if lo < smin or hi > smax:
            problems.append("window %.3f..%.3f is outside the soft limits %.3f..%.3f"
                            % (lo, hi, smin, smax))
        if hi - lo < 2.0:
            problems.append("window is shorter than 2 mm")
        if verbose:
            g = self.gains()
            print("joint %d (axis %s), pid %s, %s" % (
                self.joint, self.m.letter, self.pid,
                "stepgen %s" % self.stepgen if self.stepgen else "no stepgen (analog/other)"))
            print("  servo period %.3f ms, test limits %.1f mm/s, %.0f mm/s^2" % (
                self.dt * 1e3, self.vmax, self.amax))
            print("  window %.3f .. %.3f (soft limits %.3f .. %.3f), now at %.3f" % (
                lo, hi, smin, smax, self.m.position()))
            print("  live gains: " + "  ".join("%s=%g" % (k, v) for k, v in g.items()))
            print("  FERROR %s  MIN_FERROR %s (INI)" % (
                self.ini.find(self.sec, "FERROR"), self.ini.find(self.sec, "MIN_FERROR")))
            for p in problems:
                print("  NOT READY: " + p)
            if not problems:
                print("  ready")
        return problems

    def window(self):
        if self.o.window is None:
            raise Abort("--window LO HI is required (machine coordinates of axis %s)"
                        % self.m.letter)
        lo, hi = sorted(self.o.window)
        return lo, hi

    # ------------------------------------------------------------ test moves
    def _sweeps(self, speeds, accel):
        lo, hi = self.window()
        self.m.set_accel(accel)
        time.sleep(0.3)
        self.m.move(lo, self.vmax * 0.3)
        time.sleep(0.25)
        for frac in speeds:
            self.m.move(hi, self.vmax * frac)
            time.sleep(0.25)
            self.m.move(lo, self.vmax * frac)
            time.sleep(0.25)

    def _steps(self, sizes, accel):
        lo, hi = self.window()
        mid = 0.5 * (lo + hi)
        self.m.set_accel(accel)
        time.sleep(0.3)
        self.m.move(mid, self.vmax * 0.3)
        time.sleep(0.3)
        for d in sizes:
            for target in (mid + d, mid, mid - d, mid):
                self.m.move(target, self.vmax)
                time.sleep(0.3)

    def test_moves(self, kind):
        a = self.amax
        if kind == "sweep":       # identification and feedforward
            self._sweeps([0.25, 0.6, 1.0], a)
            self._sweeps([1.0], 0.4 * a)
        elif kind == "fast":      # verify / jerk: full speed and accel
            self._sweeps([1.0, 0.5], a)
            self._steps([0.5, 2.0], a)
        elif kind == "steps":     # P tuning: settling after short moves
            self._steps([0.02, 0.1, 0.5, 2.0], a)
        else:
            raise ValueError(kind)

    def run(self, label, kind):
        """Run one captured test with the live gains; returns (Run, path)."""
        pins = ["joint.%d.motor-pos-cmd" % self.joint, "joint.%d.motor-pos-fb" % self.joint,
                self.pid + ".output"]
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        base = os.path.join(self.dir, "%s-%s" % (stamp, label))
        cap = Capture(pins)
        try:
            cap.open()
            cap.start(base + ".txt")
            self.test_moves(kind)
            time.sleep(0.2)
            d = cap.stop()
        finally:
            cap.close()
        if os.path.exists(base + ".txt"):
            os.remove(base + ".txt")
        r = analysis.Run(d[pins[0]], d[pins[1]], d[pins[2]], self.dt)
        r.save(base + ".npz", gains=json.dumps(self.gains()))
        plots.plot_run(r, base + ".png", "%s  %s" % (label, self._gain_str()))
        return r, base

    def _gain_str(self):
        g = self.gains()
        return " ".join("%s=%.4g" % (k.replace("gain", ""), v) for k, v in g.items()
                        if v or k in ("Pgain", "FF1"))

    # ------------------------------------------------------------ session wrapper
    def session(self, fn, *args):
        """Run a phase with the safety set-up around it; on any failure put the
        gains back to what they were when the phase started."""
        problems = self.check(verbose=True)
        if problems:
            raise Abort("machine not ready")
        if "original" not in self.state:
            self.state["original"] = self.gains()
            self.save()
        start = self.gains()
        ok = False
        try:
            self.m.enter()
            self.m.limit_ferror(self.o.ferror_limit)
            if self.o.jerk:
                self.m.set_scurve(self.o.jerk)
            res = fn(*args)
            ok = True
            return res
        except (KeyboardInterrupt, MachineError, halio.HalError, Abort) as e:
            print("\n  phase stopped: %s" % (e or type(e).__name__))
            self.m.abort()
            raise Abort(str(e) or type(e).__name__)
        finally:
            if not ok:
                self.set_gains(start)
                print("  gains restored to the values at the start of the phase")
            self.m.leave()

    # ============================================================ phases
    def phase_ident(self):
        ask("ident: back-and-forth moves in the window with the current gains", self.o.yes)
        r, base = self.run("ident", "sweep")
        plant = r.identify()
        m = r.metrics()
        self.state["plant"] = plant
        if "baseline" not in self.state:
            self.state["baseline"] = m
        ff = analysis.model_ff(plant)
        p = analysis.model_p(plant)
        self.log("ident", plant=plant, metrics=m, gains=self.gains(), file=base)
        print("  drive model: gain g = %.4f, delay %.2f ms, lag %.2f ms -> tau %.2f ms (fit r2 %.3f)"
              % (plant["g"], plant["delay"] * 1e3, plant["T"] * 1e3, plant["tau"] * 1e3, plant["r2"]))
        print("  model says: FF1 %.4f  FF2 %.5f  FF3 %.3g  P ~%.0f" % (
            ff["FF1"], ff["FF2"], ff["FF3"], p))
        self._print_metrics(m)
        return plant

    def phase_ff(self):
        plant = self._plant()
        use_jerk = bool(self.o.jerk) or self.o.ff3
        g = self.gains()
        ask("ff: tune FF1/FF2%s with repeated back-and-forth moves" % ("/FF3" if use_jerk else ""),
            self.o.yes)
        if not self.state.get("ff_started"):
            model = analysis.model_ff(plant)
            g["FF1"], g["FF2"] = model["FF1"], model["FF2"]
            if use_jerk:
                g["FF3"] = model["FF3"]
            g["FF0"] = 0.0
            self.set_gains(g)
            self.state["ff_started"] = True
        best = None
        damp = 0.8
        for it in range(self.o.iterations):
            r, base = self.run("ff%d" % it, "sweep")
            m = r.metrics()
            P = g["Pgain"]
            reg, corr = r.ff_regression(plant, P, use_jerk)
            print("  iter %d: FF1 %.5f FF2 %.6f FF3 %.3g | max|e| %.4f rms %.4f | fit r2 %.2f" % (
                it, g["FF1"], g["FF2"], g["FF3"], m["max_e"], m["rms_moving"], reg["r2"]))
            self.log("ff", it=it, gains=dict(g), metrics=m, reg=reg, corr=corr, file=base)
            if best is None or m["rms_moving"] < best[0]["rms_moving"]:
                best = (m, dict(g))
            elif m["rms_moving"] > 1.1 * best[0]["rms_moving"]:
                print("  worse than the best so far: going back and taking smaller steps")
                g = dict(best[1])
                damp *= 0.5
            small = all(abs(corr[k]) <= self.o.tol * max(abs(g[k]), 1e-6) +
                        (1e-6 if k != "FF1" else 0)
                        for k in corr)
            if small:
                print("  converged")
                break
            for k, dv in corr.items():
                g[k] = g[k] + damp * dv
            g["FF2"] = max(g["FF2"], 0.0)
            self.set_gains(g)
        self.set_gains(best[1])
        self.state["ff"] = {k: best[1][k] for k in ("FF0", "FF1", "FF2", "FF3")}
        self.save()
        print("  best: FF1 %.5f FF2 %.6f FF3 %.3g, max|e| %.4f mm" % (
            best[1]["FF1"], best[1]["FF2"], best[1]["FF3"], best[0]["max_e"]))
        return best

    def phase_p(self):
        plant = self._plant()
        g = self.gains()
        start = g["Pgain"]
        p_lo = max(start, analysis.model_p(plant, 0.15))
        p_hi = min(analysis.model_p(plant, 1.3), self.o.p_max)
        if p_lo >= p_hi:
            p_lo = p_hi / 4.0
        cands = list(np.geomspace(p_lo, p_hi, max(3, int(np.log(p_hi / p_lo) / np.log(1.25)) + 1)))
        ask("p: settling tests at P = %s" % ", ".join("%.0f" % c for c in cands), self.o.yes)
        results, first_bad = [], None
        base_noise = None
        # with FF2/FF3 on, a well tuned feedforward leaves too little error for
        # the P loop to show how it settles; test with them off, put them back
        ff_saved = {k: g[k] for k in ("FF2", "FF3")}
        g["FF2"] = g["FF3"] = 0.0
        print("  FF2/FF3 off during the P tests (restored afterwards)")
        for P in cands:
            g["Pgain"] = float(P)
            self.set_gains(g)
            r, base = self.run("p%.0f" % P, "steps")
            ring = r.ringing(resolution=self.o.resolution)
            m = r.metrics()
            if base_noise is None:
                base_noise = max(ring["noise"], 1e-6)
            bad = (ring["osc"] >= 4 or ring["overshoot"] > 0.35 or
                   ring["noise"] > 4 * base_noise and ring["noise"] > 2 * self.o.resolution)
            print("  P %7.1f: lobes-1 %d decay %3.0f%% settle %5.1f ms residual %.4f "
                  "still-rms %.5f (%.0f Hz) max|e| %.4f %s" % (
                      P, ring["osc"], 100 * ring["overshoot"], ring["settle"] * 1e3,
                      ring["residual"], ring["noise"], ring["freq"], m["max_e"],
                      "<- ringing" if bad else ""))
            self.log("p", P=float(P), ring=ring, metrics=m, file=base)
            results.append((P, ring, m, bad))
            if bad:
                first_bad = P
                break
        g.update(ff_saved)
        # best damped: shortest settle time without much overshoot
        ok = [x for x in results if not x[3] and x[1]["overshoot"] <= 0.2]
        if ok:
            sel = min(ok, key=lambda x: (round(x[1]["settle"], 4), -x[0]))[0]
            why = "shortest settling, oscillation decay <= 20%"
        else:
            sel = min(analysis.model_p(plant, 0.5), cands[0])
            why = "no clean candidate, model value"
        if first_bad is not None and sel > 0.6 * first_bad:
            sel = 0.6 * first_bad
            why = "0.6 x the first ringing P %.0f" % first_bad
        g["Pgain"] = round(sel, 1)
        self.set_gains(g)
        self.state["P"] = g["Pgain"]
        self.save()
        print("  selected P = %.1f (%s)" % (g["Pgain"], why))
        return g["Pgain"]

    def phase_verify(self, label="verify"):
        ask("%s: full speed / full accel moves and steps" % label, self.o.yes)
        r, base = self.run(label, "fast")
        m = r.metrics()
        ring = r.ringing(resolution=self.o.resolution)
        slow = np.abs(r.v) < 0.1 * r.vmax
        m["max_e_slow"] = r.max_e_where(r.moving & slow)
        self.state.setdefault("verify", {})[label] = {"metrics": m, "ring": ring,
                                                       "gains": self.gains()}
        self.log(label, metrics=m, ring=ring, gains=self.gains(), file=base)
        self._print_metrics(m)
        print("  settling: osc %d overshoot %.0f%% settle %.1f ms" % (
            ring["osc"], 100 * ring["overshoot"], ring["settle"] * 1e3))
        b = self.state.get("baseline")
        if b:
            print("  baseline max|e| %.4f -> now %.4f mm  (x%.1f better)" % (
                b["max_e"], m["max_e"], b["max_e"] / max(m["max_e"], 1e-9)))
        return m

    def phase_drive(self):
        label = "drive-" + (self.o.label or "run%d" % len(
            [h for h in self.state["history"] if h["phase"] == "drive"]))
        ask("drive: same moves as ident, to compare a drive (ASDA) setting change; label '%s'"
            % label, self.o.yes)
        r, base = self.run(label, "sweep")
        m = r.metrics()
        plant = r.identify()
        self.log("drive", label=label, metrics=m, plant=plant, gains=self.gains(), file=base)
        print("  %-24s %9s %9s %9s %8s %7s" % ("label", "max|e|", "rms", "accel|e|", "tau ms", "g"))
        for h in self.state["history"]:
            if h["phase"] in ("drive", "ident"):
                hm, hp = h["metrics"], h["plant"]
                print("  %-24s %9.4f %9.4f %9.4f %8.2f %7.4f" % (
                    h.get("label", "ident"), hm["max_e"], hm["rms_moving"], hm["max_e_accel"],
                    hp["tau"] * 1e3, hp["g"]))
        return m

    def phase_jerk(self):
        jerks = self.o.jerks or [self.amax * f for f in (5, 10, 20, 40, 80, 160)]
        ask("jerk: S-curve at jerk %s mm/s^3" % ", ".join("%.0f" % j for j in jerks), self.o.yes)
        rows = []
        for J in jerks:
            self.m.set_scurve(J)
            r, base = self.run("jerk%.0f" % J, "fast")
            m = r.metrics()
            ring = r.ringing(resolution=self.o.resolution)
            rows.append((J, m, ring))
            self.log("jerk", jerk=J, metrics=m, ring=ring, file=base)
            print("  jerk %8.0f: move time %.3f s  max|e| %.4f  residual after stop %.4f  settle %.1f ms"
                  % (J, m["move_time"], m["max_e"], ring["residual"], ring["settle"] * 1e3))
        best_res = min(x[2]["residual"] for x in rows)
        best_e = min(x[1]["max_e"] for x in rows)
        ok = [x for x in rows if x[2]["residual"] <= 1.5 * best_res + self.o.resolution
              and x[1]["max_e"] <= 1.3 * best_e + self.o.resolution]
        pick = max(ok, key=lambda x: x[0]) if ok else rows[0]
        self.state["jerk"] = pick[0]
        self.save()
        print("  suggested jerk %.0f mm/s^3 (fastest one whose vibration and error stay close to the best)"
              % pick[0])
        return pick[0]

    # ------------------------------------------------------------ helpers
    def _plant(self):
        if "plant" not in self.state:
            print("  no drive model yet: running ident first")
            return self.phase_ident()
        return self.state["plant"]

    def _print_metrics(self, m):
        print("  max|e| %.4f mm (accel %.4f, const v %.4f), rms moving %.4f, still noise %.5f"
              % (m["max_e"], m["max_e_accel"], m["max_e_constv"], m["rms_moving"], m["rms_still"]))
