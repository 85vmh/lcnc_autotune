"""Analysis of one capture: plant identification, following-error regression,
metrics and ringing detection. Pure numpy/scipy, no LinuxCNC needed, so it
can be re-run on saved captures (.npz)."""

import numpy as np
from scipy.signal import lfilter


def ma(x, n):
    """Centered moving average over n samples (n odd keeps it centered)."""
    if n <= 1:
        return x
    k = np.ones(n) / n
    return np.convolve(x, k, mode="same")


def deriv(x, dt):
    """Backward difference like pid's command derivative; first sample 0."""
    d = np.zeros_like(x)
    d[1:] = np.diff(x) / dt
    return d


class Run:
    """One capture with its derived signals."""

    def __init__(self, cmd, fb, out, dt):
        self.dt = dt
        self.cmd = np.asarray(cmd, float)
        self.fb = np.asarray(fb, float)
        self.out = np.asarray(out, float)
        self.t = np.arange(len(self.cmd)) * dt
        self.e = self.cmd - self.fb
        self.v = deriv(self.cmd, dt)
        self.a = deriv(self.v, dt)
        self.j = ma(deriv(self.a, dt), 5)
        self.vfb = deriv(self.fb, dt)
        vmax = np.max(np.abs(self.v)) or 1.0
        amax = np.max(np.abs(self.a)) or 1.0
        self.vmax, self.amax = vmax, amax
        self.moving = ma((np.abs(self.v) > 1e-6).astype(float), 41) > 0
        self.accel = np.abs(ma(self.a, 5)) > 0.05 * amax
        # standstill: commanded velocity zero for >= 60 ms on both sides
        still = ma((np.abs(self.v) < 1e-9).astype(float), int(0.12 / dt) | 1) > 0.999
        self.still = still

    @classmethod
    def load(cls, path):
        d = np.load(path)
        return cls(d["cmd"], d["fb"], d["out"], float(d["dt"]))

    def save(self, path, **extra):
        np.savez_compressed(path, cmd=self.cmd, fb=self.fb, out=self.out,
                            dt=self.dt, **extra)

    # ------------------------------------------------------------------
    def metrics(self):
        e = self.e
        mv = self.moving
        constv = mv & ~self.accel & (np.abs(self.v) > 0.05 * self.vmax)
        m = {
            "max_e": float(np.max(np.abs(e[mv]))) if mv.any() else 0.0,
            "rms_moving": float(np.sqrt(np.mean(e[mv] ** 2))) if mv.any() else 0.0,
            "max_e_accel": float(np.max(np.abs(e[self.accel]))) if self.accel.any() else 0.0,
            "max_e_constv": float(np.max(np.abs(e[constv]))) if constv.any() else 0.0,
            "rms_still": float(np.std(e[self.still])) if self.still.any() else 0.0,
            "vmax": float(self.vmax),
            "amax": float(self.amax),
            "move_time": float(np.sum(np.abs(self.v) > 1e-6) * self.dt),
        }
        return m

    def max_e_where(self, mask):
        return float(np.max(np.abs(self.e[mask]))) if mask.any() else 0.0

    # ------------------------------------------------------------------
    def identify(self, max_delay=0.03, smooth=5):
        """Fit the drive as  v_fb = g * lag(delay(out), T)  (velocity in, the
        measured velocity out). Returns g, delay, T and tau = delay + T, the
        effective lag the feedforward has to make up for."""
        dt = self.dt
        y = ma(self.vfb, smooth)
        x = ma(self.out, smooth)
        use = ma(self.moving.astype(float), 201) > 0
        best = None
        Ts = np.concatenate(([0.0], np.geomspace(0.1e-3, 30e-3, 40)))
        nd = int(max_delay / dt)
        for T in Ts:
            if T > 0:
                a = 1.0 - np.exp(-dt / T)
                xf = lfilter([a], [1.0, a - 1.0], x)
            else:
                xf = x
            for d in range(0, nd + 1):
                xs = np.zeros_like(xf)
                xs[d:] = xf[:len(xf) - d] if d else xf
                xu, yu = xs[use], y[use]
                den = np.dot(xu, xu)
                if den <= 0:
                    continue
                g = np.dot(xu, yu) / den
                r = yu - g * xu
                sse = float(np.dot(r, r))
                if best is None or sse < best[0]:
                    best = (sse, g, d, T)
        sse, g, d, T = best
        yu = y[use]
        r2 = 1.0 - sse / max(float(np.dot(yu - yu.mean(), yu - yu.mean())), 1e-30)
        return {"g": float(g), "delay": d * dt, "T": float(T),
                "tau": d * dt + float(T), "r2": float(r2)}

    # ------------------------------------------------------------------
    def ff_regression(self, plant, P, use_jerk=False):
        """Feedforward corrections from the measured following error.

        The error is e = H * deficit, where deficit is the velocity the
        feedforward failed to deliver and H is the closed loop seen from
        there (integrator, P gain and the identified drive lag). The deficit
        is modelled as c_v v + c_a a (+ c_j j); regressing e on H v, H a, H j
        gives c_*, and the FF changes that cancel them follow from inverting
        the drive (gain g, lag tau): dFF1 = c_v/g, dFF2 = (c_a + tau c_v)/g,
        dFF3 = (c_j + tau c_a)/g."""
        g, tau = plant["g"], plant["tau"]
        regs = [("v", self.v), ("a", ma(self.a, 3))]
        if use_jerk:
            regs.append(("j", ma(self.j, 5)))
        cols = [closed_loop(r, plant, P, self.dt) for _, r in regs]
        A = np.stack([np.ones_like(self.e)] + cols, axis=1)
        sc = np.max(np.abs(A), axis=0)
        sc[sc == 0] = 1.0
        coef, *_ = np.linalg.lstsq(A / sc, self.e, rcond=None)
        coef = coef / sc
        c = {name: float(coef[i + 1]) for i, (name, _) in enumerate(regs)}
        fit = A @ coef
        r2 = 1.0 - float(np.sum((self.e - fit) ** 2)) / max(
            float(np.sum((self.e - self.e.mean()) ** 2)), 1e-30)
        corr = {"FF1": c["v"] / g, "FF2": (c["a"] + tau * c["v"]) / g}
        if use_jerk:
            corr["FF3"] = (c["j"] + tau * c["a"]) / g
        return {"c": c, "r2": r2}, corr

    # ------------------------------------------------------------------
    def stops(self, min_still=0.15):
        """Indices where the commanded motion ends and is followed by at least
        min_still seconds of standstill."""
        mv = np.abs(self.v) > 1e-9
        idx = []
        n = len(mv)
        w = int(min_still / self.dt)
        for i in np.nonzero(mv[:-1] & ~mv[1:])[0] + 1:
            if i + w < n and not mv[i:i + w].any():
                idx.append(int(i))
        return idx

    def ringing(self, window=0.25, resolution=0.0):
        """Settling after each stop: number of sign changes ('osc'), the decay
        ratio of the oscillation lobes after the first one ('overshoot',
        > 0.3 means lightly damped) and settle time. resolution is the feedback resolution (mm/count)."""
        n_w = int(window / self.dt)
        res = {"stops": 0, "osc": 0, "overshoot": 0.0, "settle": 0.0,
               "residual": 0.0, "freq": 0.0, "noise": 0.0}
        stops = self.stops(window + 0.02)
        if not stops:
            return res
        tails = [self.e[i + n_w // 2:i + n_w] for i in stops]
        noise = float(np.median([np.std(t) for t in tails]))
        tol = max(3.0 * noise, 2.0 * resolution, 1e-5)
        res["noise"] = noise
        res["stops"] = len(stops)
        for i in stops:
            seg = self.e[i:i + n_w]
            final = float(np.median(seg[-n_w // 4:]))
            d = seg - final
            # split the free response after the stop into same-sign lobes
            # (only samples above tol count) and take each lobe's peak
            lobes, cur_sign, cur_peak = [], 0, 0.0
            for x in d:
                if abs(x) <= tol:
                    continue
                sg = 1 if x > 0 else -1
                if sg != cur_sign:
                    if cur_sign:
                        lobes.append(cur_peak)
                    cur_sign, cur_peak = sg, 0.0
                cur_peak = max(cur_peak, abs(x))
            if cur_sign:
                lobes.append(cur_peak)
            # one sign change can be the plain shape of a well damped response
            # (the deficit before the stop is a doublet); ringing shows as a
            # third lobe that has not died out
            decay = 0.0
            for k in range(1, len(lobes) - 1):
                decay = max(decay, lobes[k + 1] / lobes[k])
            peak = max(lobes) if lobes else 0.0
            outside = np.nonzero(np.abs(d) > max(tol, 0.05 * peak))[0]
            settle = (outside[-1] + 1) * self.dt if len(outside) else 0.0
            resid = float(np.max(np.abs(d[int(0.02 / self.dt):])))
            res["osc"] = max(res["osc"], max(0, len(lobes) - 1))
            res["overshoot"] = max(res["overshoot"], decay)
            res["settle"] = max(res["settle"], settle)
            res["residual"] = max(res["residual"], resid)
        # dominant frequency of the error during standstill (sustained hunting)
        if self.still.sum() > 256:
            es = self.e[self.still] - np.mean(self.e[self.still])
            spec = np.abs(np.fft.rfft(es * np.hanning(len(es))))
            fr = np.fft.rfftfreq(len(es), self.dt)
            band = fr > 5
            if band.any():
                res["freq"] = float(fr[band][np.argmax(spec[band])])
        return res


def closed_loop(u, plant, P, dt):
    """Error response of the loop to a velocity deficit u: the discrete
    integrator e' = u - v_fb, v_fb = g * lowpass(delay(P e)) - the same model
    identify() fits."""
    g, T = plant["g"], plant["T"]
    d = int(round(plant["delay"] / dt))
    alpha = 1.0 - np.exp(-dt / T) if T > 0 else 1.0
    n = len(u)
    e = np.zeros(n)
    hist = np.zeros(d + 1)
    y = 0.0
    ek = 0.0
    gp = g * P
    for k in range(n):
        e[k] = ek
        hist = np.roll(hist, 1)
        hist[0] = ek
        y += alpha * (hist[d] - y)
        ek = ek + dt * (u[k] - gp * y)
    return e


def model_ff(plant):
    """Feedforward that inverts the identified drive: 1/g, tau/g and the
    second order term of (1 + sT) e^(s delay)."""
    g, T, d = plant["g"], plant["T"], plant["delay"]
    return {"FF1": 1.0 / g, "FF2": (T + d) / g, "FF3": (T * d + d * d / 2.0) / g}


def model_p(plant, k=0.5):
    """P for a crossover of k / tau (k = 0.5 -> ~60 deg phase margin with a
    pure delay tau: margin = 90 deg - k rad)."""
    return k / (plant["g"] * max(plant["tau"], 1e-4))
