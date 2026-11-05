"""Servo-rate capture of HAL pins with the stock sampler + halsampler.

The sampler realtime component is loaded into the running LinuxCNC, added at
the end of servo-thread (so it sees the values of the cycle that just ran) and
wired to the requested pins. halsampler streams its FIFO to a file. Everything
is removed again in close(), also when the tuning run fails."""

import os
import signal
import subprocess
import time

import numpy as np

from . import halio

SIG_PREFIX = "autotune-"


class Capture:
    def __init__(self, pins, thread="servo-thread", depth=8000):
        self.pins = list(pins)
        self.thread = thread
        self.depth = depth
        self.made_signals = []
        self.loaded = False
        self.proc = None
        self.outfile = None

    # ---- setup / teardown ----
    def open(self):
        if halio.component_exists("sampler"):
            raise halio.HalError(
                "a 'sampler' component is already loaded; autotune needs its own")
        for p in self.pins:
            if not halio.exists(p):
                raise halio.HalError("pin %s does not exist" % p)
        cfg = "f" * len(self.pins)
        halio.halcmd("loadrt", "sampler", "cfg=" + cfg, "depth=%d" % self.depth)
        self.loaded = True
        halio.setp("sampler.0.enable", False)
        halio.halcmd("addf", "sampler.0", self.thread)
        for i, p in enumerate(self.pins):
            sp = "sampler.0.pin.%d" % i
            sig = halio.signal_of(p)
            if sig is None:
                sig = SIG_PREFIX + p.replace(".", "-")
                halio.halcmd("net", sig, p, sp)
                self.made_signals.append(sig)
            else:
                halio.halcmd("net", sig, sp)
        return self

    def close(self):
        self._kill()
        if not self.loaded:
            return
        for i in range(len(self.pins)):
            halio.halcmd("unlinkp", "sampler.0.pin.%d" % i, check=False)
        for sig in self.made_signals:
            # the source pin is still on it; unlink everything, then delete
            halio.halcmd("delsig", sig, check=False)
        halio.halcmd("delf", "sampler.0", self.thread, check=False)
        halio.halcmd("unloadrt", "sampler", check=False)
        self.loaded = False
        self.made_signals = []

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # ---- recording ----
    def start(self, outfile):
        self.outfile = outfile
        with open(outfile, "w") as f:
            pass
        self.proc = subprocess.Popen(["halsampler", "-c", "0", "-t", outfile],
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.PIPE)
        time.sleep(0.2)          # let halsampler attach before samples flow
        if self.proc.poll() is not None:
            raise halio.HalError("halsampler exited: %s" %
                                 self.proc.stderr.read().decode().strip())
        halio.setp("sampler.0.overruns", 0)
        halio.setp("sampler.0.enable", True)

    def stop(self):
        """Stop recording; returns a dict pin -> numpy array, plus 'n'."""
        halio.setp("sampler.0.enable", False)
        t0 = time.time()
        while halio.getp("sampler.0.curr-depth") > 0 and time.time() - t0 < 5:
            time.sleep(0.05)
        time.sleep(0.2)
        overruns = int(halio.getp("sampler.0.overruns"))
        self._kill()
        data = np.loadtxt(self.outfile, ndmin=2)
        if data.shape[0] < 10:
            raise halio.HalError("capture has only %d samples" % data.shape[0])
        tags = data[:, 0]
        gaps = int(np.sum(np.diff(tags) != 1))
        if overruns or gaps:
            print("  warning: capture lost samples (overruns %d, gaps %d)" %
                  (overruns, gaps))
        out = {p: data[:, i + 1] for i, p in enumerate(self.pins)}
        out["n"] = data.shape[0]
        return out

    def _kill(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None
