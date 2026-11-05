"""halscope-like plot of one capture."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402


def plot_run(r, path, title):
    fig, ax = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    ax[0].plot(r.t, r.cmd, label="motor-pos-cmd")
    ax[0].plot(r.t, r.fb, label="motor-pos-fb", alpha=0.8)
    ax[0].set_ylabel("position mm")
    ax[1].plot(r.t, r.e * 1000.0, color="tab:red", lw=0.8)
    ax[1].set_ylabel("error um")
    ax[2].plot(r.t, r.v, label="vel cmd")
    ax[2].plot(r.t, np.convolve(r.vfb, np.ones(5) / 5, "same"), label="vel fb", alpha=0.8)
    ax[2].plot(r.t, r.out, label="pid output", alpha=0.6)
    ax[2].set_ylabel("mm/s")
    ax[3].plot(r.t, np.convolve(r.a, np.ones(5) / 5, "same"), color="tab:green")
    ax[3].set_ylabel("accel mm/s^2")
    ax[3].set_xlabel("s")
    for a in ax:
        a.grid(alpha=0.3)
    ax[0].legend(loc="upper right", fontsize=8)
    ax[2].legend(loc="upper right", fontsize=8)
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)
