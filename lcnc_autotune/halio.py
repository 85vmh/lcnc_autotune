"""Thin wrappers around halcmd, used for everything HAL: reading and setting
pins, finding the signal a pin is on, loading the sampler. halcmd talks to
the running LinuxCNC's HAL, so nothing here needs a HAL component of its own."""

import subprocess


class HalError(RuntimeError):
    pass


def halcmd(*args, check=True):
    r = subprocess.run(["halcmd", *map(str, args)], capture_output=True, text=True)
    if check and r.returncode != 0:
        raise HalError("halcmd %s: %s" % (" ".join(map(str, args)),
                                          (r.stderr or r.stdout).strip()))
    return r.stdout


def getp(name):
    """Value of a pin or parameter as float (bools are 0.0/1.0)."""
    s = halcmd("-s", "getp", name).strip()
    if s.upper() == "TRUE":
        return 1.0
    if s.upper() == "FALSE":
        return 0.0
    return float(s)


def setp(name, value):
    if isinstance(value, bool):
        value = "1" if value else "0"
    elif isinstance(value, float):
        value = repr(value)
    halcmd("setp", name, value)


def exists(name):
    """True when a pin or parameter called name exists."""
    return subprocess.run(["halcmd", "-s", "getp", name],
                          capture_output=True).returncode == 0


def component_exists(name):
    out = halcmd("-s", "show", "comp", name, check=False)
    return any(len(t) >= 3 and t[2] == name for t in (l.split() for l in out.splitlines()))


def signal_of(pin):
    """Name of the signal pin is linked to, or None."""
    for line in halcmd("-s", "show", "pin", pin).splitlines():
        tok = line.split()
        if pin in tok:
            i = tok.index(pin)
            if len(tok) > i + 2 and tok[i + 1] in ("==>", "<==", "<=>"):
                return tok[i + 2]
    return None


def signal_pins(sig):
    """(writer, [readers]) of a signal, pins as names."""
    writer, readers = None, []
    out = halcmd("-s", "show", "sig", sig)
    tok = out.split()
    for i, t in enumerate(tok):
        if t == "<==" and i + 1 < len(tok):
            writer = tok[i + 1]
        elif t == "==>" and i + 1 < len(tok):
            readers.append(tok[i + 1])
        elif t == "<=>" and i + 1 < len(tok):
            readers.append(tok[i + 1])
    return writer, readers


def pins_matching(pattern):
    """Names of the pins whose name starts with pattern."""
    names = []
    for line in halcmd("-s", "show", "pin", pattern).splitlines():
        tok = line.split()
        if len(tok) >= 5:
            names.append(tok[4])
    return names
