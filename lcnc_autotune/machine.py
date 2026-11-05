"""Talking to the running LinuxCNC: preconditions, MDI moves, modal state,
and the temporary safety/limit changes made through the ini.* HAL pins."""

import time

import linuxcnc

from . import halio


class MachineError(RuntimeError):
    pass


def axis_letters(ini):
    """Joint index -> axis letter, from the kinematics coordinates (trivkins)."""
    kins = ini.find("KINS", "KINEMATICS") or ""
    coords = None
    for tok in kins.split():
        if tok.startswith("coordinates="):
            coords = tok.split("=", 1)[1]
    if coords is None:
        coords = (ini.find("TRAJ", "COORDINATES") or "XYZ").replace(" ", "")
    return [c.upper() for c in coords]


class Machine:
    def __init__(self, ini, joint):
        self.ini = ini
        self.joint = joint
        self.letter = axis_letters(ini)[joint]
        self.s = linuxcnc.stat()
        self.c = linuxcnc.command()
        self.e = linuxcnc.error_channel()
        self.saved_pins = {}
        self.saved_modal = None

    # ---- state ----
    def poll(self):
        self.s.poll()
        err = self.e.poll()
        if err:
            kind, text = err
            if kind in (linuxcnc.NML_ERROR, linuxcnc.OPERATOR_ERROR):
                raise MachineError("LinuxCNC error: %s" % text)
            print("  LinuxCNC: %s" % text)
        return self.s

    def problems(self):
        """What keeps the machine from running tuning moves (empty = ready)."""
        s = self.poll()
        p = []
        if s.task_state != linuxcnc.STATE_ON:
            p.append("machine is not ON (E-stop released + machine on)")
        if not all(s.homed[j] for j in range(s.joints)):
            p.append("not all joints are homed")
        if s.interp_state != linuxcnc.INTERP_IDLE:
            p.append("a program or MDI command is running")
        if s.task_mode == linuxcnc.MODE_AUTO and s.interp_state != linuxcnc.INTERP_IDLE:
            p.append("machine is in AUTO mode with a program active")
        if s.spindle[0]["enabled"]:
            p.append("spindle is on")
        return p

    def soft_limits(self):
        s = self.poll()
        j = s.joint[self.joint]
        return j["min_position_limit"], j["max_position_limit"]

    def position(self):
        return self.poll().joint[self.joint]["output"]

    # ---- modal state the moves need, restored afterwards ----
    def enter(self):
        s = self.poll()
        self.saved_modal = (tuple(s.gcodes), s.settings[1], s.feedrate, s.rapidrate)
        if s.feedrate != 1.0:
            self.c.feedrate(1.0)
        if s.rapidrate != 1.0:
            self.c.rapidrate(1.0)
        self._mode(linuxcnc.MODE_MDI)
        self.mdi("G8 G21 G90 G94")

    def leave(self):
        try:
            if self.saved_modal is not None:
                gcodes, feed, fo, ro = self.saved_modal
                words = []
                if 70 in gcodes:
                    words.append("G7")
                if 200 in gcodes:
                    words.append("G20")
                if 910 in gcodes:
                    words.append("G91")
                if 950 in gcodes:
                    words.append("G95")
                elif 930 in gcodes:
                    words.append("G93")
                words.append("F%g" % feed)
                s = self.poll()
                if s.task_state == linuxcnc.STATE_ON:
                    self._mode(linuxcnc.MODE_MDI)
                    self.mdi(" ".join(words))
                self.c.feedrate(fo)
                self.c.rapidrate(ro)
                self.saved_modal = None
        finally:
            self.restore_pins()
            try:
                self._mode(linuxcnc.MODE_MANUAL)
            except MachineError:
                pass

    def _mode(self, mode):
        if self.poll().task_mode == mode:
            return
        self.c.mode(mode)
        self.c.wait_complete(5)
        if self.poll().task_mode != mode:
            raise MachineError("could not change LinuxCNC mode")

    # ---- moves ----
    def mdi(self, line, timeout=120):
        s = self.poll()
        if s.task_state != linuxcnc.STATE_ON:
            raise MachineError("machine is no longer ON (following error or E-stop?)")
        self.c.mdi(line)
        self.c.wait_complete(5)
        t0 = time.time()
        while True:
            time.sleep(0.005)
            s = self.poll()
            if s.task_state != linuxcnc.STATE_ON:
                raise MachineError("machine turned off during '%s'" % line)
            if s.interp_state == linuxcnc.INTERP_IDLE and s.queue == 0 \
                    and s.state != linuxcnc.RCS_EXEC:
                return
            if time.time() - t0 > timeout:
                self.c.abort()
                raise MachineError("timeout on '%s'" % line)

    def move(self, pos, feed_mm_s):
        self.mdi("G53 G1 %s%.5f F%.3f" % (self.letter, pos, feed_mm_s * 60.0))

    def abort(self):
        try:
            self.c.abort()
        except Exception:
            pass

    # ---- temporary ini.* pin changes ----
    def set_pin(self, name, value):
        if name not in self.saved_pins:
            self.saved_pins[name] = halio.getp(name)
        halio.setp(name, value)

    def restore_pins(self):
        for name, value in reversed(list(self.saved_pins.items())):
            try:
                halio.setp(name, int(value) if name.endswith("planner_type") else value)
            except halio.HalError as e:
                print("  could not restore %s: %s" % (name, e))
        self.saved_pins = {}

    def axis_pin(self, what):
        return "ini.%s.%s" % (self.letter.lower(), what)

    def joint_pin(self, what):
        return "ini.%d.%s" % (self.joint, what)

    def limit_ferror(self, limit):
        """Tighten the joint's following error limits for the duration of the
        tuning: LinuxCNC itself then stops the machine in realtime."""
        for what in ("ferror", "min_ferror"):
            pin = self.joint_pin(what)
            cur = halio.getp(pin)
            if cur > limit:
                self.set_pin(pin, float(limit))

    def set_accel(self, accel):
        self.set_pin(self.axis_pin("max_acceleration"), float(accel))

    def set_scurve(self, jerk):
        """jerk > 0: S-curve planner with that jerk on traj, axis and joint;
        jerk == 0: trapezoidal planner."""
        if jerk > 0:
            for pin in ("ini.traj_max_jerk", self.axis_pin("max_jerk"),
                        self.joint_pin("max_jerk")):
                self.set_pin(pin, float(jerk))
            self.set_pin("ini.traj_planner_type", 1)
        else:
            self.set_pin("ini.traj_planner_type", 0)
        time.sleep(0.3)     # task applies ini.* pin changes on its cycle
