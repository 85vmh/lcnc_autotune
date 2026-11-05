#!/usr/bin/env python3
"""Sim only: E-stop reset, machine on, home all joints."""
import time
import linuxcnc
s, c = linuxcnc.stat(), linuxcnc.command()
c.state(linuxcnc.STATE_ESTOP_RESET); c.wait_complete()
c.state(linuxcnc.STATE_ON); c.wait_complete()
c.mode(linuxcnc.MODE_MANUAL); c.wait_complete()
c.teleop_enable(0); c.wait_complete()
c.home(-1); c.wait_complete()
for _ in range(100):
    s.poll()
    if all(s.homed[j] for j in range(s.joints)):
        break
    time.sleep(0.1)
print("on:", s.task_state == linuxcnc.STATE_ON, "homed:", s.homed[:s.joints])
