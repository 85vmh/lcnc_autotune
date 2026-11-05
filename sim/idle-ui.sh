#!/bin/sh
# Headless "display" for the autotune sim: LinuxCNC runs until this exits.
# Stop the sim with:  kill $(cat /tmp/autotune-sim.pid)   or Ctrl-C in its terminal
echo $$ > /tmp/autotune-sim.pid
exec sleep infinity
