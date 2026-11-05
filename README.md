# lcnc_autotune

Automatic servo tuning for LinuxCNC joints with a velocity-mode PID, either
stepgen `velocity-cmd` (e.g. Mesa 7i95T + Delta ASDA-A2 in step/dir) or an
analog drive in velocity mode (e.g. Mesa 7i97). It automates what you would
otherwise do by hand in halscope:

1. run test moves;
2. capture `motor-pos-cmd`, `motor-pos-fb` and the PID output at the servo rate;
3. analyse the capture;
4. change the PID gains live and repeat until the tuning is good.

At the end it writes the values to put in the INI. **The INI file is never
modified.**

## How it works

1. **ident** identifies the drive from the PID output and the measured
   velocity, as `v = g · lag(output)`. `g` is the scale ratio (1.22 on the
   WeilerE30, from STEP_SCALE 2440 vs encoder 2000) and `tau` is the drive's
   effective lag.
2. **ff** starts from FF1 = 1/g, FF2 = tau/g. It splits the measured following
   error into its velocity, acceleration and jerk parts, taking the P loop and
   the drive lag into account. It then corrects FF1/FF2 (and FF3 with S-curve)
   and repeats until the corrections converge (usually 2–4 iterations).
3. **p** raises P step by step, with FF2/FF3 temporarily off. It measures how
   the axis settles after short moves: settling time, and whether the
   oscillation dies out. It picks the P with the fastest settling that does not
   ring.
4. **verify** runs at full speed and full acceleration and compares against the
   values measured at the start.
5. **jerk** switches the S-curve planner on live (`ini.traj_planner_type`). It
   tries several jerk values and compares error, vibration after the stop and
   move time.
6. **report** writes a table of proposed INI changes: P, FF1/FF2/FF3, corrected
   STEP_SCALE (with the gains scaled to match), FERROR/MIN_FERROR from the
   measured errors, and PLANNER_TYPE/MAX_JERK.

**drive** runs the same moves as ident under a label. Use it before and after
each change in the servo drive (for ASDA-A2: P2-02, P2-00, P1-08, P2-32
auto-tune) to get a comparison table.

## Machine state while tuning

- LinuxCNC running with the real configuration; the GUI can stay open.
- E-stop released, machine ON, all joints homed.
- No program or jogging during a phase. The script switches to MDI by itself and
  returns to Manual at the end. It restores G7/G95/F and the overrides.
- Spindle off, with the travel window (`--window`) clear of chuck and tailstock.
- During tuning, the joint's FERROR/MIN_FERROR are lowered to `--ferror-limit`
  (default 1 mm) through the `ini.N.ferror` pins. If anything goes wrong,
  LinuxCNC stops the machine in realtime, and the script puts the gains back.

## Usage

```sh
source ~/Work/linuxcnc-dev4/scripts/rip-environment
cd ~/Work/lcnc-autotune
INI=~/Work/WeilerE30-StepDir/WeilerE30.ini

# X (joint 0): window 60..120 mm in machine coordinates; gentle first runs
./autotune.py --ini $INI --joint 0 --window 60 120 check
./autotune.py --ini $INI --joint 0 --window 60 120 --vmax 30 --amax 300 ident
./autotune.py --ini $INI --joint 0 --window 60 120 auto        # ident ff p ff verify + report

# servo drive: reference run, change one drive parameter, run again
./autotune.py --ini $INI --joint 0 --window 60 120 drive --label "before"
./autotune.py --ini $INI --joint 0 --window 60 120 drive --label "P2-02=80"

# S-curve after the PID tuning
./autotune.py --ini $INI --joint 0 --window 60 120 jerk
./autotune.py --ini $INI --joint 0 --window 60 120 --jerk 20000 --ff3 ff verify
./autotune.py --ini $INI --joint 0 --window 60 120 report

./autotune.py --ini $INI --joint 0 restore     # original gains back (live)
```

Each phase waits for Enter before moving (`-y` skips the confirmation).
Results go to `runs/<MACHINE>-jointN/`:

- `state.json`: the session; later phases continue from where you left off
- `*.npz`: the captures
- `*.png`: halscope-like plots
- `report.md`: the INI proposal

To start a fresh session, delete or move that directory.

The tuned gains stay live until LinuxCNC is restarted, so you can try the
machine with them. To keep them, copy them from the report into the INI.

## Simulation (no machine needed)

`sim/` holds a simulated lathe. Each joint has a "drive" with gain 1.22 and two
lags, and the same starting PID values as the WeilerE30:

```sh
cd sim && linuxcnc autotune_sim.ini &      # headless
python3 sim/prepare.py                     # E-stop reset, machine on, home
./autotune.py --ini sim/autotune_sim.ini --joint 1 --window 100 300 -y auto
kill $(cat /tmp/autotune-sim.pid)          # stop the simulation
```

In the simulation, Z goes from 0.13 mm max following error to 0.010 mm with the
trapezoidal planner, and to 0.0008 mm with S-curve + FF3.

## Notes for the WeilerE30 lathe

- FF1 = 0.81967 is exactly 2000/2440: STEP_SCALE does not match the real step
  size. The report proposes STEP_SCALE = 2000/4000 and multiplies the gains by
  1.22. Once the scale is corrected, STEPGEN_MAX_ACC is in real mm/s² and must
  be at least 25% above MAX_ACCELERATION. On Z, 2200 vs 2000 is too little.
- `setp hm2_7i95.0.encoder.timer-number 1` in the HAL file samples the encoders
  on the same DPLL timer as the stepgens. The report points it out when the
  encoders are not on that timer.
- FERROR 10/20 and MIN_FERROR 10 are effectively disabled. The report proposes
  values derived from the measured errors.
