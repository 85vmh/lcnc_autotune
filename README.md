# lcnc-autotune

Unealtă care face servo tuning automat pentru joint-urile LinuxCNC cu PID în
mod viteză: stepgen `velocity-cmd` (7i95T + ASDA-A2) sau driver analogic în
mod viteză (7i97). Face ce faci tu de mână în halscope: pornește mișcări de test,
înregistrează la rata servo `motor-pos-cmd`, `motor-pos-fb` și ieșirea PID,
analizează datele, schimbă live parametrii PID și repetă până găsește parametri buni.
La final scrie ce ar trebui pus în INI. **INI-ul nu este modificat.**

## Cum funcționează

1. **ident**: din ieșirea PID și viteza măsurată identifică driverul ca
   `v = g · lag(ieșire)`. `g` este raportul de scară (1.22 pe strung, din
   STEP_SCALE 2440 față de encoder 2000), iar `tau` este întârzierea efectivă a driverului.
2. **ff**: pornește de la FF1 = 1/g, FF2 = tau/g. Separă eroarea măsurată în
   componenta de viteză, de accelerație și de jerk, ținând cont de bucla P și de
   lag. Corectează FF1/FF2 (și FF3 cu S-curve) și repetă până converge (2–4 iterații).
3. **p**: crește P pas cu pas, cu FF2/FF3 oprite temporar. Măsoară cum se
   oprește axa după mișcări scurte: timpul de așezare și dacă oscilația se stinge.
   Alege P-ul cu așezarea cea mai rapidă care nu oscilează.
4. **verify**: viteză și accelerație maxime, apoi compară cu valorile de la început.
5. **jerk**: activează live planner-ul S-curve (`ini.traj_planner_type`) la mai
   multe valori de jerk și compară eroarea, vibrația după oprire și timpul de mișcare.
6. **report**: tabel cu propunerile pentru INI: P, FF1/FF2/FF3, STEP_SCALE
   corectat (cu gain-urile scalate corespunzător), FERROR/MIN_FERROR din erorile
   măsurate, PLANNER_TYPE/MAX_JERK.

**drive** rulează aceleași mișcări ca ident, cu o etichetă. O folosești înainte și
după fiecare schimbare în ASDA (P2-02, P2-00, P1-08, auto-tune P2-32) și primești
un tabel comparativ.

## Starea mașinii

- LinuxCNC pornit cu configul real, cu TeachInLathe deschis.
- E-stop eliberat, mașina ON, toate axele homed.
- Niciun program sau jog în timpul unei faze. Scriptul trece singur în MDI și la final
  revine în Manual. Restaurează G7/G95/F și override-urile.
- Arborele oprit, fereastra de cursă (`--window`) departe de mandrină și de pinolă.
- Pe durata tuning-ului, FERROR/MIN_FERROR ale joint-ului sunt coborâte la
  `--ferror-limit` (implicit 1 mm), prin pinii `ini.N.ferror`. Dacă ceva o ia
  razna, LinuxCNC oprește mașina în realtime. Scriptul pune gain-urile înapoi.

## Folosire

```sh
source ~/Work/linuxcnc-dev4/scripts/rip-environment
cd ~/Work/lcnc-autotune
INI=~/Work/WeilerE30-StepDir/WeilerE30.ini

# X (joint 0): fereastră 60..120 mm în coordonate mașină; primele rulări mai blând
./autotune.py --ini $INI --joint 0 --window 60 120 check
./autotune.py --ini $INI --joint 0 --window 60 120 --vmax 30 --amax 300 ident
./autotune.py --ini $INI --joint 0 --window 60 120 auto        # ident ff p ff verify + report

# ASDA: rulare de referință, schimbi un parametru în driver, rulezi din nou
./autotune.py --ini $INI --joint 0 --window 60 120 drive --label "inainte"
./autotune.py --ini $INI --joint 0 --window 60 120 drive --label "P2-02=80"

# S-curve după PID
./autotune.py --ini $INI --joint 0 --window 60 120 jerk
./autotune.py --ini $INI --joint 0 --window 60 120 --jerk 20000 --ff3 ff verify
./autotune.py --ini $INI --joint 0 --window 60 120 report

./autotune.py --ini $INI --joint 0 restore     # gain-urile originale înapoi (live)
```

Fiecare fază cere Enter înainte de mișcare (`-y` sare peste confirmare). Rezultatele
ajung în `runs/<MACHINE>-jointN/`: `state.json` (sesiunea; fazele continuă de unde
ai rămas), `*.npz` (capturile), `*.png` (grafice ca în halscope) și `report.md`.
Pentru o sesiune nouă de la zero, șterge sau mută dosarul.

Gain-urile găsite rămân live până la repornirea LinuxCNC, ca să poți încerca
mașina cu ele. Ca să rămână permanent, le treci în INI din raport.

## Simulare (fără mașină)

`sim/` conține un strung simulat. Fiecare joint are un „driver” cu câștig 1.22 și
două lag-uri, cu aceiași parametri de start ca WeilerE30:

```sh
cd sim && linuxcnc autotune_sim.ini &      # headless
python3 sim/prepare.py                     # E-stop reset, ON, home
./autotune.py --ini sim/autotune_sim.ini --joint 1 --window 100 300 -y auto
kill $(cat /tmp/autotune-sim.pid)          # oprește simularea
```

Pe simulare, Z ajunge de la 0.13 mm la 0.010 mm (trapezoidal) și la 0.0008 mm
(S-curve + FF3).

## Note pentru strungul WeilerE30

- FF1 = 0.81967 este exact 2000/2440: STEP_SCALE nu corespunde cu pasul real.
  Raportul propune STEP_SCALE = 2000/4000 și înmulțește gain-urile cu 1.22.
  După corectare, STEPGEN_MAX_ACC devine mm/s² reali și trebuie să fie cu
  ≥ 25% peste MAX_ACCELERATION (pe Z, 2200 față de 2000 e prea puțin).
- `setp hm2_7i95.0.encoder.timer-number 1` în HAL face citirea encoderelor pe
  același timer DPLL ca stepgen-urile (raportul o semnalează).
- FERROR 10/20 și MIN_FERROR 10 sunt practic dezactivate; raportul propune
  valori din erorile măsurate.
# lcnc_autotune
