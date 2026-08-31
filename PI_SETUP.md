# Salt Sentinel — using it as it stands right now

Not a bring-up guide. This is what actually works today and how to run it —
for the 25 Aug video/report deliverables, not the full autonomous system
(that's the 1 Sep target, and it needs the I²C sensor bring-up, ESP32 serial
link, and camera focus lock this doc used to cover — none of that is done or
required for what's due this week).

**Current real state of the rig:**
- Camera is physically wired to the Pi, but the Pi isn't in the loop for
  anything live right now.
- I²C sensors (thermal, ToF, climate, etc.) are mounted on the boom arm as
  physical props — not wired, not read by anything.
- The ESP32 is running the **demo** Bluetooth sketch
  (`esp32/salt_sentinel_demo_bt/`), not the real USB-serial drive firmware
  (`esp32/salt_sentinel_drive/`). It's driven by hand from a phone, not by
  the Pi.

---

## 1 · Driving the robot (Bluetooth, for filming)

Flash `esp32/salt_sentinel_demo_bt/salt_sentinel_demo_bt.ino` if it isn't
already on the board (Arduino IDE, same as any other ESP32 sketch — this one
needs Bluetooth Classic, so the board setting must be a plain ESP32, not a
C3/S2/S3 variant).

**Power during filming: the Pi stays off.** The real firmware assumes the
ESP32 is powered from the Pi's USB port — with the Pi off, the ESP32 needs
its own USB power (a power bank or wall adapter straight into its USB port)
for the shoot. Motor power (pack → SW1 → the 7.2V buck → drivers) is on its
own circuit already and doesn't depend on the Pi either way.

Pair a phone to Bluetooth device **SaltSentinelDemo** using a gamepad-style
Bluetooth RC app (the kind with a d-pad + PlayStation-style face buttons,
sending one ASCII character per button):

| Send | Does |
|---|---|
| `F` | drive forward |
| `B` | drive backward |
| `X` (or anything else) | stop |

No `L`/`R` — all 4 drivers now share one RPWM/LPWM signal pair, so there's
no independent left/right control, only straight forward/backward.

`C`/`T`/`S`/`A`/`P` are accepted and ignored — nothing on this rig needs them
for straight drive footage.

No auto-stop timeout — `F`/`B` hold until you send `X` or another command.
Most BT RC apps send a button's character once per press, not repeatedly
while held, so a timeout-based stop would cut the motors mid-hold.

To go back to real autonomous driving later, reflash
`esp32/salt_sentinel_drive/salt_sentinel_drive.ino` — that's the one that
takes commands over USB serial from the Pi and is untouched by any of this.

---

## 2 · Generating the demo outputs (CSV / heatmap / PDF)

This is pure software — no sensors, no GPIO, no serial link required. It runs
identically on the Pi or on a laptop; use whichever is easier to point a
screen-recording at.

**On the Pi**, use a venv so these packages don't fight with anything
system-managed (Raspberry Pi OS blocks plain `pip install` outside one):

```bash
cd salt_sentinel/pi
python3 -m venv --system-site-packages ~/ss-venv
source ~/ss-venv/bin/activate      # do this every new terminal session
pip install -r requirements.txt    # once per venv — picks up reportlab if you don't have it yet
python cli.py demo --photos yourwall.jpg
```

`--system-site-packages` matters if you ever run this alongside anything
that needs `picamera2` (apt-only, won't install via pip) — harmless to
include even though today's `demo` command doesn't touch the camera.

**On a laptop** (Windows/Mac/Linux), a venv is optional — a plain
`pip install -r requirements.txt` works fine there:

```bash
cd salt_sentinel/pi
pip install -r requirements.txt
python cli.py demo --photos yourwall.jpg
```

No `--photos`? It falls back to a procedural placeholder wall automatically
so the pipeline still runs end-to-end — swap in a real photo whenever you
have one, same command.

Output lands in `pi/data/DEMO_run_<timestamp>/`:

| File | What it is |
|---|---|
| `wall_heatmap.png` | full-frame thermal-style render, panorama-stitched if you gave 2+ overlapping photos |
| `readings.csv` | per-station table: position, cooling, risk score, flag |
| `condition_report.pdf` | same data as a formatted report, heatmap embedded |
| `stations.jsonl` | raw underlying records, if you need to inspect anything by hand |

---

## What's not live right now (and doesn't need to be, yet)

- I²C sensor bus, addresses, XSHUT sequencing
- Real drive firmware / Pi↔ESP32 serial link
- Camera focus lock / exposure calibration
- Autonomous patrol, wall-following, station logic

All of that is real, designed, and in the codebase (`saltsentinel/sensors.py`,
`thermal.py`, `drive.py`, `patrol.py`) — it's just not wired or required for
this week's deliverables. Bring-up for that stack is a separate task ahead of
the 1 Sep working-robot milestone.
