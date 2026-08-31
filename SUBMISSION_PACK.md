# Submission pack — due Tue 25 Aug

**You write the prose.** WRO rule: the report and video must be made by the team,
not the coach and not anyone else. What follows is a skeleton, a page budget, and
a map of which material you already have for each section. Filling it in is your
job — and judges may ask you about any sentence in it.

---

## The reframe

| Deadline | What it actually needs | Points it drives |
|---|---|---|
| **Tue 25 Aug** | a PDF and 90 s of footage | **75 / 200** |
| **Fri 4 Sep** | a robot that runs live in a booth | **125 / 200** |

The report does **not** depend on the robot being finished. It depends on the
design being explained well, and that is already decided. Say honestly that
validation is in progress — a stated limitation scores better than a vague claim.

---

## Report skeleton — 20 pages hard maximum

Front page, contents and sources are **not** counted. Anything over 20 pages is
**not scored at all**, so cutting is not optional.

### 1 · Team presentation — max 1 page
- Who is in the team, where you are from
- **How you divided the tasks** (they ask this explicitly)
- Team photo

### 2 · Summary project idea — max 1 page
Executive summary. If a judge read only this page, what must they know?
- The problem: saline groundwater rises through Mohenjo-daro's fired brick, crystallises, breaks it apart
- The gap: capping is targeted **by eye**; no systematic condition data exists
- What you built and what it outputs: a per-waypoint **risk map**, not a salt measurement
- Why it matters: turns capping allocation from judgement into a ranked list

### 3 · Presenting the robotic solution — max 12 pages
The rules ask four "general" questions. You have unusually strong answers.

| Question asked | Your material |
|---|---|
| How did you come up with the idea? | UNESCO lists saline action as a threat; conservators cap by eye |
| **What other ideas did you investigate?** | Touch probe, EMI conductivity, ultrasonic array, LIDAR-on-stepper, depth camera, spin-in-place — **and why each was rejected** |
| Similar ideas available? | Kogou & Liang 2025 — SWIR hyperspectral + remote Raman, HySpex SWIR-384, 950–2500 nm, 7–10 m standoff |
| What is different? | Same physics on a six-figure instrument vs a student budget. Application-level novelty, stated as such |

> The rejected-ideas section is the strongest page in the report. Most teams
> cannot explain what they *didn't* build. You can explain six, with physics.
> The EMI one especially: dry salt is non-conductive, so an EMI channel would
> have offered specificity, not earliness.

**Technical aspects** — mechanical construction, coding, challenges faced:
- Two-unit split, Pi/ESP32 boundary drawn by **timing determinism**
- The two-standoff argument: thermal at ~1 m so the frame contains a dry reference, camera at ~25 cm
- In-frame differential: ±2.5 °C absolute vs 0.05 °C pixel-to-pixel, and how flat-fielding exploits the gap
- Photometric stereo: four lights, `I = ρ(N·L)`, albedo and normals
- Safety: watchdog, duty cap, `MOT_EN` pulldown
- **Challenges** — be specific, this is where teams go vague:
  - efflorescence metric was self-referential and scored a clean wall identically to a crusted one; fixed with an absolute white reference
  - a wire only honours one mid-span junction, so generated rails silently failed to connect
  - a floating ADC pin reads noise, not zero — fail-safe not fail-open

**AI usage disclosure** — mandatory, new rule 6.5. State: which systems, for what
purpose, to what extent. Cover the report, the video, the code, the robot model.
Be ready to answer out loud.

### 4 · Social impact & innovation — max 6 pages
- **Impact on society, including possible negative effects.** They ask for the
  negatives explicitly. Honest candidates: dry dormant salt is invisible to every
  channel (L1), so a clean map could create false confidence; risk of displacing
  skilled human judgement; risk of the data being used to justify fewer
  conservation staff; battery and e-waste at end of life.
- **One tried, practical use case** — your Karachi test wall.
- **Business model canvas** (Senior only, 10 pts):
  - Cost structure — BOM, one-off vs consumable
  - Revenue stream — likely *not* unit sales; a service to heritage authorities, or open hardware plus a build service
  - Key resources — the sensing stack, the calibration method, local fabrication
  - Partners — Sindh Antiquities, PSSEC, universities, agricultural salinity bodies
- **Next steps & prototype development** (10 pts) — multi-season deployment,
  species-specific calibration once site lab data exists, the agricultural
  salinity pivot (SDG 11.4 and the same process degrading Sindh's farmland).

### 5 · Slogan — 10 points, currently missing
Do not skip this; it is a scored criterion and it takes an afternoon.
Directions worth trying, then **write your own**:
- lead on the blind spot you close (nobody can see where salt is active)
- lead on the price gap (lab-grade capability, student budget)
- lead on timing (detects at the moment the wall becomes treatable)

---

## Video — 90 seconds hard maximum

100 MB max, .mp4, filmed **by the team**. A phone in one take is explicitly fine —
judges do not expect production value. Main part must show the robot **running**.

| Time | Shot | Ready by |
|---|---|---|
| 0:00–0:12 | Team on camera, name the problem in one sentence | today |
| 0:12–0:25 | The wall — salt damage, close up | today |
| 0:25–0:55 | **Rover driving and following a wall** | Sun 23 |
| 0:55–1:10 | Thermal reading live on screen, damp patch cooler than dry brick | Mon 24 |
| 1:10–1:22 | **Deliquescence demo** — dosed patches darken, controls stay inert | today, no robot needed |
| 1:22–1:30 | Scale-up: same stack, agricultural salinity | today |

**Not in the video:** photometric stereo and the flaking index. They will not be
calibrated by Monday. Describe them in the report, demo them live on 4 Sep.

**Film b-roll continuously from today.** Every bench session, every first
power-up. You cannot go back for footage of a moment that already happened.

---

## Revised schedule

| Day | Build track | Documents track |
|---|---|---|
| **Sat 22** | Power tree only. Track B validation wall starts. | Write §1, §2, and all of §3's design rationale — none of it needs results |
| **Sun 23** | Drivetrain. Rover drives. Film b-roll. | Write §4, business canvas, next steps, slogan |
| **Mon 24** | Pi + 2 ToF + AMG8833. Wall-following. | **Film the video.** Assemble PDF, check page count |
| **Tue 25** | — | **SUBMIT.** Check the 15 MB / 20 page / 100 MB limits |
| Wed 26 – Wed 3 Sep | 10 days: sensor head, photometric, calibration, real-wall data, booth, rehearsal | — |

**Cut from the Monday build so wall-following lands:** camera, LED ring,
photometric calibration. They are not in the video and not in the submission.
They have 10 days afterwards.

---

## Before you hit send

- [ ] PDF ≤ 20 pages single-sided, excluding front page, contents, sources
- [ ] PDF ≤ 15 MB
- [ ] Video ≤ 90 seconds, ≤ 100 MB, .mp4
- [ ] AI disclosure present and specific
- [ ] Slogan present
- [ ] Business model canvas present
- [ ] Next steps present
- [ ] Negative effects discussed, not just benefits
- [ ] Every [E] figure still labelled as an estimate, not presented as data
- [ ] Sources list complete — UNESCO primary source, not Facts and Details
- [ ] Confirm PSSEC's exact submission method and time on the 25th
