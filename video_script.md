# Salt Sentinel — 90s Demo Video: Script & Storyboard

Honest framing throughout: voiceover describes the finished system's design;
visuals show the current staged/manually-driven prototype. Nothing in the
voiceover claims autonomy the footage doesn't show.

Bluetooth control reference (see `esp32/salt_sentinel_demo_bt/`): pair to
**SaltSentinelDemo** with a gamepad-style Bluetooth RC app — `F`/`B` drive,
`X` stops. No L/R (all 4 drivers share one signal pair now, straight
forward/back only) and no auto-stop timeout — commands hold until you send
another one.

**Power during filming: only the ESP32 is powered. The Pi is off.** No
sensors are live, and the camera/LED ring on the boom arm are props only for
this shoot — don't frame any shot as if the Pi-side software is running
during the robot footage, because it isn't. This is also why shot 6 (the
screen recording) has to be captured as a completely separate session, not
simultaneously with the driving shots — there's no world in which both are
true at once on this rig right now.

---

## Shot list

| # | Time | Shot | Voiceover | Notes |
|---|------|------|-----------|-------|
| 1 | 0:00–0:08 | Team + robot, static wide shot | "We're Nixor Engineering Solutions, and this is Salt Sentinel." | Doubles as the required team photo — shoot this one carefully, good light |
| 2 | 0:08–0:20 | Wall damage close-up — use the real test-wall photo/footage if you have it | "Mohenjo-daro is 4,500 years old, and it's being destroyed by salt rising up through the brick. The fix already exists — knowing *where* to apply it doesn't." | No robot on screen yet — let the problem land first |
| 3 | 0:20–0:40 | **Robot driving forward/back along the wall (BT `F`/`B`), 20s continuous** | "Salt Sentinel patrols a wall without ever touching it, stopping at fixed stations to measure." | The centerpiece shot. Straight line only — no turning available on this build, so frame the wall run so a straight path reads as intentional, not limited. Smooth, deliberate F→stop→B beats jerky. Do several takes, pick the cleanest |
| 4 | 0:40–0:55 | Close-ups: LED ring, camera housing, thermal sensor mount | "A thermal camera finds damp patches by temperature. A camera under controlled lighting measures salt crust and surface texture." | Unpowered props for this shot — Pi's off, nothing is actually capturing. Say "camera under controlled lighting," not "from four directions" — matches the corrected report text |
| 5 | 0:55–1:10 | Screen recording: `python cli.py demo` running, then the PDF/heatmap/CSV opening on screen | "Each patrol produces a condition report: a risk score per station, a moisture map of the wall, and the full reading history." | Filmed separately, laptop or Pi powered up on its own — not simultaneous with the robot/driving shots |
| 6 | 1:10–1:30 | Close on impact line, team/logo card | "So conservators can treat what's actively getting worse — not just what's easiest to reach." | Land on the mission, not a feature list |

Total: 90s.

---

## Things to avoid saying

- Don't say "detects salt" over the heatmap — say "moisture map" or "risk score," which is what it actually measures and what your report claims.
- Don't claim the four-direction lighting method (already fixed in the report) — say "controlled lighting," not "from four directions."
- Don't imply the robot can turn — it currently can't (single shared motor-driver signal, forward/back only). Frame shot 3's straight path as the shot, not as a limitation to explain away.
- The written report carries its own disclaimer section covering manual control and the staged data preview — keep that consistent with whatever you say in front of the judges live.

---

## Filming logistics

1. Daylight, not artificial light if possible — better exposure, less flicker in the footage.
2. Get more than 20s of raw footage for shot 3 — film several full runs, cut down to the cleanest continuous 20s rather than trying to nail one take exactly.
3. Shoot the screen recording (shot 5) separately, calmly, with time to steady the camera or use a direct screen capture instead of filming a monitor.
4. Order of filming doesn't have to match final cut order — get shot 1 (team) done first since it needs the most people/coordination, then the robot shots, then the screen recording last once you've actually run `python cli.py demo`.
