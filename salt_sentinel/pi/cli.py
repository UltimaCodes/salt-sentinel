#!/usr/bin/env python3
"""Salt Sentinel command line.

  python cli.py selftest          check every sensor and the drive link
  python cli.py teleop            manual override - WASD, space = stop
  python cli.py calib-camera      meter the scene ONCE and freeze the settings
  python cli.py calib-thermal     two-point per-pixel flat field
  python cli.py station           one station capture, no driving
  python cli.py patrol            follow the wall and sample
  python cli.py geometry          standoff maths, no hardware needed

Add --sim anywhere to run without hardware.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from saltsentinel import config as cfg
from saltsentinel.drive import Drive
from saltsentinel.sensors import SensorHub
from saltsentinel.thermal import ThermalCamera, frame_width_mm, mm_per_pixel
from saltsentinel.store import Store


def cmd_geometry(a):
    print("THERMAL: does the frame contain a dry reference?\n")
    print(f"{'standoff':>9}{'frame':>10}{'mm/px':>9}   verdict")
    print("-" * 62)
    for d in (50, 100, 250, 500, 750, 1000, 1250):
        ok = ("yes" if ThermalCamera.frame_spans_reference(d)
              else "NO - differential can read zero")
        print(f"{d:>7.0f}mm{frame_width_mm(d):>9.0f}mm{mm_per_pixel(d):>9.1f}   {ok}")

    from saltsentinel.photometric import incidence_deg
    print("\nLED RING (not fitted yet - config.LED_FITTED is False. Dimensioning "
          "math for whenever it is, photometric stereo): incidence angle, "
          "25-35 deg useful band\n")
    for r in (60, 90, 120, 150):
        ang = incidence_deg(cfg.STANDOFF_NEAR_MM, r)
        print(f"  radius {r:>3d} mm at {cfg.STANDOFF_NEAR_MM:.0f} mm -> {ang:>4.1f} deg  "
              f"{'good' if 25 <= ang <= 35 else 'outside band'}")

    from saltsentinel import geometry as geo
    print("\nOVERLAY: thermal-to-camera registration vs measured range\n")
    print(f"{'range':>8}{'parallax':>11}{'% frame':>10}{'usable cols':>13}")
    for d in (100, 250, 500, 1000):
        g = geo.overlay_geometry(d)
        print(f"{d:>6.0f}mm{g.parallax_px:>10.0f}px"
              f"{g.parallax_px/cfg.CAM_PX_WIDTH*100:>9.1f}%"
              f"{g.usable_thermal_cols:>10.1f}/8")
    print("\n  Parallax scales with distance, so an ASSUMED range misregisters")
    print("  the two frames. This is why the range must be measured on the arm.")


def cmd_selftest(a):
    ok = True
    print("=== drive link ===")
    try:
        with Drive(simulate=a.sim) as d:
            time.sleep(1.2)
            t = d.telemetry
            print(f"  enabled={t.enabled}")
            if not a.sim and t.stale:
                print("  FAIL no telemetry - check the port and that the ESP32 is flashed")
                ok = False
    except Exception as e:
        print(f"  FAIL {e}")
        ok = False

    print("=== i2c sensors ===")
    try:
        hub = SensorHub(simulate=a.sim)
        for n in cfg.TOF_ORDER:
            print(f"  tof {n:<8} {hub.read_tof(n):8.1f} mm  (0x{cfg.TOF_ADDR[n]:02X})")
        if "front" not in cfg.TOF_FITTED:
            print("  tof front    NOT FITTED - no corner detection")
        pose = hub.wall_pose()
        yaw = f"{pose.yaw_deg:+.1f} deg" if pose.yaw_deg == pose.yaw_deg else "unavailable"
        print(f"  wall pose  {pose.distance_mm:.0f} mm  yaw {yaw}  "
              f"({pose.n_sensors} sensor{'s' if pose.n_sensors != 1 else ''})")
        c = hub.climate()
        print(f"  climate    {c.temp_c:.1f} C  {c.rh_pct:.0f} %RH  dew {c.dew_c:.1f} C "
              f"margin {c.dew_margin_c:+.1f} C")
        print(f"  deliquescence channel {'OPEN' if c.deliquescence_open else 'closed'}")
    except Exception as e:
        print(f"  FAIL {e}")
        ok = False

    print("=== thermal ===")
    try:
        tc = ThermalCamera(simulate=a.sim)
        r = tc.analyse(cfg.STANDOFF_FAR_MM, 28.0, n=5)
        print(f"  dry ref {r.dry_reference_c:.2f} C   cooling {r.moisture_index:+.2f} C")
        print(f"  peak {r.peak_cooling_c:+.2f} C   damp row {r.damp_row}   "
              f"noise {r.noise_floor_c:.3f} C")
        print(f"  {r.mm_per_px:.1f} mm/pixel at {cfg.STANDOFF_FAR_MM:.0f} mm")
    except Exception as e:
        print(f"  FAIL {e}")
        ok = False

    print("\n" + ("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED"))
    return 0 if ok else 1


def cmd_teleop(a):
    """Manual override. Takes precedence over any autonomous behaviour."""
    try:
        import termios, tty
    except ImportError:
        print("teleop needs a POSIX terminal (run it on the Pi)")
        return 1

    speed = cfg.CRUISE
    print("WASD drive | space stop | [ ] speed | q quit (disables drivers)")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    with Drive(simulate=a.sim) as d:
        d.enable(True)
        try:
            tty.setcbreak(fd)
            while True:
                ch = sys.stdin.read(1).lower()
                if ch == "q":
                    break
                elif ch == "w":
                    d.drive(speed, speed)
                elif ch == "s":
                    d.drive(-speed, -speed)
                elif ch == "a":
                    d.drive(-speed, speed)
                elif ch == "d":
                    d.drive(speed, -speed)
                elif ch == " ":
                    d.stop()
                elif ch == "[":
                    speed = max(80, speed - 40)
                    print(f"speed {speed}")
                elif ch == "]":
                    speed = min(1000, speed + 40)
                    print(f"speed {speed}")
                for e in d.pop_events():
                    print(f"  EVENT {e}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            d.stop()
    return 0


def cmd_calib_camera(a):
    from saltsentinel.camera import Camera
    cam = Camera(simulate=a.sim)
    print("Metering once under the lighting you will actually shoot under...")
    lock = cam.autotune_once(cfg.STANDOFF_NEAR_MM)
    print(f"  exposure    {lock.exposure_us} us")
    print(f"  gain        {lock.analogue_gain:.2f}")
    print(f"  colour      {lock.colour_gains}")
    if lock.lens_position is not None:
        print(f"  lens        {lock.lens_position:.2f} dioptres "
              f"(~{1000/lock.lens_position:.0f} mm)")
    else:
        print("  lens        fixed-focus sensor - no software lens control, "
              "focus is whatever the ring is physically set to")
    print("\nFrozen and saved. Do NOT re-run between visits - that breaks comparability.")
    cam.close()
    return 0


def cmd_calib_thermal(a):
    import numpy as np
    tc = ThermalCamera(simulate=a.sim)
    print("Point the sensor at a UNIFORM flat surface at ambient.")
    input("  press enter when steady... ")
    cool = np.stack([tc._raw() for _ in range(40)])
    cool_ref = float(input("  true surface temperature in C: ") or 25.0)
    print("Now warm the same surface a few degrees (hand-warm is enough).")
    input("  press enter when steady... ")
    warm = np.stack([tc._raw() for _ in range(40)])
    warm_ref = float(input("  true surface temperature in C: ") or 32.0)
    g, o = tc.calibrate_flat_field(cool, warm, cool_ref, warm_ref)
    print(f"  gain   {g.min():.3f} .. {g.max():.3f}")
    print(f"  offset {o.min():+.3f} .. {o.max():+.3f} C")
    print("Saved. This is what turns +/-2.5 C absolute accuracy into a usable "
          "pixel-to-pixel differential.")
    return 0


def cmd_station(a):
    from saltsentinel import geometry as geo
    hub = SensorHub(simulate=a.sim)
    tc = ThermalCamera(simulate=a.sim)
    store = Store()
    c = hub.climate()
    pose = hub.wall_pose()
    d = pose.distance_mm if pose.valid else cfg.STANDOFF_FAR_MM
    r = tc.analyse(d, c.temp_c)
    yaw = f"{pose.yaw_deg:+.1f} deg" if pose.yaw_deg == pose.yaw_deg else "unavailable"
    print(f"standoff {pose.distance_mm:.0f} mm  yaw {yaw}")
    print(f"cooling  {r.moisture_index:+.2f} C   peak {r.peak_cooling_c:+.2f} C")
    print(f"damp row {r.damp_row}  height {r.damp_height_mm:.0f} mm")
    print(f"noise floor {r.noise_floor_c:.3f} C  <- nothing smaller is claimable")
    for w in geo.check(d):
        print(f"  ! {w}")
    print(f"record dir {store.dir}")
    return 0


def _placeholder_wall_photo(path):
    """Procedural stand-in brick wall, used until a real photo is supplied."""
    import numpy as np
    import cv2
    h, w = 900, 2028
    img = np.full((h, w, 3), (58, 74, 96), dtype=np.uint8)   # BGR brick-ish red
    for row in range(0, h, 60):
        offset = 90 if (row // 60) % 2 else 0
        cv2.line(img, (0, row), (w, row), (40, 40, 40), 3)
        for col in range(-90, w, 180):
            cv2.line(img, (col + offset, row), (col + offset, row + 60), (40, 40, 40), 3)
    noise = np.random.default_rng(1).normal(0, 6, img.shape).astype(np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    cv2.imwrite(str(path), img)
    return str(path)


def cmd_demo(a):
    """Generate the CSV, wall heatmap and PDF report from the staged demo
    scenario. Pass --photos once real wall photos are available."""
    from saltsentinel import demo_scenario, wallmap, report_pdf

    store = demo_scenario.build_run()
    records = store.load()
    print(f"wrote {len(records)} simulated station records to {store.dir}")

    photos = a.photos
    if not photos:
        ph_path = store.dir / "placeholder_wall.jpg"
        photos = [str(_placeholder_wall_photo(ph_path))]
        print(f"no --photos given: using a procedural placeholder at {ph_path}")
        print("re-run with --photos <your wall photo(s)> once you have them")

    heat_path = store.dir / "wall_heatmap.png"
    res = wallmap.render(
        photos,
        station_odo_mm=[r["odo_mm"] for r in records],
        station_cooling_c=[-r["thermal"]["moisture_index"] for r in records],
        station_damp_height_mm=[r["thermal"]["damp_height_mm"] for r in records],
        out_path=str(heat_path),
    )
    print(f"heatmap ({'stitched from ' + str(res.stitched_from) + ' photos' if res.stitched_from > 1 else 'single photo'}): {heat_path}")

    csv_path = store.export_csv()
    print(f"csv: {csv_path}")

    pdf_path = store.dir / "condition_report.pdf"
    report_pdf.build(records, str(heat_path), str(pdf_path))
    print(f"pdf: {pdf_path}")
    return 0


def cmd_patrol(a):
    from saltsentinel.patrol import Patrol
    from saltsentinel.camera import Camera
    from saltsentinel.leds import LedRing
    hub = SensorHub(simulate=a.sim)
    tc = ThermalCamera(simulate=a.sim)
    cam = None if a.no_camera else Camera(simulate=a.sim)
    # LED_FITTED: no ring is wired yet - without this gate a real (non-sim)
    # run would still try to drive GPIO17 as if a ring were connected there.
    ring = LedRing(simulate=a.sim) if (cfg.LED_FITTED and not a.no_camera) else None
    with Drive(simulate=a.sim) as d:
        p = Patrol(d, hub, tc, cam, ring)
        if cam and ring:
            p.calibrate_white()
        p.run(max_stations=a.stations)
    return 0


def main():
    # --sim needs to parse whether it comes before or after the subcommand
    # ("cli.py --sim selftest" or "cli.py selftest --sim"), so every parser
    # gets its own --sim with default=SUPPRESS: if a subparser doesn't see
    # --sim on its own tokens it must leave `sim` alone rather than reset
    # it, or "--sim selftest" would parse sim=True at the top level and then
    # have the selftest subparser immediately clobber it back to False.
    # (Sharing one parent parser object via parents=[...] across multiple
    # add_parser() calls silently breaks this SUPPRESS default - each
    # parser needs its own --sim action, not a shared one.) The actual
    # default (False) is set once, explicitly, on the top-level parser.
    ap = argparse.ArgumentParser(description="Salt Sentinel")
    ap.add_argument("--sim", action="store_true", default=argparse.SUPPRESS,
                     help="run without hardware")
    ap.set_defaults(sim=False)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, fn in (("selftest", cmd_selftest), ("teleop", cmd_teleop),
                     ("calib-camera", cmd_calib_camera),
                     ("calib-thermal", cmd_calib_thermal),
                     ("station", cmd_station), ("geometry", cmd_geometry),
                     ("patrol", cmd_patrol), ("demo", cmd_demo)):
        s = sub.add_parser(name)
        s.add_argument("--sim", action="store_true", default=argparse.SUPPRESS,
                        help="run without hardware")
        s.set_defaults(func=fn)
        if name == "patrol":
            s.add_argument("--stations", type=int, default=20)
            s.add_argument("--no-camera", action="store_true")
        if name == "demo":
            s.add_argument("--photos", nargs="*", default=[],
                           help="one or more overlapping wall photos - 2+ get "
                                "real cv2 panorama stitching, 1 is used as-is")
    a = ap.parse_args()
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main() or 0)
