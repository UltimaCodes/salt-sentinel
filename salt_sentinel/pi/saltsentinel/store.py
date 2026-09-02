"""Per-station records.

Rule from the design: no reading is ever compared against one taken under
non-comparable conditions. So every record carries the pass mode, the ambient
conditions, the dew-point margin and the standoff it was taken at. A record
missing those is not a measurement, it is a number.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path

from . import config as cfg


@dataclass
class StationRecord:
    station: int
    pass_mode: str                  # "far" | "near" | "deliquescence_dawn" | ...
    stamp: float = field(default_factory=time.time)
    iso: str = ""

    standoff_mm: float = float("nan")
    wall_yaw_deg: float = float("nan")
    odo_mm: float = float("nan")

    air_temp_c: float = float("nan")
    rh_pct: float = float("nan")
    dew_margin_c: float = float("nan")
    raining: bool = False
    deliquescence_open: bool = False

    thermal: dict = field(default_factory=dict)
    surface: dict = field(default_factory=dict)
    surface_temp_c: float = float("nan")
    geometry_warnings: list = field(default_factory=list)

    registration_shift_px: tuple = (0.0, 0.0)
    registration_inliers: int = 0

    risk_score: float = float("nan")
    flagged: bool = False

    notes: str = ""

    def __post_init__(self):
        if not self.iso:
            self.iso = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(self.stamp))

    @property
    def comparable(self) -> bool:
        """Cheap gate before this record is allowed into a trend."""
        if self.raining:
            return False
        if not (self.dew_margin_c == self.dew_margin_c) or self.dew_margin_c < 1.5:
            return False
        if not (self.standoff_mm == self.standoff_mm):
            return False
        return True


class Store:
    def __init__(self, run_name: str | None = None, root: Path = cfg.DATA):
        # Named without a verdict yet - finalize() appends OUTPUTCLEAN/
        # OUTPUTSALT once the wall is actually done and we know which.
        self.run = run_name or time.strftime("%Y%m%d-%H%M%S")
        self.root = root
        self.dir = root / self.run
        self.dir.mkdir(parents=True, exist_ok=True)
        self.jsonl = self.dir / "stations.jsonl"
        self.finalized = False

    def append(self, rec: StationRecord):
        with self.jsonl.open("a", encoding="utf-8") as f:
            f.write(json.dumps(asdict(rec), default=list) + "\n")
        return rec

    def images_dir(self, station: int) -> Path:
        d = self.dir / f"station_{station:03d}"
        d.mkdir(parents=True, exist_ok=True)
        return d

    def load(self) -> list[dict]:
        if not self.jsonl.exists():
            return []
        return [json.loads(l) for l in self.jsonl.read_text(encoding="utf-8").splitlines()
                if l.strip()]

    def export_csv(self, path: Path | None = None, header_comment: str | None = None) -> Path:
        """Flatten every station record into one row, with thermal/surface
        fields pulled up to top level. header_comment, if given, is written
        as a '#'-prefixed line before the header."""
        import csv
        from .thermal import frame_width_mm
        path = path or (self.dir / "readings.csv")
        fields = ["station", "pass_mode", "iso", "x_m", "y_m", "radius_mm",
                   "standoff_mm", "wall_yaw_deg", "air_temp_c", "rh_pct",
                   "dew_margin_c", "raining", "moisture_index_c", "peak_cooling_c",
                   "damp_row", "bright_fraction", "roughness",
                   "risk_score", "flagged", "comparable"]
        with path.open("w", newline="", encoding="utf-8") as f:
            if header_comment:
                f.write(f"# {header_comment}\n")
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for r in self.load():
                th = r.get("thermal") or {}
                sf = r.get("surface") or {}
                comparable = StationRecord(**r).comparable
                standoff = r.get("standoff_mm") or 1000.0
                w.writerow({
                    "station": r["station"], "pass_mode": r["pass_mode"],
                    "iso": r["iso"],
                    "x_m": round((r.get("odo_mm") or 0.0) / 1000.0, 3),
                    "y_m": round((th.get("damp_height_mm") or 0.0) / 1000.0, 3),
                    "radius_mm": round(frame_width_mm(standoff) / 2.0, 1),
                    "standoff_mm": r.get("standoff_mm"),
                    "wall_yaw_deg": r.get("wall_yaw_deg"),
                    "air_temp_c": r.get("air_temp_c"), "rh_pct": r.get("rh_pct"),
                    "dew_margin_c": r.get("dew_margin_c"), "raining": r.get("raining"),
                    "moisture_index_c": th.get("moisture_index"),
                    "peak_cooling_c": th.get("peak_cooling_c"),
                    "damp_row": th.get("damp_row"),
                    "bright_fraction": sf.get("bright_fraction"),
                    "roughness": sf.get("roughness"),
                    "risk_score": r.get("risk_score"), "flagged": r.get("flagged"),
                    "comparable": comparable,
                })
        return path

    @staticmethod
    def _usb_mounts() -> list[Path]:
        """Writable removable-media mount points.

        Scanning /media/<user>/<label> alone assumes some desktop-session
        automount service already reacted to the drive being plugged in.
        That's true if someone's actually logged into the Pi's graphical
        session (or connected over VNC), but false over a plain SSH/remote
        shell session with nobody at the desktop - nothing mounts the drive
        at all in that case, and this silently returned [] every time.
        So: also enumerate removable partitions directly via lsblk and
        self-mount any that aren't mounted yet, via udisksctl - which works
        with no desktop session and (per Raspberry Pi OS's default polkit
        rules) without root either. Falls back to [] on any failure, same
        as before - a missing/failed mount is "no USB backup this run",
        never a crash, since the Pi's own copy is already safely written.
        """
        out: list[Path] = []
        seen: set[str] = set()

        media = Path("/media")
        if media.is_dir():
            for user_dir in media.iterdir():
                if not user_dir.is_dir():
                    continue
                for mount in user_dir.iterdir():
                    if mount.is_dir() and os.access(mount, os.W_OK):
                        out.append(mount)
                        seen.add(str(mount))

        try:
            raw = subprocess.run(
                ["lsblk", "-J", "-o", "NAME,RM,TYPE,MOUNTPOINT"],
                capture_output=True, text=True, timeout=5, check=True,
            ).stdout
            devices = json.loads(raw).get("blockdevices", [])
        except Exception:
            devices = []

        def walk(nodes):
            for node in nodes:
                if node.get("type") == "part" and node.get("rm"):
                    mp = node.get("mountpoint")
                    if not mp:
                        try:
                            r = subprocess.run(
                                ["udisksctl", "mount", "-b", f"/dev/{node['name']}",
                                 "--no-user-interaction"],
                                capture_output=True, text=True, timeout=15,
                            )
                            # stdout on success: "Mounted /dev/sda1 at /media/pi/LABEL."
                            if " at " in r.stdout:
                                mp = r.stdout.strip().rsplit(" at ", 1)[1].rstrip(".")
                        except Exception:
                            mp = None
                    if mp and mp not in seen and os.access(mp, os.W_OK):
                        out.append(Path(mp))
                        seen.add(mp)
                walk(node.get("children", []))

        walk(devices)
        return out

    def finalize(self, salt_detected: bool) -> Path:
        """Call once, after the wall is actually done. Renames the run
        directory to carry the OUTPUTCLEAN/OUTPUTSALT verdict, then mirrors
        the whole thing onto every writable USB drive found - a judge or a
        conservator should be able to pull the data without touching the Pi
        itself. Safe to call more than once; only the first call does anything.
        """
        if self.finalized:
            return self.dir
        suffix = "OUTPUTSALT" if salt_detected else "OUTPUTCLEAN"
        final_dir = self.root / f"{self.run}{suffix}"
        if final_dir != self.dir:
            self.dir.rename(final_dir)
            self.dir = final_dir
            self.jsonl = self.dir / "stations.jsonl"
        self.finalized = True

        for mount in self._usb_mounts():
            try:
                dest = mount / self.dir.name
                shutil.copytree(self.dir, dest, dirs_exist_ok=True)
            except Exception as e:
                print(f"USB backup to {mount} failed (data is still on the Pi): {e!r}")
        return self.dir

    @staticmethod
    def latest_run(root: Path = cfg.DATA) -> Path | None:
        runs = sorted([p for p in root.iterdir()
                      if p.is_dir() and p.name[:8].isdigit()])
        return runs[-1] if runs else None
