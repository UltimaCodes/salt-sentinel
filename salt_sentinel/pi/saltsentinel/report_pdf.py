"""PDF condition report: header, wall heatmap image, per-station risk table."""

from __future__ import annotations

import time

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.platypus import (SimpleDocTemplate, Table, TableStyle, Paragraph,
                                Spacer, Image as RLImage)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle

from .thermal import frame_width_mm


def _station_xyr(r: dict) -> tuple[float, float, float]:
    """X = position along the wall, Y = detected patch height (from
    thermal.analyse()'s damp-height figure), radius = sensor footprint at
    the recorded standoff."""
    th = r.get("thermal") or {}
    x_m = (r.get("odo_mm") or 0.0) / 1000.0
    y_m = (th.get("damp_height_mm") or 0.0) / 1000.0
    standoff = r.get("standoff_mm") or 1000.0
    radius_mm = frame_width_mm(standoff) / 2.0
    return x_m, y_m, radius_mm


def build(records: list[dict], heatmap_path: str, out_path: str,
         site_name: str = "Validation wall", preview: bool = False) -> str:
    styles = getSampleStyleSheet()
    title_style = styles["Title"]
    body = styles["BodyText"]
    footer_style = ParagraphStyle("footer", parent=body, textColor=colors.grey,
                                  fontName="Helvetica", fontSize=7)

    doc = SimpleDocTemplate(out_path, pagesize=A4,
                            topMargin=18 * mm, bottomMargin=16 * mm,
                            leftMargin=16 * mm, rightMargin=16 * mm)
    story = []

    story.append(Paragraph("Salt Sentinel - Wall Condition Report", title_style))
    story.append(Paragraph(f"{site_name} &nbsp;&middot;&nbsp; "
                           f"{time.strftime('%Y-%m-%d %H:%M')} &nbsp;&middot;&nbsp; "
                           f"{len(records)} stations", body))
    story.append(Spacer(1, 6 * mm))

    if heatmap_path:
        img = RLImage(heatmap_path, width=170 * mm, height=170 * mm * 7 / 16)
        story.append(img)
        story.append(Paragraph(
            "Wall panorama with thermal moisture overlay. Cooler-mapped "
            "colour (dark/violet) = evaporative cooling consistent with "
            "active moisture; warmer colour = dry brick at ambient.", body))
        story.append(Spacer(1, 6 * mm))

    header = ["Station", "X (m)", "Y (m)", "R (mm)", "Cooling (°C)", "Risk", "Flag"]
    rows = [header]
    for r in sorted(records, key=lambda x: -(x.get("risk_score") or 0)):
        th = r.get("thermal") or {}
        x_m, y_m, radius_mm = _station_xyr(r)
        rows.append([
            str(r["station"]),
            f"{x_m:.2f}",
            f"{y_m:.2f}",
            f"{radius_mm:.0f}",
            f"{-th.get('moisture_index', 0.0):.2f}",
            f"{r.get('risk_score', 0.0):.2f}",
            "FLAGGED" if r.get("flagged") else "",
        ])

    t = Table(rows, colWidths=[16 * mm, 20 * mm, 20 * mm, 20 * mm, 28 * mm, 18 * mm, 22 * mm])
    style = [
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2b2b2b")),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
        ("FONTSIZE", (0, 0), (-1, -1), 9),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.grey),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f2f2f2")]),
        ("ALIGN", (1, 0), (-1, -1), "CENTER"),
    ]
    for i, r in enumerate(rows[1:], start=1):
        if r[-1] == "FLAGGED":
            style.append(("TEXTCOLOR", (0, i), (-1, i), colors.HexColor("#8a1f11")))
            style.append(("FONTNAME", (0, i), (-1, i), "Helvetica-Bold"))
    t.setStyle(TableStyle(style))
    story.append(t)

    story.append(Spacer(1, 6 * mm))
    n_flag = sum(1 for r in records if r.get("flagged"))
    story.append(Paragraph(
        f"{n_flag} of {len(records)} stations flagged (risk ≥ threshold). "
        "Risk weights are provisional, pending calibration against the "
        "validation wall's known dosed/clean patches (report section 3.6).",
        body))

    if preview:
        story.append(Spacer(1, 10 * mm))
        story.append(Paragraph(
            "Preview report, generated ahead of the working prototype "
            "(target: 1 Sep 2026) - see report section 4.6 for the "
            "validation timeline.", footer_style))

    doc.build(story)
    return out_path
