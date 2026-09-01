"""Generate reproducible vector figures from SCONE benchmark JSONL files."""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

# PDF eXpress rejects Type 3 text in otherwise vector figures. Preserve
# editable vector text as embedded TrueType glyphs instead.
plt.rcParams["pdf.fonttype"] = 42
plt.rcParams["ps.fonttype"] = 42


ROOT = Path(__file__).resolve().parents[3]
RESULTS = ROOT / "benchmark" / "results"
OUT = Path(__file__).resolve().parent
COLORS = {"articulated-walk": "#E45756", "distal-only-roll": "#F2CF5B", "full-roll": "#4C78A8"}


def records(name: str) -> list[dict]:
    return [json.loads(line) for line in (RESULTS / name).read_text().splitlines() if line]


def flat_metrics() -> None:
    rows = records("flat-nominal.jsonl")
    labels = ["Walk", "Distal\nonly", "Coordinated\narc"]
    keys = ["articulated-walk", "distal-only-roll", "full-roll"]
    values = {row["controller"]: row for row in rows}
    fig, axes = plt.subplots(1, 3, figsize=(7.1, 2.15))
    specs = [
        ("mean_vx_mps", "Mean forward speed", "m/s", 0.18),
        ("mechanical_cost_of_transport", "Mechanical CoT", "ratio", None),
        ("slip_distance_m", "Integrated slip", "m", None),
    ]
    for ax, (metric, title, unit, command) in zip(axes, specs):
        vals = [values[key][metric] for key in keys]
        ax.bar(labels, vals, color=[COLORS[key] for key in keys], width=0.68)
        if command is not None:
            ax.axhline(command, color="#333333", linestyle="--", linewidth=1, label="command")
            ax.legend(frameon=False, fontsize=7, loc="upper left")
        ax.set_title(title, fontsize=9)
        ax.set_ylabel(unit, fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(axis="y", alpha=0.22)
        for i, value in enumerate(vals):
            ax.text(i, value, f"{value:.2f}", ha="center", va="bottom", fontsize=7)
    fig.tight_layout(pad=0.6, w_pad=1.0)
    fig.savefig(OUT / "flat_metrics.pdf", bbox_inches="tight")
    plt.close(fig)


def stair_results() -> None:
    """Time-to-top per controller and riser.

    Column-width figure: it is included at \\columnwidth in a one-column float,
    so it must be authored at that size or the labels become unreadable. The
    binary top-reached outcome is already tabulated, so only the timing panel
    is plotted here and failures are marked in place.
    """
    rows = records("stairs-nominal.jsonl")
    controllers = ["distal-only", "synchronized-open-loop", "full-scone"]
    names = ["Distal only", "Open loop", "Full hybrid"]
    colors = ["#F2CF5B", "#72B7B2", "#4C78A8"]
    risers = [0.10, 0.15, 0.20]
    fig, ax = plt.subplots(figsize=(3.42, 1.95))
    width = 0.25
    x = np.arange(len(risers))
    for j, (controller, name, color) in enumerate(zip(controllers, names, colors)):
        subset = {round(row["maximum_riser_m"], 2): row for row in rows if row["controller"] == controller}
        times = [subset[r]["time_to_top_s"] if subset[r]["top_reached"] else np.nan for r in risers]
        ax.bar(x + (j - 1) * width, times, width, label=name, color=color)
        for i, value in enumerate(times):
            if np.isnan(value):
                ax.text(x[i] + (j - 1) * width, 0.4, "no top", rotation=90,
                        ha="center", va="bottom", fontsize=6)
            else:
                ax.text(x[i] + (j - 1) * width, value, f"{value:.1f}",
                        ha="center", va="bottom", fontsize=6)
    ax.set_ylabel("time to top (s)", fontsize=8)
    ax.set_xlabel("riser height (mm)", fontsize=8)
    ax.set_xticks(x, ["100", "150", "200"])
    ax.set_ylim(0, 19.5)
    ax.tick_params(labelsize=7)
    ax.grid(axis="y", alpha=0.22)
    ax.legend(frameon=False, fontsize=7, ncol=3, loc="upper center",
              handlelength=1.1, columnspacing=1.0, borderaxespad=0.1)
    fig.tight_layout(pad=0.5)
    fig.savefig(OUT / "stair_results.pdf", bbox_inches="tight")
    plt.close(fig)


def arc_geometry() -> None:
    """Schematic for the arc cross-section and the closed-wheel edge pivot.

    Pure geometry: every dimension is derived from the symbols in the text, so
    the figure cannot drift away from the equations it illustrates.
    """
    from matplotlib.patches import Wedge

    r_o, r_i = 0.1225, 0.1125
    opening_deg = 135.0
    gap_lo, gap_hi = 90.0 - opening_deg / 2.0, 90.0 + opening_deg / 2.0

    fig, axes = plt.subplots(1, 2, figsize=(3.42, 1.80))

    # ---- (a) open distal arc, opening facing up, support patch at the bottom
    ax = axes[0]
    ang = np.radians(np.linspace(gap_hi, gap_lo + 360.0, 400))
    ax.plot(r_o * np.cos(ang), r_o * np.sin(ang), color="#4C78A8", lw=1.7)
    ax.plot(r_i * np.cos(ang), r_i * np.sin(ang), color="#4C78A8", lw=1.7)
    for a in (ang[0], ang[-1]):
        ax.plot([r_i * np.cos(a), r_o * np.cos(a)],
                [r_i * np.sin(a), r_o * np.sin(a)], color="#4C78A8", lw=1.7)
    y_chord = r_o * math.sin(math.radians(gap_lo))
    x_chord = r_o * math.cos(math.radians(gap_lo))
    ax.plot([-x_chord, x_chord], [y_chord, y_chord], "--", color="#E45756", lw=1.1)
    ax.text(-0.055, y_chord + 0.009, "$l_o$", fontsize=7.5, color="#E45756", ha="center")
    ax.add_patch(Wedge((0, 0), 0.040, gap_lo, gap_hi, facecolor="#F2CF5B",
                       alpha=0.6, lw=0))
    ax.text(0.0, 0.052, r"$\psi$", fontsize=7.5, ha="center")
    for r, deg, lab, dx in ((r_o, 235.0, "$r_o$", -0.004), (r_i, 300.0, "$r_i$", 0.004)):
        a = math.radians(deg)
        ax.plot([0, r * math.cos(a)], [0, r * math.sin(a)], color="#333333", lw=0.8)
        ax.text(0.55 * r * math.cos(a) + dx, 0.55 * r * math.sin(a) + 0.004,
                lab, fontsize=7.5)
    ax.plot(0, 0, "o", color="#333333", ms=2.5)
    ax.plot([-0.155, 0.155], [-r_o, -r_o], color="#666666", lw=1.0)
    ax.text(0.0, -r_o - 0.030, "support patch", fontsize=6.5, ha="center",
            color="#666666")
    ax.set_xlim(-0.17, 0.17)
    ax.set_ylim(-0.19, 0.115)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("(a) open distal arc", fontsize=8, pad=2)

    # ---- (b) closed-wheel pivot over a sharp nosing at (0, h)
    ax = axes[1]
    h = 0.070
    x_e = math.sqrt(max(0.0, 2 * r_o * h - h * h))
    cx, cy = -x_e, r_o
    circ = np.radians(np.linspace(0, 360, 300))
    ax.plot(cx + r_o * np.cos(circ), cy + r_o * np.sin(circ), color="#4C78A8", lw=1.4)
    ax.plot([-0.30, 0.0], [0, 0], color="#333333", lw=1.3)
    ax.plot([0.0, 0.0], [0, h], color="#333333", lw=1.3)
    ax.plot([0.0, 0.10], [h, h], color="#333333", lw=1.3)
    ax.plot(cx, cy, "o", color="#333333", ms=2.5)
    ax.plot(0.0, h, "o", color="#E45756", ms=3.0)
    ax.plot([cx, 0.0], [cy, cy], "--", color="#E45756", lw=1.0)
    ax.plot([0.0, 0.0], [cy, h], ":", color="#E45756", lw=1.0)
    ax.text(cx / 2, cy + 0.008, "$x_e(h)$", fontsize=7.5, color="#E45756", ha="center")
    ax.text(0.006, (cy + h) / 2 - 0.008, "$r_o\\!-\\!h$", fontsize=7, color="#E45756")
    ax.annotate("", xy=(0.050, 0), xytext=(0.050, h),
                arrowprops=dict(arrowstyle="<->", lw=0.8, color="#333333"))
    ax.text(0.058, h / 2 - 0.006, "$h$", fontsize=7.5)
    ax.add_patch(FancyArrowPatch(
        (cx - 0.040, cy + 0.035), (cx + 0.015, cy + 0.060),
        connectionstyle="arc3,rad=-0.5", arrowstyle="-|>",
        mutation_scale=8, linewidth=0.9, color="#2c7a74",
    ))
    ax.text(cx - 0.065, cy + 0.074, r"$\tau_a$", fontsize=7.5, color="#2c7a74")
    ax.add_patch(FancyArrowPatch(
        (cx - 0.090, cy), (cx - 0.020, cy), arrowstyle="-|>",
        mutation_scale=8, linewidth=0.9, color="#E45756",
    ))
    ax.text(cx - 0.060, cy + 0.010, r"$F_h$", fontsize=7.5, color="#E45756")
    ax.text(-0.295, 0.275, "force-only singularity is not\nan actuated-wheel height limit",
            fontsize=6.2, color="#555555", va="top")
    ax.set_xlim(-0.31, 0.12)
    ax.set_ylim(-0.02, 0.29)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("(b) closed-wheel edge pivot", fontsize=8, pad=2)

    fig.tight_layout(pad=0.3, w_pad=0.5)
    fig.savefig(OUT / "arc_geometry.pdf", bbox_inches="tight")
    plt.close(fig)


def control_architecture() -> None:
    fig, ax = plt.subplots(figsize=(7.1, 1.75))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 2.4)
    ax.axis("off")
    boxes = [
        (0.1, 0.75, 1.45, 0.9, "command\n$[v_x,v_y,\\omega_z]$", "#E8EEF7"),
        (1.95, 0.75, 1.75, 0.9, "tripod phase +\nbounded IK", "#E6F4EA"),
        (4.10, 1.35, 1.65, 0.72, "joints 1--12\nposition targets", "#FFF4D6"),
        (4.10, 0.25, 1.65, 0.72, "distal reference\nderivative", "#FFF4D6"),
        (6.15, 0.25, 1.50, 0.72, "continuous arc\nrotation", "#FCE8E6"),
        (8.05, 0.75, 1.75, 0.9, "DC motors +\nMuJoCo contacts", "#EDE7F6"),
    ]
    for x0, y0, width0, height0, label, color in boxes:
        ax.add_patch(FancyBboxPatch((x0, y0), width0, height0, boxstyle="round,pad=0.04", facecolor=color, edgecolor="#333333", linewidth=0.8))
        ax.text(x0 + width0 / 2, y0 + height0 / 2, label, ha="center", va="center", fontsize=8)
    arrows = [
        ((1.55, 1.2), (1.95, 1.2)),
        ((3.70, 1.35), (4.10, 1.70)),
        ((3.70, 1.05), (4.10, 0.62)),
        ((5.75, 0.62), (6.15, 0.62)),
        ((5.75, 1.70), (8.05, 1.35)),
        ((7.65, 0.62), (8.05, 1.05)),
    ]
    for start, end in arrows:
        ax.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=10, linewidth=0.9, color="#333333"))
    ax.text(6.90, 0.12, "sum: distal rolling + bounded-gait rate", ha="center",
            fontsize=6.8, color="#555555")
    ax.text(4.95, 2.20, "50 Hz targets", ha="center", fontsize=7, color="#555555")
    ax.text(8.92, 0.43, "500 Hz physics", ha="center", fontsize=7, color="#555555")
    fig.savefig(OUT / "control_architecture.pdf", bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)


if __name__ == "__main__":
    flat_metrics()
    stair_results()
    arc_geometry()
    control_architecture()
