#!/usr/bin/env python3
"""Render a 3-D grasp plot from ``demo_from_mesh.py`` and a README/docs banner.

Requires ``matplotlib`` and ``Pillow`` (``pip install -r docs-requirements.txt``).

Example:
    python scripts/render_banner.py --input /tmp/grasps.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ASSETS = REPO_ROOT / "docs" / "assets"
BG = "#0b1020"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("/tmp/grasps.npz"))
    parser.add_argument("--assets", type=Path, default=DEFAULT_ASSETS)
    parser.add_argument("--top", type=int, default=10)
    parser.add_argument("--caption", default="three.js Lucy100k.ply")
    return parser.parse_args()


def _plot(data: np.lib.npyio.NpzFile, top: int, caption: str, path: Path) -> None:
    points, poses = data["points"], data["poses"]
    scores, widths = data["scores"], data["widths"]

    rng = np.random.default_rng(0)
    sample = points[rng.choice(len(points), size=min(6000, len(points)), replace=False)]

    fig = plt.figure(figsize=(14, 9), facecolor=BG)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor(BG)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.set_facecolor("#111930")
        pane.set_edgecolor("#24304a")

    ax.scatter(
        sample[:, 0], sample[:, 1], sample[:, 2],
        s=1.5, c="#4fd1c5", alpha=0.5, linewidths=0,
    )

    order = np.argsort(scores)[::-1][:top]
    cmap = plt.get_cmap("plasma")
    score_min, score_max = float(scores.min()), float(scores.max())
    drawn: list[np.ndarray] = []
    for rank, idx in enumerate(order):
        color = cmap(0.15 + 0.85 * (scores[idx] - score_min) /
                     max(score_max - score_min, 1e-6))
        pose, rot = poses[idx], poses[idx][:3, :3]
        origin, x_axis, z_axis = pose[:3, 3], rot[:, 0], rot[:, 2]
        width = float(widths[idx])
        half = 0.5 * width

        # Parallel-jaw gripper: two fingers, a base bar and the wrist stem.
        # +Z is the approach axis; the finger tips lie at the contact plane
        # (0.1034 m in front of the gripper-base origin).
        tip = origin + 0.1034 * z_axis
        back = origin + 0.045 * z_axis
        stem = origin - 0.05 * z_axis
        left_tip, right_tip = tip - half * x_axis, tip + half * x_axis
        left_back, right_back = back - half * x_axis, back + half * x_axis

        for start, end in (
            (left_back, left_tip),      # left finger: | 
            (right_back, right_tip),    # right finger:    |
            (left_back, right_back),    # base bar: ------
            (back, stem),               # wrist stem:   |
        ):
            ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]],
                    color=color, lw=2.2)
            drawn.extend((start, end))

        ax.scatter([left_tip[0], right_tip[0]],
                   [left_tip[1], right_tip[1]],
                   [left_tip[2], right_tip[2]],
                   color=color, s=16, depthshade=False)
        ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2],
                  length=0.03, color=color, linewidth=1.2, arrow_length_ratio=0.3)
        ax.text(origin[0], origin[1], origin[2], f" {rank + 1}", color=color, fontsize=8)

    drawn_points = np.vstack(drawn) if drawn else poses[order][:, :3, 3]
    mins = np.minimum(sample.min(axis=0), drawn_points.min(axis=0)) - 0.015
    maxs = np.maximum(sample.max(axis=0), drawn_points.max(axis=0)) + 0.015
    ax.set_xlim(mins[0], maxs[0])
    ax.set_ylim(mins[1], maxs[1])
    ax.set_zlim(mins[2], maxs[2])
    ax.set_box_aspect(tuple(maxs - mins))
    ax.view_init(elev=17, azim=-58)
    ax.set_xlabel("x (right)", color="#c7d2e0", labelpad=6)
    ax.set_ylabel("y (down)", color="#c7d2e0", labelpad=6)
    ax.set_zlabel("z (forward)", color="#c7d2e0", labelpad=6)
    ax.tick_params(colors="#7c8ba1", labelsize=8)
    ax.grid(color="#24304a", alpha=0.5)
    ax.set_title(f"Top {top} grasps on a downloaded mesh ({caption})",
                 color="#e6edf7", fontsize=13, pad=12)

    fig.subplots_adjust(left=0.03, right=0.98, bottom=0.05, top=0.90)
    fig.savefig(path, dpi=180, facecolor=BG)
    plt.close(fig)


def _banner(plot_path: Path, out_path: Path, top: int, caption: str, count: int) -> None:
    plot = Image.open(plot_path).convert("RGB")
    width, height = 2200, 820
    banner = Image.new("RGB", (width, height), BG)
    draw = ImageDraw.Draw(banner)

    plot_height = height - 40
    plot_width = int(plot.width * plot_height / plot.height)
    plot = plot.resize((plot_width, plot_height), Image.Resampling.LANCZOS)
    banner.paste(plot, (width - plot_width - 20, 20))

    font_dir = Path(matplotlib.get_data_path()) / "fonts" / "ttf"
    title = ImageFont.truetype(str(font_dir / "DejaVuSans-Bold.ttf"), 76)
    sub = ImageFont.truetype(str(font_dir / "DejaVuSans.ttf"), 32)
    small = ImageFont.truetype(str(font_dir / "DejaVuSans.ttf"), 24)

    left, y = 80, 150
    draw.text((left, y), "Contact-GraspNet", font=title, fill="#f7fafc")
    draw.text((left, y + 96), "ROS 2", font=title, fill="#4fd1c5")
    draw.text((left, y + 220), "Depth + mask + intrinsics", font=sub, fill="#c7d2e0")
    draw.text((left, y + 266), "→ 6-DoF grasp proposals", font=sub, fill="#c7d2e0")

    for offset, label in ((0, "ROS 2 · Jazzy"), (318, "Zensical docs")):
        draw.rounded_rectangle(
            (left + offset, y + 350, left + offset + 290, y + 402),
            radius=10, fill="#16233c", outline="#2c3e5e", width=2,
        )
        draw.text((left + offset + 20, y + 360), label, font=small, fill="#9fb3c8")

    draw.text(
        (left, height - 90),
        f"Sample: {caption} · {count} proposals · top {top} shown",
        font=small, fill="#7c8ba1",
    )
    banner.save(out_path, optimize=True)


def main() -> None:
    args = _parse_args()
    args.assets.mkdir(parents=True, exist_ok=True)
    with np.load(args.input) as data:
        _plot(data, args.top, args.caption, args.assets / "grasps-3d.png")
        _banner(
            args.assets / "grasps-3d.png",
            args.assets / "banner.png",
            args.top,
            args.caption,
            len(data["scores"]),
        )
    print(f"wrote {args.assets / 'grasps-3d.png'}")
    print(f"wrote {args.assets / 'banner.png'}")


if __name__ == "__main__":
    main()
