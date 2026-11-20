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
    scores, widths, contacts = data["scores"], data["widths"], data["contacts"]

    rng = np.random.default_rng(0)
    sample = points[rng.choice(len(points), size=min(6000, len(points)), replace=False)]

    fig = plt.figure(figsize=(12, 7.5), facecolor=BG)
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
    for rank, idx in enumerate(order):
        color = cmap(0.15 + 0.85 * (scores[idx] - score_min) /
                     max(score_max - score_min, 1e-6))
        pose, rot = poses[idx], poses[idx][:3, :3]
        origin, x_axis, z_axis = pose[:3, 3], rot[:, 0], rot[:, 2]
        contact, width = contacts[idx], float(widths[idx])

        p1 = contact - 0.5 * width * x_axis
        p2 = contact + 0.5 * width * x_axis
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color=color, lw=2.4)
        ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2],
                  length=0.055, color=color, linewidth=1.6, arrow_length_ratio=0.25)
        ax.scatter([contact[0]], [contact[1]], [contact[2]], color=color, s=22,
                   depthshade=False)
        ax.text(origin[0], origin[1], origin[2], f" {rank + 1}", color=color, fontsize=8)

    origins = poses[order][:, :3, 3]
    mins = np.minimum(sample.min(axis=0), origins.min(axis=0)) - 0.015
    maxs = np.maximum(sample.max(axis=0), origins.max(axis=0)) + 0.015
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
    fig.savefig(path, dpi=150, facecolor=BG)
    plt.close(fig)


def _banner(plot_path: Path, out_path: Path, top: int, caption: str, count: int) -> None:
    plot = Image.open(plot_path).convert("RGB")
    width, height = 1800, 620
    banner = Image.new("RGB", (width, height), BG)
    draw = ImageDraw.Draw(banner)

    plot_height = height - 40
    plot_width = int(plot.width * plot_height / plot.height)
    plot = plot.resize((plot_width, plot_height), Image.Resampling.LANCZOS)
    banner.paste(plot, (width - plot_width - 20, 20))

    font_dir = Path(matplotlib.get_data_path()) / "fonts" / "ttf"
    title = ImageFont.truetype(str(font_dir / "DejaVuSans-Bold.ttf"), 62)
    sub = ImageFont.truetype(str(font_dir / "DejaVuSans.ttf"), 27)
    small = ImageFont.truetype(str(font_dir / "DejaVuSans.ttf"), 21)

    left, y = 70, 120
    draw.text((left, y), "Contact-GraspNet", font=title, fill="#f7fafc")
    draw.text((left, y + 78), "ROS 2", font=title, fill="#4fd1c5")
    draw.text((left, y + 178), "Depth + mask + intrinsics", font=sub, fill="#c7d2e0")
    draw.text((left, y + 216), "→ 6-DoF grasp proposals", font=sub, fill="#c7d2e0")

    for offset, label in ((0, "ROS 2 · Jazzy"), (268, "Zensical docs")):
        draw.rounded_rectangle(
            (left + offset, y + 290, left + offset + 250, y + 334),
            radius=10, fill="#16233c", outline="#2c3e5e", width=2,
        )
        draw.text((left + offset + 18, y + 298), label, font=small, fill="#9fb3c8")

    draw.text(
        (left, height - 78),
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
