"""KPI report figure (matplotlib, static PNG)."""

from __future__ import annotations

import os

SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]   # categorical slots 1-3
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def _style(ax, title: str) -> None:
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", color=INK, fontsize=11, fontweight="bold")
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=9)


def _lines(ax, t, series: dict[str, list]) -> None:
    for (label, ys), color in zip(series.items(), SERIES):
        ax.plot(t, ys, color=color, linewidth=2, label=label)
        ax.annotate(f"{label} {ys[-1]}", (t[-1], ys[-1]), xytext=(4, 0), textcoords="offset points",
                    color=INK2, fontsize=8, va="center")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK2, loc="upper left")
    ax.set_xlabel("simulated time (s)", color=INK2, fontsize=9)


def plot_kpis(kpis: dict, timeline: list[dict], out_dir: str) -> str:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6), facecolor=SURFACE)
    t = [r["t"] for r in timeline]

    _style(axes[0], "Completed tasks (cumulative)")
    _lines(axes[0], t, {"rack picks": [r["picks"] for r in timeline],
                        "pallet transfers": [r["pallets"] for r in timeline]})

    _style(axes[1], "Task backlog")
    _lines(axes[1], t, {"pending": [r["pending"] for r in timeline],
                        "in progress": [r["active"] for r in timeline]})

    ax = axes[2]
    _style(ax, "Robot utilization (share of time on a task)")
    names = list(kpis["robots"])
    util = [kpis["robots"][n]["utilization"] * 100 for n in names]
    bars = ax.bar(names, util, color=SERIES[0], width=0.6, edgecolor=SURFACE, linewidth=2)
    for b, n in zip(bars, names):
        ax.annotate(f"{kpis['robots'][n]['tasks']}", (b.get_x() + b.get_width() / 2, b.get_height()),
                    xytext=(0, 3), textcoords="offset points", ha="center", color=INK2, fontsize=8)
    ax.set_ylim(0, 105)
    ax.set_ylabel("% of time (label = tasks done)", color=INK2, fontsize=9)
    ax.tick_params(axis="x", rotation=45)

    fig.suptitle(f"AGV/AMR warehouse - {kpis['picks_per_hour']:.0f} picks/h, "
                 f"{kpis['pallets_per_hour']:.0f} pallets/h, {kpis['collisions']} collisions",
                 x=0.01, ha="left", color=INK, fontsize=13)
    fig.tight_layout()
    path = os.path.join(out_dir, "kpis.png")
    fig.savefig(path, dpi=120, facecolor=SURFACE)
    plt.close(fig)
    return path
