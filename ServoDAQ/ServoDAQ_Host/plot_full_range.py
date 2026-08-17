"""
plot_full_range.py

Single combined-axis view of a plot_naive_vs_smart.py run: naive,
smart-coarse, and smart-fine all on one plot spanning the servo's whole
tested pulse range (instead of split into separate low/high subplots),
with clear markers at find_range()'s own reported min/max pulse -- the
actual numbers the algorithm decided on, not just where the raw traces
happen to end.

Reads the most recent naive_low/high, smart_coarse_low/high,
smart_fine_low/high, and summary CSVs already saved under ../data/ by
plot_naive_vs_smart.py -- no hardware access, just a different view of
data already captured in one connection/session (so the centideg
reference is consistent across all of it).

Usage: python plot_full_range.py [STAMP]
  STAMP: e.g. 20260817-000142. Defaults to the most recent set found.
"""

import csv
import glob
import os
import re
import sys

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")


def load_csv(path):
    with open(path, newline="") as f:
        r = csv.reader(f)
        next(r)
        return [(int(a), int(b)) for a, b in r]


def find_stamp():
    matches = sorted(glob.glob(os.path.join(DATA_DIR, "summary_*.csv")))
    if not matches:
        raise FileNotFoundError(f"no summary_*.csv found in {DATA_DIR}")
    m = re.search(r"summary_(.+)\.csv$", matches[-1])
    return m.group(1)


def main():
    stamp = sys.argv[1] if len(sys.argv) > 1 else find_stamp()
    print(f"using stamp: {stamp}")

    def p(name):
        return os.path.join(DATA_DIR, f"{name}_{stamp}.csv")

    naive_low = load_csv(p("naive_low"))
    naive_high = load_csv(p("naive_high"))
    coarse_low = load_csv(p("smart_coarse_low"))
    coarse_high = load_csv(p("smart_coarse_high"))
    fine_low = load_csv(p("smart_fine_low"))
    fine_high = load_csv(p("smart_fine_high"))

    summary = {}
    with open(p("summary"), newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            summary[(row["method"], row["side"])] = (int(row["pulse_us"]), int(row["centideg"]))

    min_pulse, min_centideg = summary[("smart", "low")]
    max_pulse, max_centideg = summary[("smart", "high")]
    print(f"smart min: {min_pulse}us ({min_centideg} centideg)")
    print(f"smart max: {max_pulse}us ({max_centideg} centideg)")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    NAIVE_COLOR, COARSE_COLOR, FINE_COLOR = "#1f77b4", "#2ca02c", "#d62728"

    fig, ax = plt.subplots(figsize=(13, 7))

    # Naive and coarse each span the whole tested range as two segments
    # (low sweep, high sweep) of the same series -- label only the first
    # so the legend doesn't duplicate.
    ax.plot(*zip(*naive_low), "o-", color=NAIVE_COLOR, markersize=3, label="naive")
    ax.plot(*zip(*naive_high), "o-", color=NAIVE_COLOR, markersize=3)
    ax.plot(*zip(*coarse_low), "o-", color=COARSE_COLOR, markersize=4, label="smart: coarse")
    ax.plot(*zip(*coarse_high), "o-", color=COARSE_COLOR, markersize=4)
    ax.plot(*zip(*fine_low), "o-", color=FINE_COLOR, markersize=4, label="smart: fine")
    ax.plot(*zip(*fine_high), "o-", color=FINE_COLOR, markersize=4)

    # Clear markers at find_range()'s own reported min/max -- the actual
    # decision, not just wherever the raw traces happen to stop.
    for pulse, centideg, label in [(min_pulse, min_centideg, "min"), (max_pulse, max_centideg, "max")]:
        ax.plot(pulse, centideg, marker="*", markersize=26, color="black",
                 markerfacecolor="gold", markeredgewidth=1.5, zorder=5)
        ax.annotate(
            f"{label}\n{pulse}us\n{centideg} centideg",
            xy=(pulse, centideg), xytext=(0, 22 if label == "min" else -55),
            textcoords="offset points", ha="center", fontsize=9, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="black", alpha=0.9),
        )

    ax.set_xlabel("pulse (us)")
    ax.set_ylabel("position (centideg)")
    ax.set_title(f"Full range: naive vs smart coarse/fine, with find_range()'s reported min/max\n(stamp {stamp})")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    plot_path = os.path.join(DATA_DIR, f"full_range_{stamp}.png")
    fig.savefig(plot_path, dpi=150)
    print(f"saved: {plot_path}")


if __name__ == "__main__":
    main()
