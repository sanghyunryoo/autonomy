#!/usr/bin/env python3
import argparse
import csv
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description="Visualize a height map CSV exported by the autonomy DDS logger")
    parser.add_argument("csv_path", help="Path to height_map.csv")
    parser.add_argument("--width", type=int, default=12, help="Grid width for reshaping the height map")
    parser.add_argument("--height", type=int, default=12, help="Grid height for reshaping the height map")
    parser.add_argument("--mode", choices=["latest", "mean"], default="latest", help="Use the latest sample or mean across all samples")
    parser.add_argument("--output", default=None, help="Output image path (defaults to <csv_path>.png)")
    parser.add_argument("--dpi", type=int, default=150, help="Image DPI")
    return parser.parse_args()


def load_matrix(csv_path, width, height, mode):
    rows = []
    with open(csv_path, "r", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        header = next(reader, None)
        if header is None:
            raise ValueError("CSV file is empty")
        for row in reader:
            if not row:
                continue
            if len(row) < 2:
                continue
            try:
                values = [float(v) for v in row[1:]]
            except ValueError as exc:
                raise ValueError(f"Non-numeric value found in {csv_path}: {row}") from exc
            rows.append(values)

    if not rows:
        raise ValueError(f"No data rows found in {csv_path}")

    data = np.array(rows, dtype=float)
    if data.shape[1] != width * height:
        raise ValueError(
            f"Expected {width * height} values per row, but found {data.shape[1]} values. "
            "Use --width/--height to match the exported grid size."
        )

    if mode == "latest":
        sample = data[-1]
    else:
        sample = data.mean(axis=0)

    return sample.reshape(height, width)


def main():
    args = parse_args()
    csv_path = Path(args.csv_path).expanduser().resolve()
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    matrix = load_matrix(csv_path, args.width, args.height, args.mode)

    output_path = Path(args.output).expanduser().resolve() if args.output else csv_path.with_suffix(".png")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(6, 6), dpi=args.dpi)
    image = ax.imshow(matrix, cmap="viridis", origin="lower")
    fig.colorbar(image, ax=ax, shrink=0.9, pad=0.04)
    ax.set_title(f"Height map ({args.mode})")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_xticks([])
    ax.set_yticks([])
    plt.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved visualization to {output_path}")


if __name__ == "__main__":
    main()
