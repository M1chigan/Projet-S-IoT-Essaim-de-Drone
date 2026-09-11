#!/usr/bin/env python3
"""Plot positions, speed norm and error norm from a CSV (pandas).

Usage:
    python Mission3\plot_extracted_grouped.py --csv Mission3\extracted_grouped.csv --outdir Mission3\plots

Produces three PNG files in the output directory:
    - positions.png      (lon vs lat colored by index)
    - speed_norm.png     (speed_norm vs index)
    - error_norm.png     (error norm vs index)
"""
from pathlib import Path
import argparse
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def read_and_prepare(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)

    # Ensure relevant columns are numeric
    for col in ["lat", "lon", "alt", "err_x", "err_y", "err_z", "vx", "vy", "vz", "speed_norm"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    # Create a sequence index to color the trajectory if no explicit 'index' column
    if "index" in df.columns:
        df["seq_index"] = pd.to_numeric(df["index"], errors="coerce")
    else:
        df["seq_index"] = np.arange(len(df))

    # Compute error norm
    if all(c in df.columns for c in ("err_x", "err_y", "err_z")):
        df["err_norm"] = np.sqrt(df.err_x**2 + df.err_y**2 + df.err_z**2)
    else:
        df["err_norm"] = np.nan

    # If speed_norm missing but vx,vy,vz present, compute it
    if "speed_norm" not in df.columns or df["speed_norm"].isna().all():
        if all(c in df.columns for c in ("vx", "vy", "vz")):
            df["speed_norm"] = np.sqrt(df.vx**2 + df.vy**2 + df.vz**2)

    # Fill seq_index for plotting (ensures finite values)
    if df["seq_index"].isna().any():
        df["seq_index"] = np.arange(len(df))

    return df


def plot_positions(df: pd.DataFrame, outpath: Path):
    plt.figure(figsize=(8, 6))
    x = df["lon"]
    y = df["lat"]
    c = df["seq_index"]
    sc = plt.scatter(x, y, c=c, cmap="viridis", s=25, edgecolor="none")
    plt.colorbar(sc, label="index")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Positions (lon vs lat) colored by index")
    plt.gca().set_aspect("equal", adjustable="datalim")
    plt.grid(True, linestyle=':', alpha=0.5)
    out = outpath / "positions.png"
    plt.tight_layout()
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"Saved positions plot to: {out}")


def plot_speed_norm(df: pd.DataFrame, outpath: Path):
    plt.figure(figsize=(8, 4))
    idx = df["seq_index"]
    plt.plot(idx, df["speed_norm"], marker=".", linewidth=1)
    plt.xlabel("index")
    plt.ylabel("speed_norm")
    plt.title("Speed norm vs index")
    plt.grid(True, linestyle=':', alpha=0.5)
    out = outpath / "speed_norm.png"
    plt.tight_layout()
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"Saved speed norm plot to: {out}")


def plot_err_norm(df: pd.DataFrame, outpath: Path):
    plt.figure(figsize=(8, 4))
    idx = df["seq_index"]
    plt.plot(idx, df["err_norm"], marker=".", linewidth=1, color="tab:red")
    plt.xlabel("index")
    plt.ylabel("err_norm")
    plt.title("Error norm vs index")
    plt.grid(True, linestyle=':', alpha=0.5)
    out = outpath / "error_norm.png"
    plt.tight_layout()
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"Saved error norm plot to: {out}")


def main():
    parser = argparse.ArgumentParser(description="Plot positions, speed norm and error norm from CSV.")
    parser.add_argument("--csv", required=True, help="Path to CSV file")
    parser.add_argument("--outdir", default='plots', help="Output directory to save plots (defaults to CSV folder)")
    parser.add_argument("--show", action="store_true", help="Show plots interactively (useful when running locally)")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"CSV file not found: {csv_path}")

    outdir = Path(args.outdir) if args.outdir else csv_path.parent
    outdir.mkdir(parents=True, exist_ok=True)

    df = read_and_prepare(csv_path)

    # Basic checks
    if not ("lat" in df.columns and "lon" in df.columns):
        raise SystemExit("CSV must contain 'lat' and 'lon' columns for position plotting.")

    plot_positions(df, outdir)
    if "speed_norm" in df.columns:
        plot_speed_norm(df, outdir)
    else:
        print("No 'speed_norm' column found (or couldn't compute). Skipping speed plot.")

    if "err_norm" in df.columns:
        plot_err_norm(df, outdir)
    else:
        print("No error components found to compute 'err_norm'. Skipping error plot.")

    if args.show:
        print("Note: '--show' was requested but script saves plotted PNGs instead of showing them interactively.")


if __name__ == "__main__":
    main()
