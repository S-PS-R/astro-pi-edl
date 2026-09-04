"""PNG from a telemetry CSV."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

from .log import DEFAULT_LOG_PATH


def plot_log(path: str | Path = DEFAULT_LOG_PATH, out: str | Path | None = None) -> Path:
    src = Path(path)
    if not src.exists():
        raise FileNotFoundError(src)
    dest = Path(out) if out is not None else src.with_suffix(".png")

    t: list[float] = []
    mag: list[float] = []
    ax: list[float] = []
    ay: list[float] = []
    az: list[float] = []
    temp: list[float] = []
    pres: list[float] = []
    td: list[float] = []
    with src.open(newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        mag_key = "a_mag_g" if reader.fieldnames and "a_mag_g" in reader.fieldnames else "mag_g"
        for row in reader:
            t.append(float(row["t_s"]))
            mag.append(float(row[mag_key]))
            ax.append(float(row["ax"]))
            ay.append(float(row["ay"]))
            az.append(float(row["az"]))
            temp.append(float(row["T_C"]))
            pres.append(float(row["P_mbar"]))
            td.append(float(row["touchdown"]))
    if not t:
        raise ValueError(f"{src} is empty")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t, mag, color="black", label="|a|")
    axes[0].plot(t, ax, alpha=0.6, label="ax")
    axes[0].plot(t, ay, alpha=0.6, label="ay")
    axes[0].plot(t, az, alpha=0.6, label="az")
    for ti, flag in zip(t, td):
        if flag:
            axes[0].axvline(ti, color="red", alpha=0.15, linewidth=1)
    axes[0].set_ylabel("IMU (g)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, temp, color="tab:orange")
    axes[1].set_ylabel("T (C)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, pres, color="tab:blue")
    axes[2].set_ylabel("P (mbar)")
    axes[2].set_xlabel("t (s)")
    axes[2].grid(True, alpha=0.3)

    fig.suptitle(src.name)
    fig.tight_layout()
    fig.savefig(dest, dpi=120)
    plt.close(fig)
    return dest


def main(argv: list[str] | None = None) -> None:
    args = list(sys.argv[1:] if argv is None else argv)
    print(f"wrote {plot_log(args[0] if args else DEFAULT_LOG_PATH)}")


if __name__ == "__main__":
    main()
