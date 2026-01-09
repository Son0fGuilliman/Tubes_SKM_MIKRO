#!/usr/bin/env python3
import argparse
import csv
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Sample:
    t: int
    temp: float
    duty: int
    step: int


@dataclass(frozen=True)
class StepSummary:
    step: int
    duty: int
    n: int
    t0: int
    t1: int
    duration_sec: int
    temp0: float
    temp1: float
    delta_t: float
    rate_c_per_min: float


def parse_samples(path: Path) -> list[Sample]:
    samples: list[Sample] = []

    with path.open("r", encoding="utf-8", errors="replace", newline="") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                continue

            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 4:
                continue

            try:
                t = int(parts[0])
                temp = float(parts[1])
                duty = int(parts[2])
                step = int(parts[3])
            except ValueError:
                # Skip header or malformed lines
                continue

            samples.append(Sample(t=t, temp=temp, duty=duty, step=step))

    return samples


def summarize(samples: list[Sample]) -> list[StepSummary]:
    by_step: dict[int, list[Sample]] = {}
    for s in samples:
        if s.step < 0 or s.step > 200:
            continue
        if s.step == 255:
            continue
        by_step.setdefault(s.step, []).append(s)

    summaries: list[StepSummary] = []
    for step in sorted(by_step.keys()):
        items = by_step[step]
        items.sort(key=lambda s: s.t)

        t0 = items[0].t
        t1 = items[-1].t
        n = len(items)
        duty = items[0].duty
        temp0 = items[0].temp
        temp1 = items[-1].temp
        delta_t = temp0 - temp1

        duration_sec = max(1, (t1 - t0 + 1))
        rate_c_per_min = delta_t / (duration_sec / 60.0)

        summaries.append(
            StepSummary(
                step=step,
                duty=duty,
                n=n,
                t0=t0,
                t1=t1,
                duration_sec=duration_sec,
                temp0=temp0,
                temp1=temp1,
                delta_t=delta_t,
                rate_c_per_min=rate_c_per_min,
            )
        )

    return summaries


def write_csv(path: Path, summaries: list[StepSummary]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "step",
                "duty_percent",
                "n_samples",
                "t0_sec",
                "t1_sec",
                "duration_sec",
                "temp0_c",
                "temp1_c",
                "delta_t_c",
                "rate_c_per_min",
            ]
        )
        for s in summaries:
            w.writerow(
                [
                    s.step,
                    s.duty,
                    s.n,
                    s.t0,
                    s.t1,
                    s.duration_sec,
                    f"{s.temp0:.3f}",
                    f"{s.temp1:.3f}",
                    f"{s.delta_t:.3f}",
                    f"{s.rate_c_per_min:.3f}",
                ]
            )


def print_markdown(summaries: list[StepSummary]) -> None:
    print(
        "| Step | Duty (%) | N | t0 (s) | t1 (s) | Durasi (s) | T_awal (°C) | T_akhir (°C) | ΔT (°C) | Laju (°C/menit) |"
    )
    print("| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for s in summaries:
        print(
            f"| {s.step} | {s.duty} | {s.n} | {s.t0} | {s.t1} | {s.duration_sec} | "
            f"{s.temp0:.2f} | {s.temp1:.2f} | {s.delta_t:.2f} | {s.rate_c_per_min:.2f} |"
        )


def print_report_table(summaries: list[StepSummary]) -> None:
    print("| Duty (%) | $T_{awal}$ (°C) | $T_{akhir}$ (°C) | Durasi (s) | $\\Delta T$ (°C) | Laju (°C/menit) |")
    print("| ---: | ---: | ---: | ---: | ---: | ---: |")
    for s in summaries:
        print(
            f"| {s.duty} | {s.temp0:.2f} | {s.temp1:.2f} | {s.duration_sec} | {s.delta_t:.2f} | {s.rate_c_per_min:.2f} |"
        )


def print_text(summaries: list[StepSummary]) -> None:
    for s in summaries:
        duration = s.t1 - s.t0 + 1
        print(
            f"step={s.step} duty={s.duty:3d}%  n={s.n:4d}  "
            f"t={s.t0:5d}..{s.t1:5d} ({duration:4d}s)  "
            f"T={s.temp0:6.2f}→{s.temp1:6.2f}  "
            f"ΔT={s.delta_t:6.2f}  rate={s.rate_c_per_min:6.2f} °C/min"
        )


def maybe_plot(out_png: Path, summaries: list[StepSummary]) -> None:
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception as e:
        raise SystemExit(f"matplotlib not available: {e}")

    duties = [s.duty for s in summaries]
    rates = [s.rate_c_per_min for s in summaries]

    plt.figure(figsize=(7, 4))
    plt.plot(duties, rates, marker="o")
    plt.grid(True, alpha=0.3)
    plt.xlabel("Duty PWM (%)")
    plt.ylabel("Laju pendinginan (°C/menit)")
    plt.title("Karakteristik Open-Loop: Duty vs Laju Pendinginan")
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Summarize open-loop sweep logs (format: t,temp,duty,step)."
    )
    parser.add_argument("input", type=Path, help="log file captured from UART")
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="write summary CSV to this path",
    )
    parser.add_argument(
        "--markdown",
        action="store_true",
        help="print a Markdown table (for laporan Bab 3.1)",
    )
    parser.add_argument(
        "--report-table",
        action="store_true",
        help="print a simplified Markdown table matching the Bab 3.1 template",
    )
    parser.add_argument(
        "--plot",
        type=Path,
        default=None,
        help="save plot PNG (requires matplotlib)",
    )

    args = parser.parse_args()

    samples = parse_samples(args.input)
    if not samples:
        raise SystemExit("no samples found (check log format: t,temp,duty,step)")

    summaries = summarize(samples)
    if not summaries:
        raise SystemExit("no step summaries found (did you run OPEN_LOOP_SWEEP_TEST?)")

    if args.report_table:
        print_report_table(summaries)
    elif args.markdown:
        print_markdown(summaries)
    else:
        print_text(summaries)

    if args.out is not None:
        write_csv(args.out, summaries)

    if args.plot is not None:
        maybe_plot(args.plot, summaries)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
