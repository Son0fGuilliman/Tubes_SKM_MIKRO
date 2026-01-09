#!/usr/bin/env python3
import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class Sample:
    t: int
    temp: float
    setpoint: float
    fan: int


@dataclass(frozen=True)
class ScenarioMeta:
    name: str
    kp: str | None
    ki: str | None
    kd: str | None


@dataclass(frozen=True)
class Metrics:
    scenario: ScenarioMeta
    t0: int
    temp0: float
    setpoint: float
    step: float
    td: float | None
    tr: float | None
    tp: float | None
    ts: float | None
    undershoot_pct: float
    steady_state_error: float


def iter_lines(path: Path) -> Iterable[str]:
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            yield raw.strip()


def parse_scenario_meta(path: Path) -> ScenarioMeta:
    name = path.stem
    kp = ki = kd = None
    last_line: str | None = None
    for line in iter_lines(path):
        if not line.startswith("#SCENARIO,"):
            continue
        last_line = line

    if last_line is not None:
        parts = [p.strip() for p in last_line.split(",")]
        if len(parts) >= 2 and parts[1]:
            name = parts[1]
        for p in parts[2:]:
            if p.startswith("Kp="):
                kp = p.removeprefix("Kp=")
            elif p.startswith("Ki="):
                ki = p.removeprefix("Ki=")
            elif p.startswith("Kd="):
                kd = p.removeprefix("Kd=")
    return ScenarioMeta(name=name, kp=kp, ki=ki, kd=kd)


def parse_samples(path: Path) -> list[Sample]:
    segments: list[list[Sample]] = [[]]
    last_t: int | None = None
    for line in iter_lines(path):
        if not line or line.startswith("#") or line.startswith("---"):
            continue
        if line.lower().startswith("t,temp,setpoint,fan"):
            segments.append([])
            last_t = None
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 4:
            continue
        try:
            t = int(parts[0])
            temp = float(parts[1])
            setpoint = float(parts[2])
            fan = int(float(parts[3]))
        except ValueError:
            continue
        if temp < -100.0:
            continue
        if last_t is not None and t < last_t:
            continue
        last_t = t
        segments[-1].append(Sample(t=t, temp=temp, setpoint=setpoint, fan=fan))

    for seg in reversed(segments):
        if seg:
            return seg
    return []


def first_index(pred: Iterable[bool]) -> int | None:
    for idx, ok in enumerate(pred):
        if ok:
            return idx
    return None


def compute_metrics(
    samples: list[Sample],
    meta: ScenarioMeta,
    *,
    settle_window_sec: int,
    steady_window_sec: int,
) -> Metrics:
    if not samples:
        raise ValueError("no samples")

    setpoint = samples[-1].setpoint

    start_idx = first_index(s.fan > 0 for s in samples)
    if start_idx is None:
        start_idx = 0

    t_start = samples[start_idx].t
    temp0 = samples[start_idx].temp

    step = temp0 - setpoint
    if step <= 0.0:
        raise ValueError(
            f"invalid step: temp0={temp0:.2f} <= setpoint={setpoint:.2f} (need start above setpoint)"
        )

    td_level = setpoint + 0.98 * step
    t10_level = setpoint + 0.90 * step
    t90_level = setpoint + 0.10 * step

    def time_to_reach(level: float) -> float | None:
        idx = first_index(s.temp <= level for s in samples[start_idx:])
        if idx is None:
            return None
        return float(samples[start_idx + idx].t - t_start)

    td = time_to_reach(td_level)
    t10 = time_to_reach(t10_level)
    t90 = time_to_reach(t90_level)
    tr = None if (t10 is None or t90 is None) else (t90 - t10)

    min_idx_rel = min(range(start_idx, len(samples)), key=lambda i: samples[i].temp)
    t_min = samples[min_idx_rel].t
    temp_min = samples[min_idx_rel].temp
    tp = float(t_min - t_start)

    undershoot_pct = max(0.0, (setpoint - temp_min) / step * 100.0)

    tol = 0.02 * step
    window = max(1, settle_window_sec)
    ts: float | None = None
    for i in range(start_idx, len(samples) - window + 1):
        ok = True
        for j in range(i, i + window):
            if abs(samples[j].temp - setpoint) > tol:
                ok = False
                break
        if ok:
            ts = float(samples[i].t - t_start)
            break

    steady_n = max(1, steady_window_sec)
    tail = samples[-steady_n:] if len(samples) >= steady_n else samples
    steady_state_error = sum(s.temp - setpoint for s in tail) / float(len(tail))

    return Metrics(
        scenario=meta,
        t0=t_start,
        temp0=temp0,
        setpoint=setpoint,
        step=step,
        td=td,
        tr=tr,
        tp=tp,
        ts=ts,
        undershoot_pct=undershoot_pct,
        steady_state_error=steady_state_error,
    )


def fmt_optional(value: float | None) -> str:
    if value is None:
        return "—"
    return f"{value:.0f}"


def print_markdown_table(metrics: list[Metrics]) -> None:
    print("| Skenario | (Kp, Ki, Kd) | Td (s) | Tr (s) | Tp (s) | Ts (s) | Undershoot (%) | Steady State Error (°C) |")
    print("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for m in metrics:
        kp = m.scenario.kp or "?"
        ki = m.scenario.ki or "?"
        kd = m.scenario.kd or "?"
        print(
            f"| {m.scenario.name} | ({kp}, {ki}, {kd}) | {fmt_optional(m.td)} | {fmt_optional(m.tr)} | "
            f"{m.tp:.0f} | {fmt_optional(m.ts)} | {m.undershoot_pct:.1f} | {m.steady_state_error:.2f} |"
        )


def print_text(metrics: list[Metrics]) -> None:
    for m in metrics:
        kp = m.scenario.kp or "?"
        ki = m.scenario.ki or "?"
        kd = m.scenario.kd or "?"
        print(
            f"{m.scenario.name}: Kp={kp} Ki={ki} Kd={kd}  "
            f"Td={fmt_optional(m.td)}s Tr={fmt_optional(m.tr)}s Tp={m.tp:.0f}s Ts={fmt_optional(m.ts)}s  "
            f"undershoot={m.undershoot_pct:.1f}%  SSE={m.steady_state_error:.2f}°C"
        )


def plot_temperature(out_png: Path, scenarios: list[tuple[ScenarioMeta, list[Sample]]]) -> None:
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception as e:
        raise SystemExit(f"matplotlib not available: {e}")

    plt.figure(figsize=(8, 4.5))
    for meta, samples in scenarios:
        if not samples:
            continue
        t0 = samples[0].t
        xs = [s.t - t0 for s in samples]
        ys = [s.temp for s in samples]
        plt.plot(xs, ys, label=meta.name)

    if scenarios and scenarios[0][1]:
        sp = scenarios[0][1][-1].setpoint
        plt.axhline(sp, color="k", linestyle="--", linewidth=1, alpha=0.6, label="Setpoint")

    plt.grid(True, alpha=0.3)
    plt.xlabel("Waktu (s)")
    plt.ylabel("Temperatur (°C)")
    plt.title("Step Response (Temperatur vs Waktu)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze step response logs (format: t,temp,setpoint,fan)."
    )
    parser.add_argument("inputs", nargs="+", type=Path, help="one or more log files")
    parser.add_argument(
        "--report-table",
        action="store_true",
        help="print a Markdown table for Bab 3.2 (default output)",
    )
    parser.add_argument(
        "--text",
        action="store_true",
        help="print a compact text summary instead of Markdown",
    )
    parser.add_argument(
        "--plot",
        type=Path,
        default=None,
        help="save overlay temperature plot PNG (requires matplotlib)",
    )
    parser.add_argument(
        "--settle-window-sec",
        type=int,
        default=10,
        help="settling window length in seconds (default: 10)",
    )
    parser.add_argument(
        "--steady-window-sec",
        type=int,
        default=10,
        help="steady-state averaging window in seconds (default: 10)",
    )

    args = parser.parse_args()
    if args.settle_window_sec <= 0:
        raise SystemExit("--settle-window-sec must be >= 1")

    all_metrics: list[Metrics] = []
    plot_series: list[tuple[ScenarioMeta, list[Sample]]] = []

    for path in args.inputs:
        meta = parse_scenario_meta(path)
        samples = parse_samples(path)
        if not samples:
            raise SystemExit(f"{path}: no samples found")
        plot_series.append((meta, samples))
        try:
            m = compute_metrics(
                samples,
                meta,
                settle_window_sec=args.settle_window_sec,
                steady_window_sec=args.steady_window_sec,
            )
        except ValueError as e:
            raise SystemExit(f"{path}: {e}")
        all_metrics.append(m)

    if args.text:
        print_text(all_metrics)
    else:
        print_markdown_table(all_metrics)

    if args.plot is not None:
        plot_temperature(args.plot, plot_series)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
