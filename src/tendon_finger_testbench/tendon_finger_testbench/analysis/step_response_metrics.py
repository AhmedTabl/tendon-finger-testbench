"""Compute step-response metrics from a logged CSV file."""

from __future__ import annotations

import argparse
import math
from typing import Dict, List, Sequence

from tendon_finger_testbench.analysis.common import (
    as_float,
    crossing_time,
    load_rows,
    mean_tail,
    series,
)


def compute_step_metrics(
    rows: Sequence[Dict[str, str]],
    target: float | None = None,
    signal_column: str = "measured_output_position",
    settling_band: float = 0.02,
) -> Dict[str, float]:
    times = series(rows, "time")
    signal = series(rows, signal_column)
    command = series(rows, "command_position")
    if target is None:
        finite_commands = [value for value in command if math.isfinite(value)]
        target = finite_commands[-1] if finite_commands else mean_tail(signal)

    finite_signal = [value for value in signal if math.isfinite(value)]
    if not finite_signal:
        return {}

    start = finite_signal[0]
    amplitude = target - start
    if abs(amplitude) < 1.0e-12:
        return {
            "target": target,
            "rise_time_s": float("nan"),
            "overshoot_percent": float("nan"),
            "settling_time_s": float("nan"),
            "steady_state_error_rad": target - mean_tail(signal),
            "peak_torque_nm": max_abs(rows, "torque_command"),
            "peak_current_a": max_abs(rows, "current_command"),
        }

    t10 = crossing_time(times, signal, start + 0.1 * amplitude)
    t90 = crossing_time(times, signal, start + 0.9 * amplitude)
    rise_time = t90 - t10 if math.isfinite(t10) and math.isfinite(t90) else float("nan")

    peak = max(finite_signal) if amplitude > 0.0 else min(finite_signal)
    overshoot = (
        (peak - target) / abs(amplitude) * 100.0
        if amplitude > 0.0
        else (target - peak) / abs(amplitude) * 100.0
    )

    tolerance = max(abs(amplitude) * settling_band, 1.0e-4)
    settling_time = float("nan")
    for index, time_value in enumerate(times):
        if not math.isfinite(time_value):
            continue
        tail = [value for value in signal[index:] if math.isfinite(value)]
        if tail and max(abs(value - target) for value in tail) <= tolerance:
            settling_time = time_value
            break

    return {
        "target": target,
        "rise_time_s": rise_time,
        "overshoot_percent": overshoot,
        "settling_time_s": settling_time,
        "steady_state_error_rad": target - mean_tail(signal),
        "peak_torque_nm": max_abs(rows, "torque_command"),
        "peak_current_a": max_abs(rows, "current_command"),
    }


def max_abs(rows: Sequence[Dict[str, str]], key: str) -> float:
    values = [abs(as_float(row, key)) for row in rows]
    finite = [value for value in values if math.isfinite(value)]
    return max(finite) if finite else float("nan")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    parser.add_argument("--target", type=float, default=None)
    parser.add_argument("--signal", default="measured_output_position")
    args = parser.parse_args()

    metrics = compute_step_metrics(load_rows(args.csv_path), args.target, args.signal)
    for key, value in metrics.items():
        print(f"{key}: {value:.6g}" if isinstance(value, float) else f"{key}: {value}")


if __name__ == "__main__":
    main()

