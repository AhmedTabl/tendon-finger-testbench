"""Estimate backlash/deadband from reversal data."""

from __future__ import annotations

import argparse
import math
from statistics import mean
from typing import Dict, List, Sequence

from tendon_finger_testbench.analysis.common import as_float, load_rows


def estimate_backlash(
    rows: Sequence[Dict[str, str]],
    command_column: str = "command_position",
    output_column: str = "measured_output_position",
    motor_column: str = "measured_motor_encoder_position",
    output_threshold_rad: float = 0.01,
    finger_length_m: float = 0.09,
) -> Dict[str, float]:
    if len(rows) < 3:
        return {}

    command_estimates: List[float] = []
    motor_estimates: List[float] = []
    last_direction = 0.0
    reversal_command = None
    reversal_motor = None
    reversal_output = None

    previous_command = as_float(rows[0], command_column)
    for row in rows[1:]:
        command = as_float(row, command_column)
        output = as_float(row, output_column)
        motor = as_float(row, motor_column)
        if not math.isfinite(command):
            continue

        delta = command - previous_command if math.isfinite(previous_command) else 0.0
        direction = math.copysign(1.0, delta) if abs(delta) > 1.0e-5 else last_direction
        if last_direction and direction != last_direction:
            reversal_command = command
            reversal_motor = motor
            reversal_output = output

        if reversal_command is not None and math.isfinite(output) and math.isfinite(reversal_output):
            if abs(output - reversal_output) >= output_threshold_rad:
                command_estimates.append(abs(command - reversal_command))
                if math.isfinite(motor) and math.isfinite(reversal_motor):
                    motor_estimates.append(abs(motor - reversal_motor))
                reversal_command = None
                reversal_motor = None
                reversal_output = None

        last_direction = direction
        previous_command = command

    command_backlash = mean(command_estimates) if command_estimates else float("nan")
    motor_backlash = mean(motor_estimates) if motor_estimates else float("nan")
    fingertip_mm = command_backlash * finger_length_m * 1000.0
    return {
        "command_side_backlash_rad": command_backlash,
        "motor_side_backlash_rad": motor_backlash,
        "fingertip_equivalent_backlash_mm": fingertip_mm,
        "reversal_count": float(len(command_estimates)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    parser.add_argument("--finger-length-m", type=float, default=0.09)
    parser.add_argument("--threshold-rad", type=float, default=0.01)
    args = parser.parse_args()

    result = estimate_backlash(
        load_rows(args.csv_path),
        output_threshold_rad=args.threshold_rad,
        finger_length_m=args.finger_length_m,
    )
    for key, value in result.items():
        print(f"{key}: {value:.6g}")


if __name__ == "__main__":
    main()

