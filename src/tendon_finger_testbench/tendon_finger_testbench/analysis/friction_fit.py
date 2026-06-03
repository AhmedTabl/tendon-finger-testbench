"""Fit a simple Coulomb + viscous friction model from logged data."""

from __future__ import annotations

import argparse
import math
from typing import Dict, List, Sequence, Tuple

from tendon_finger_testbench.analysis.common import as_float, load_rows


def fit_friction(
    rows: Sequence[Dict[str, str]],
    velocity_column: str = "true_finger_velocity",
    torque_column: str = "applied_torque_after_saturation",
    min_velocity: float = 0.02,
) -> Dict[str, float]:
    design: List[Tuple[float, float, float]] = []
    targets: List[float] = []
    for row in rows:
        velocity = as_float(row, velocity_column)
        torque = as_float(row, torque_column)
        if not math.isfinite(velocity) or not math.isfinite(torque):
            continue
        if abs(velocity) < min_velocity:
            continue
        design.append((math.copysign(1.0, velocity), velocity, 1.0))
        targets.append(torque)

    if len(targets) < 3:
        return {}

    normal = [[0.0 for _ in range(3)] for _ in range(3)]
    rhs = [0.0, 0.0, 0.0]
    for row, target in zip(design, targets):
        for i in range(3):
            rhs[i] += row[i] * target
            for j in range(3):
                normal[i][j] += row[i] * row[j]

    coeffs = solve_3x3(normal, rhs)
    predictions = [sum(coeffs[i] * row[i] for i in range(3)) for row in design]
    mean_target = sum(targets) / len(targets)
    ss_tot = sum((target - mean_target) ** 2 for target in targets)
    ss_res = sum((target - pred) ** 2 for target, pred in zip(targets, predictions))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1.0e-12 else float("nan")

    return {
        "coulomb_friction_nm": coeffs[0],
        "viscous_friction_nm_s_per_rad": coeffs[1],
        "torque_offset_nm": coeffs[2],
        "r_squared": r2,
        "sample_count": float(len(targets)),
    }


def solve_3x3(matrix: List[List[float]], rhs: List[float]) -> List[float]:
    augmented = [matrix[i][:] + [rhs[i]] for i in range(3)]
    for pivot in range(3):
        best = max(range(pivot, 3), key=lambda row: abs(augmented[row][pivot]))
        augmented[pivot], augmented[best] = augmented[best], augmented[pivot]
        scale = augmented[pivot][pivot]
        if abs(scale) < 1.0e-12:
            raise ValueError("Singular normal matrix")
        for column in range(pivot, 4):
            augmented[pivot][column] /= scale
        for row in range(3):
            if row == pivot:
                continue
            factor = augmented[row][pivot]
            for column in range(pivot, 4):
                augmented[row][column] -= factor * augmented[pivot][column]
    return [augmented[i][3] for i in range(3)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    parser.add_argument("--velocity-column", default="true_finger_velocity")
    parser.add_argument("--torque-column", default="applied_torque_after_saturation")
    args = parser.parse_args()

    result = fit_friction(
        load_rows(args.csv_path),
        velocity_column=args.velocity_column,
        torque_column=args.torque_column,
    )
    for key, value in result.items():
        print(f"{key}: {value:.6g}")


if __name__ == "__main__":
    main()

