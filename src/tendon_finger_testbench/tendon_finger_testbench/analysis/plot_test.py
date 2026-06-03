"""Plot command, measured/true state, torque/current, error, and temperature."""

from __future__ import annotations

import argparse
from pathlib import Path

from tendon_finger_testbench.analysis.common import load_rows, series


def plot(csv_path: str, output: str | None = None) -> str | None:
    import matplotlib.pyplot as plt

    rows = load_rows(csv_path)
    time = series(rows, "time")

    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(10, 12))
    axes[0].plot(time, series(rows, "command_position"), label="command")
    axes[0].plot(time, series(rows, "measured_output_position"), label="measured output")
    axes[0].plot(time, series(rows, "true_finger_position"), label="true finger")
    axes[0].set_ylabel("position [rad]")
    axes[0].legend()

    axes[1].plot(time, series(rows, "true_motor_velocity"), label="motor")
    axes[1].plot(time, series(rows, "true_finger_velocity"), label="finger")
    axes[1].set_ylabel("velocity [rad/s]")
    axes[1].legend()

    axes[2].plot(time, series(rows, "torque_command"), label="torque command")
    axes[2].plot(time, series(rows, "applied_torque_after_saturation"), label="applied")
    axes[2].set_ylabel("torque [N m]")
    axes[2].legend()

    axes[3].plot(time, series(rows, "controller_error_position"), label="position error")
    axes[3].plot(time, series(rows, "controller_error_velocity"), label="velocity error")
    axes[3].set_ylabel("error")
    axes[3].legend()

    axes[4].plot(time, series(rows, "temperature"), label="temperature")
    axes[4].set_ylabel("temperature [C]")
    axes[4].set_xlabel("time [s]")
    axes[4].legend()

    fig.tight_layout()
    if output is None:
        output = str(Path(csv_path).with_suffix(".png"))
    fig.savefig(output, dpi=160)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    parser.add_argument("--output", default=None)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    output = plot(args.csv_path, args.output)
    print(f"Saved plot to {output}")
    if args.show:
        import matplotlib.pyplot as plt

        plt.show()


if __name__ == "__main__":
    main()

