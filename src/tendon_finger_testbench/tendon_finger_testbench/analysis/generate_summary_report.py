"""Generate a Markdown summary report from one logged CSV file."""

from __future__ import annotations

import argparse
from pathlib import Path

from tendon_finger_testbench.analysis.backlash_analysis import estimate_backlash
from tendon_finger_testbench.analysis.common import load_rows
from tendon_finger_testbench.analysis.friction_fit import fit_friction
from tendon_finger_testbench.analysis.step_response_metrics import compute_step_metrics


def generate_report(csv_path: str, output: str | None = None, make_plot: bool = True) -> str:
    rows = load_rows(csv_path)
    if output is None:
        output = str(Path(csv_path).with_suffix(".md"))

    plot_path = None
    if make_plot:
        try:
            from tendon_finger_testbench.analysis.plot_test import plot

            plot_path = plot(csv_path)
        except Exception as exc:
            plot_path = f"Plot skipped: {exc}"

    step = compute_step_metrics(rows)
    backlash = estimate_backlash(rows)
    friction = fit_friction(rows)

    lines = [
        "# Tendon Finger Test Summary",
        "",
        f"Source CSV: `{csv_path}`",
        f"Samples: {len(rows)}",
        "",
        "## Step Response",
    ]
    lines.extend(format_dict(step))
    lines.extend(["", "## Backlash"])
    lines.extend(format_dict(backlash))
    lines.extend(["", "## Friction Fit"])
    lines.extend(format_dict(friction))
    if plot_path:
        lines.extend(["", "## Plot", str(plot_path)])

    with open(output, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    return output


def format_dict(values: dict) -> list[str]:
    if not values:
        return ["No estimate available."]
    return [f"- {key}: {value:.6g}" for key, value in values.items()]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    output = generate_report(args.csv_path, args.output, make_plot=not args.no_plot)
    print(f"Wrote report to {output}")


if __name__ == "__main__":
    main()

