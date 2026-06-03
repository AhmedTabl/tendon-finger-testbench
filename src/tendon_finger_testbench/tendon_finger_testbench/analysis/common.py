"""Common CSV helpers for analysis scripts."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


Row = Dict[str, str]


def load_rows(path: str) -> List[Row]:
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def as_float(row: Row, key: str, default: float = float("nan")) -> float:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def series(rows: Sequence[Row], key: str) -> List[float]:
    return [as_float(row, key) for row in rows]


def finite_xy(rows: Sequence[Row], x_key: str, y_key: str) -> Tuple[List[float], List[float]]:
    xs: List[float] = []
    ys: List[float] = []
    for row in rows:
        x = as_float(row, x_key)
        y = as_float(row, y_key)
        if math.isfinite(x) and math.isfinite(y):
            xs.append(x)
            ys.append(y)
    return xs, ys


def latest_csv(directory: str = "data") -> str:
    paths = sorted(Path(directory).glob("*.csv"), key=lambda path: path.stat().st_mtime)
    if not paths:
        raise FileNotFoundError(f"No CSV files found under {directory!r}")
    return str(paths[-1])


def mean_tail(values: Sequence[float], fraction: float = 0.1) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return float("nan")
    count = max(1, int(len(finite) * fraction))
    return sum(finite[-count:]) / count


def crossing_time(times: Sequence[float], values: Sequence[float], threshold: float) -> float:
    for index in range(1, min(len(times), len(values))):
        t0, t1 = times[index - 1], times[index]
        y0, y1 = values[index - 1], values[index]
        if not all(math.isfinite(value) for value in (t0, t1, y0, y1)):
            continue
        if (y0 <= threshold <= y1) or (y1 <= threshold <= y0):
            if abs(y1 - y0) < 1.0e-12:
                return t1
            alpha = (threshold - y0) / (y1 - y0)
            return t0 + alpha * (t1 - t0)
    return float("nan")

