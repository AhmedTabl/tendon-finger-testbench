"""Small ROS 2 helper functions for the testbench nodes."""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, Mapping, Optional

from sensor_msgs.msg import JointState


def declare_parameters(node: Any, defaults: Mapping[str, Any]) -> None:
    for name, value in defaults.items():
        node.declare_parameter(name, value)


def read_parameters(node: Any, defaults: Mapping[str, Any]) -> Dict[str, Any]:
    values: Dict[str, Any] = {}
    for name in defaults:
        values[name] = node.get_parameter(name).value
    return values


def parameters_to_dict(parameters: Iterable[Any]) -> Dict[str, Any]:
    return {parameter.name: parameter.value for parameter in parameters}


def joint_value(
    msg: JointState,
    joint_name: str,
    field: str,
    default: float = float("nan"),
) -> float:
    if joint_name not in msg.name:
        return default
    index = msg.name.index(joint_name)
    values = getattr(msg, field)
    if index >= len(values):
        return default
    value = values[index]
    return value if math.isfinite(value) else default


def joint_by_index(
    msg: JointState,
    index: int,
    field: str,
    default: float = float("nan"),
) -> float:
    values = getattr(msg, field)
    if index >= len(values):
        return default
    value = values[index]
    return value if math.isfinite(value) else default

