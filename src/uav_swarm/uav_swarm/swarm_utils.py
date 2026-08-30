import math
from typing import Iterable, List


Vector = List[float]


def vec3(value: Iterable[float]) -> Vector:
    data = [float(x) for x in value]
    if len(data) != 3:
        raise ValueError("expected a 3D vector")
    return data


def vec3_list(flat_values: Iterable[float]) -> List[Vector]:
    data = [float(x) for x in flat_values]
    if len(data) % 3 != 0:
        raise ValueError("expected flat 3D vector list with length divisible by 3")
    return [data[i : i + 3] for i in range(0, len(data), 3)]


def obstacle_list(flat_values: Iterable[float]) -> List[List[float]]:
    data = [float(x) for x in flat_values]
    if len(data) % 4 != 0:
        raise ValueError("expected flat obstacle list as x, y, z, radius tuples")
    return [data[i : i + 4] for i in range(0, len(data), 4)]


def dynamic_obstacle_list(flat_values: Iterable[float]) -> List[List[float]]:
    data = [float(x) for x in flat_values]
    if len(data) % 8 != 0:
        raise ValueError("expected flat dynamic obstacle list as x, y, z, radius, vx, vy, vz, range tuples")
    return [data[i : i + 8] for i in range(0, len(data), 8)]


def add(a: Vector, b: Vector) -> Vector:
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def sub(a: Vector, b: Vector) -> Vector:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def scale(a: Vector, gain: float) -> Vector:
    return [a[0] * gain, a[1] * gain, a[2] * gain]


def norm(a: Vector) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def clamp_norm(a: Vector, limit: float) -> Vector:
    magnitude = norm(a)
    if limit <= 0.0 or magnitude <= limit or magnitude < 1.0e-9:
        return a
    return scale(a, limit / magnitude)


def yaw_to_quaternion(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def yaw_from_velocity(velocity: Vector, fallback: float = 0.0) -> float:
    if abs(velocity[0]) + abs(velocity[1]) < 1.0e-6:
        return fallback
    return math.atan2(velocity[1], velocity[0])


def rmse(values: List[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))
