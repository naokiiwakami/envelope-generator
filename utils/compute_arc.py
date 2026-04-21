#!/usr/bin/env python3

import math
from typing import Tuple

Point = Tuple[float, float]


def distance(p1: Point, p2: Point) -> float:
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def angle_deg(origin: Point, p: Point) -> float:
    dx = p[0] - origin[0]
    dy = p[1] - origin[1]

    angle = math.degrees(math.atan2(dy, dx))
    if angle < 0:
        angle += 360
    return angle


def tangent_angles(origin: Point, center: Point, radius: float):
    """
    Returns (min_angle, max_angle) that define the forbidden cone
    where the arc would intersect the circle.
    """
    d = distance(origin, center)

    if d <= radius:
        raise ValueError("Origin is inside or on the circle")

    theta = angle_deg(origin, center)
    alpha = math.degrees(math.asin(radius / d))

    return theta - alpha, theta + alpha


def normalize_angle(a):
    return a % 360


def compute_arc(c0: Point, d0: float, c1: Point, d1: float, origin: Point):
    r0 = d0 / 2
    r1 = d1 / 2

    # Tangent exclusion zones
    t0_min, t0_max = tangent_angles(origin, c0, r0)
    t1_min, t1_max = tangent_angles(origin, c1, r1)

    # Center directions
    a0 = angle_deg(origin, c0)
    a1 = angle_deg(origin, c1)

    # Normalize
    a0 = normalize_angle(a0)
    a1 = normalize_angle(a1)

    # Ensure clockwise ordering
    if a1 < a0:
        a1 += 360

    # Clip against forbidden zones
    start_angle = a0
    end_angle = a1

    # Push start forward if inside circle 0 forbidden zone
    if t0_min <= start_angle <= t0_max:
        start_angle = t0_max

    # Push end backward if inside circle 1 forbidden zone
    if t1_min <= end_angle <= t1_max:
        end_angle = t1_min

    # Radius: choose something safe (minimum distance to tangent)
    d0_center = distance(origin, c0)
    d1_center = distance(origin, c1)

    radius = min(
        math.sqrt(d0_center**2 - r0**2),
        math.sqrt(d1_center**2 - r1**2),
    )

    return {
        "radius": radius,
        "start_angle": normalize_angle(start_angle),
        "end_angle": normalize_angle(end_angle),
        "center_angle_0": a0 % 360,
        "center_angle_1": a1 % 360,
    }


# Example
if __name__ == "__main__":
    c0 = (9, 25)
    d0 = 20

    c1 = (119, 25)
    d1 = 20

    origin = (64, 160)

    result = compute_arc(c0, d0, c1, d1, origin)

    print(result)
