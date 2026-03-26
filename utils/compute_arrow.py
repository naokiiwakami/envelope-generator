#!/usr/bin/env python3

import math
from typing import Tuple, Dict

Point = Tuple[float, float]


def normalize(vx: float, vy: float):
    length = math.hypot(vx, vy)
    if length == 0:
        raise ValueError("Zero-length line")
    return vx / length, vy / length


def compute_arrow_head(
    start: Point,
    end: Point,
    length: float = 5.0,
    width: float = 4.0,
) -> Dict[str, Point]:
    """
    Returns triangle vertices for arrow head at 'end'.

    length: arrow head length (pixels)
    width: base width (pixels)
    """

    x0, y0 = start
    x1, y1 = end

    # Direction
    dx = x1 - x0
    dy = y1 - y0

    ux, uy = normalize(dx, dy)

    # Perpendicular (screen coords: +y downward)
    px, py = -uy, ux

    # Tip
    tip = (x1, y1)

    # Base center
    bx = x1 - ux * length
    by = y1 - uy * length

    # Base corners
    half_w = width / 2

    left = (bx + px * half_w, by + py * half_w)
    right = (bx - px * half_w, by - py * half_w)

    return {
        "tip": tip,
        "left": left,
        "right": right,
    }


# Example
if __name__ == "__main__":
    start = (45, 16)
    end = (105, 47)

    arrow = compute_arrow_head(start, end, length=6, width=4)

    print("Tip:", arrow["tip"])
    print("Left:", arrow["left"])
    print("Right:", arrow["right"])