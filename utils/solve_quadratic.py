#!/usr/bin/env python3

def solve_coefficients(R: float):
    """
    Solve coefficients a, b, c for:
        y = ax^2 + bx + c

    such that:
        x=0 -> y=1/R
        x=1 -> y=1
        x=2 -> y=R
    """

    c = 1.0 / R

    # From equations:
    # a + b + c = 1
    # 4a + 2b + c = R

    # Solve:
    # a + b = 1 - c
    # 2a + b = (R - c) / 2

    a = (R + c - 2.0) / 2.0
    b = (1.0 - c) - a

    return a, b, c


def verify(R: float):
    a, b, c = solve_coefficients(R)

    print(f"R = {R}")
    print(f"a = {a}")
    print(f"b = {b}")
    print(f"c = {c}")
    print()

    # Check constraints
    print("Verification:")
    print(f"x=0 -> {c} (expected {1/R})")
    print(f"x=1 -> {a + b + c} (expected 1)")
    print(f"x=2 -> {4*a + 2*b + c} (expected {R})")


if __name__ == "__main__":
    verify(4.0)