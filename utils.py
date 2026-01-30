"""General utility functions for the robot code."""

import wpilib


def color8FromHSV(h: float, s: float, v: float) -> wpilib.Color8Bit:
    """Convert HSV to Color8Bit with full precision.

    I'd love to use wpilib.Color.fromHSV, but only returns 12-bit colors, not 24-bit colors.

    Args:
        h: Hue 0-360
        s: Saturation 0-1
        v: Value 0-1

    Returns:
        Color8Bit with 8-bit RGB values (24 bits total)
    """
    h %= 360
    c = v * s
    x = c * (1 - abs((h / 60) % 2 - 1))
    m = v - c

    if h < 60:
        r, g, b = c, x, 0
    elif h < 120:
        r, g, b = x, c, 0
    elif h < 180:
        r, g, b = 0, c, x
    elif h < 240:
        r, g, b = 0, x, c
    elif h < 300:
        r, g, b = x, 0, c
    else:
        r, g, b = c, 0, x

    return wpilib.Color8Bit(int((r + m) * 255), int((g + m) * 255), int((b + m) * 255))
