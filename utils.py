"""General utility functions for the robot code."""

import phoenix6 as p6
import wpilib
import wpimath.units as units


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


def setMotorLimits(
    motorController: p6.hardware.TalonFX | p6.hardware.TalonFXS,
    maxSupplyCurrent: units.amperes | None = None,
    maxStatorCurrent: units.amperes | None = None,
) -> None:
    """Set supply and/or stator current limits on a TalonFXS motor controller.

    See https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html

    Args:
        motorController: The TalonFX or TalonFXS motor controller to configure.
        maxSupplyCurrent: The max supply current in amperes, helping to prevent brownouts, or None to leave unchanged.
        maxStatorCurrent: The max stator current in amperes, helping to prevent wheel slip (or None to leave unlimited).
    """
    if maxSupplyCurrent is None and maxStatorCurrent is None:
        return  # nothing to do

    cfg = p6.configs.CurrentLimitsConfigs()
    if maxSupplyCurrent is not None:
        cfg.supply_current_limit_enable = True
        cfg.supply_current_limit = maxSupplyCurrent
    if maxStatorCurrent is not None:
        cfg.stator_current_limit_enable = True
        cfg.stator_current_limit = maxStatorCurrent
    motorController.configurator.apply(cfg)


def setMotorNeutralBrake(
    motorController: p6.hardware.TalonFX | p6.hardware.TalonFXS,
    brake_in_neutral: bool = True,
) -> None:
    """Set motor neutral behavior to BRAKE or COAST.

    See https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/configuration.html

    Args:
        motorController: The TalonFX or TalonFXS motor controller to configure.
        brake_in_neutral: True sets neutral mode to BRAKE. False sets neutral mode to COAST.
    """
    cfg = p6.configs.MotorOutputConfigs()
    cfg.neutral_mode = p6.signals.NeutralModeValue.BRAKE if brake_in_neutral else p6.signals.NeutralModeValue.COAST
    motorController.configurator.apply(cfg)


def setMotorMotionMagic(
    motorController: p6.hardware.TalonFX | p6.hardware.TalonFXS,
    *,
    k_s: float = 0.25,
    k_v: float = 0.12,
    k_a: float = 0.01,
    k_p: float = 4.8,
    k_i: float = 0.0,
    k_d: float = 0.1,
    cruise_velocity: float = 80,
    acceleration: float = 160,
    jerk: float = 1600,
) -> None:
    """Apply Slot 0 and Motion Magic configs using subgroup applies.

    This updates only the Slot0 and MotionMagic config groups, avoiding
    accidental resets of unrelated persistent settings.
    """
    slot0_cfg = p6.configs.Slot0Configs()
    slot0_cfg.k_s = k_s
    slot0_cfg.k_v = k_v
    slot0_cfg.k_a = k_a
    slot0_cfg.k_p = k_p
    slot0_cfg.k_i = k_i
    slot0_cfg.k_d = k_d

    motion_magic_cfg = p6.configs.MotionMagicConfigs()
    motion_magic_cfg.motion_magic_cruise_velocity = cruise_velocity
    motion_magic_cfg.motion_magic_acceleration = acceleration
    motion_magic_cfg.motion_magic_jerk = jerk

    motorController.configurator.apply(slot0_cfg)
    motorController.configurator.apply(motion_magic_cfg)


def scaleColor(color: wpilib.Color8Bit, scale: float) -> wpilib.Color8Bit:
    """Scale a Color8Bit's brightness by a given factor."""
    return wpilib.Color8Bit(
        int(max(0, min(255, color.red * scale))),
        int(max(0, min(255, color.green * scale))),
        int(max(0, min(255, color.blue * scale))),
    )
