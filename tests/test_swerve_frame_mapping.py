"""Regression tests for the 2026 shooter-front swerve frame mapping."""

import pytest
from wpimath.units import inchesToMeters

from generated.tuner_constants import TunerConstants


def test_logical_corners_match_expected_physical_modules() -> None:
    """Lock module-ID remapping for shooter-front orientation."""
    assert TunerConstants._front_left_drive_motor_id == 6
    assert TunerConstants._front_left_steer_motor_id == 4
    assert TunerConstants._front_left_encoder_id == 5
    assert TunerConstants._front_left_encoder_offset == pytest.approx(-0.37744140625)

    assert TunerConstants._front_right_drive_motor_id == 12
    assert TunerConstants._front_right_steer_motor_id == 10
    assert TunerConstants._front_right_encoder_id == 11
    assert TunerConstants._front_right_encoder_offset == pytest.approx(0.004638671875)

    assert TunerConstants._back_left_drive_motor_id == 2
    assert TunerConstants._back_left_steer_motor_id == 0
    assert TunerConstants._back_left_encoder_id == 1
    assert TunerConstants._back_left_encoder_offset == pytest.approx(0.136962890625)

    assert TunerConstants._back_right_drive_motor_id == 18
    assert TunerConstants._back_right_steer_motor_id == 16
    assert TunerConstants._back_right_encoder_id == 17
    assert TunerConstants._back_right_encoder_offset == pytest.approx(0.466552734375)


def test_logical_corner_positions_match_clockwise_90deg_rotation() -> None:
    """Validate x'=-y, y'=x frame rotation for new shooter-front geometry."""
    assert TunerConstants._front_left_x_pos == pytest.approx(inchesToMeters(9.8125))
    assert TunerConstants._front_left_y_pos == pytest.approx(inchesToMeters(12.3125))

    assert TunerConstants._front_right_x_pos == pytest.approx(inchesToMeters(9.8125))
    assert TunerConstants._front_right_y_pos == pytest.approx(inchesToMeters(-12.3125))

    assert TunerConstants._back_left_x_pos == pytest.approx(inchesToMeters(-9.8125))
    assert TunerConstants._back_left_y_pos == pytest.approx(inchesToMeters(12.3125))

    assert TunerConstants._back_right_x_pos == pytest.approx(inchesToMeters(-9.8125))
    assert TunerConstants._back_right_y_pos == pytest.approx(inchesToMeters(-12.3125))
