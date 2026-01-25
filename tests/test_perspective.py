"""Integration-ish tests for alliance perspective and drive modes.

These use DriverStationSim and the real Drivetrain to verify request routing.
"""

import time

import hal
import pytest
import wpilib
from phoenix6.swerve.requests import FieldCentric, ForwardPerspectiveValue
from wpilib.simulation import DriverStationSim
from wpimath.geometry import Pose2d, Rotation2d

import constants as const
import robot
from components.swerve import Drivetrain

TICK = 0.02  # 20 ms


@pytest.fixture(autouse=True)
def enable_robot():
    """Enable sim driver station so motor outputs are honored."""
    DriverStationSim.setEnabled(True)
    DriverStationSim.notifyNewData()
    time.sleep(0.02)  # let sim settle
    yield
    DriverStationSim.setEnabled(False)
    DriverStationSim.notifyNewData()


@pytest.fixture
def bot() -> robot.Scurvy:
    """Provide a robot instance with a real drivetrain in sim."""
    b = robot.Scurvy()
    b.drivetrain = Drivetrain()
    return b


def _exercise_drive(
    bot: robot.Scurvy,
    alliance: hal.AllianceStationID,
    centric: str,
    cmd_vx: float,
    cmd_vy: float,
    exp_vx: float,
    exp_vy: float,
    monkeypatch: pytest.MonkeyPatch,
):
    """Common helper to set alliance, drive, and assert request contents.

    centric: "operator" or "field" chooses drive() vs drive_field_centric().
    exp_vx/exp_vy are the expected values present on the request object.
    """
    captured_rot = {}

    def capture_rot(rot):
        captured_rot["rot"] = rot

    monkeypatch.setattr(bot.drivetrain._drivetrain, "set_operator_perspective_forward", capture_rot)

    DriverStationSim.setAllianceStationId(alliance)
    DriverStationSim.notifyNewData()
    bot.maybe_set_operator_perspective()

    bot.drivetrain.reset_pose(Pose2d(0, 0, Rotation2d(0)))

    if centric == "operator":
        bot.drivetrain.drive(velocity_x=cmd_vx, velocity_y=cmd_vy, rotation_rate=0.0)
    else:
        bot.drivetrain.drive_field_centric(velocity_x=cmd_vx, velocity_y=cmd_vy, rotation_rate=0.0)

    req = bot.drivetrain._pending_request
    assert isinstance(req, FieldCentric)
    if centric == "operator":
        assert req.forward_perspective == ForwardPerspectiveValue.OPERATOR_PERSPECTIVE
    else:
        assert req.forward_perspective == ForwardPerspectiveValue.BLUE_ALLIANCE

    assert req.velocity_x == pytest.approx(exp_vx)
    assert req.velocity_y == pytest.approx(exp_vy)

    # ensure operator forward was set according to alliance
    if alliance == hal.AllianceStationID.kRed1:
        assert captured_rot["rot"].radians() == pytest.approx(
            const.ALLIANCE_PERSPECTIVE_ROTATION[wpilib.DriverStation.Alliance.kRed].radians()
        )
    else:
        assert captured_rot["rot"].radians() == pytest.approx(
            const.ALLIANCE_PERSPECTIVE_ROTATION[wpilib.DriverStation.Alliance.kBlue].radians()
        )

    # Flush pending to mirror the robot loop; this ensures the request would be sent
    bot.drivetrain.execute()


def test_operator_perspective_blue(bot: robot.Scurvy, monkeypatch: pytest.MonkeyPatch):
    """Operator-centric on blue should align with field (no inversion)."""
    _exercise_drive(bot, hal.AllianceStationID.kBlue1, "operator", 1.0, 2.0, 1.0, 2.0, monkeypatch)


def test_operator_perspective_red(bot: robot.Scurvy, monkeypatch: pytest.MonkeyPatch):
    """Operator-centric on red should be 180° rotated vs field (inverted x/y)."""
    _exercise_drive(bot, hal.AllianceStationID.kRed1, "operator", 1.0, 2.0, 1.0, 2.0, monkeypatch)


def test_field_centric_blue(bot: robot.Scurvy, monkeypatch: pytest.MonkeyPatch):
    """Field-centric +X/+Y on blue stays in field frame."""
    _exercise_drive(bot, hal.AllianceStationID.kBlue1, "field", 1.0, 2.0, 1.0, 2.0, monkeypatch)


def test_field_centric_red(bot: robot.Scurvy, monkeypatch: pytest.MonkeyPatch):
    """Field-centric +X/+Y on red stays in field frame (no inversion)."""
    _exercise_drive(bot, hal.AllianceStationID.kRed1, "field", 1.0, 2.0, 1.0, 2.0, monkeypatch)
