"""Intake for the robot.

This component controls a single motorized intake used to pick up and release game pieces (fuel).
It follows the same simple style as the other MagicBot components in this project: tunable speeds,
simple commands (ingest, release, stop) and an `execute` method which is called each control loop
to apply the desired motor output.
"""

import math
from typing import Literal

import magicbot
import phoenix6 as p6
import phoenix6.units as p6_units
import wpimath.units as units

import constants as const


class Intake:
    """A simple intake subsystem.

    Attributes injected by MagicBot:
    - `intakeMotor`: the motor controller (e.g. `wpilib.Talon`) used for the
      intake rollers.

    Usage:
    - Call `ingest()` to run the rollers inward and pick up fuel.
    - Call `release()` to run the rollers outward and eject fuel.
    - Call `stop()` to stop the rollers.
    """

    # Motors+ injected by MagicBot when the robot sets an attribute of the same name on the robot class.
    intakeMotorExtendFore: p6.hardware.TalonFX
    intakeMotorExtendAft: p6.hardware.TalonFX
    intakeMotorIntake: p6.hardware.TalonFX
    transitMotor: p6.hardware.TalonFX
    runIntake: bool = False
    """Set to True to extend the intake fully and run the intake rollers; False to stop the intake and retract."""

    activelyTransit: bool = False
    reverseIntake: bool = False
    intakeCANCoder: p6.hardware.CANcoder

    # Tunable speeds (can be adjusted at runtime via NetworkTables)
    intakeSpeed = magicbot.tunable(-1)  # negative: pick up; positive: vomit
    transitSpeed = magicbot.tunable(0.75)
    intakeExtendSpeed = magicbot.tunable(0.45)
    intakeRetractSpeed = magicbot.tunable(-0.45)

    # Calibrated CANCoder readings at extension limit (in sensor rotations).
    # Keep this as internal calibration constant; operators tune physical limits in meters.
    _RETRACTED_CANCODER_ROTATIONS = 0.889850959

    # Linear extension setpoints (meters).
    intakeRetractedMeters = magicbot.tunable(units.inchesToMeters(1))  # Physically stops around 0.0
    intakeExtendedMeters = magicbot.tunable(units.inchesToMeters(10))  # Physically stops around 11.875
    # How close to the target extension we need to be to consider it "close enough" for control purposes
    intakeToleranceMeters = magicbot.tunable(units.inchesToMeters(0.75))

    def __init__(self) -> None:
        """Initialize internal state."""
        # human friendly state string for telemetry/debugging
        self._extendState: Literal["extend", "retract", "hold"] = "hold"

        # Runtime calibration offset for the CANCoder's retracted reference point.
        self._retractedCancoderRotations = self._RETRACTED_CANCODER_ROTATIONS

        self.runIntake = False
        self.activelyTransit = False
        """Set to True to run the transit mechanism; False to stop it."""

        self._position: p6.StatusSignal[p6_units.rotation] | None = None

    def setup(self) -> None:
        """Configure CANcoder.

        We intentionally do not call set_position(0) here, as this is an absolute encoder.
        """
        magnet_sensor = p6.configs.MagnetSensorConfigs()
        config = p6.configs.CANcoderConfiguration().with_magnet_sensor(magnet_sensor)
        status = self.intakeCANCoder.configurator.apply(config)
        if status.is_error():
            print(f"Intake CANcoder config failed: {status.name}: {status.description}")
            return

        self._position = self.intakeCANCoder.get_position(False)
        self._position.set_update_frequency(50.0)

    @magicbot.feedback
    def extensionPosition(self) -> units.meters:
        """Linear extension position in meters."""
        if self._position is None:
            return 0
        self._position.refresh()
        encoder_rotations = float(self._position.value) - self._retractedCancoderRotations
        pinion_rotations = -encoder_rotations * const.RobotDimension.INTAKE_PINION_TO_ENCODER_RATIO
        return pinion_rotations * math.pi * const.RobotDimension.INTAKE_EXTENSION_GEAR_DIAMETER

    def calibrateFullyExtendedNow(self) -> None:
        """Treat the current CANcoder reading as the fully-extended position.

        This updates the internal calibration offset used by `extensionPosition()`.
        """
        if self._position is None:
            return

        self._position.refresh()
        encoder_rotations = float(self._position.value)

        meters_per_encoder_rotation = (
            -const.RobotDimension.INTAKE_PINION_TO_ENCODER_RATIO
            * math.pi
            * const.RobotDimension.INTAKE_EXTENSION_GEAR_DIAMETER
        )

        if meters_per_encoder_rotation == 0:
            return

        # Solve for retracted reference so current reading maps to intakeExtendedMeters.
        self._retractedCancoderRotations = encoder_rotations - (self.intakeExtendedMeters / meters_per_encoder_rotation)
        print(
            "Intake calibration captured. "
            f"Use this in code: _RETRACTED_CANCODER_ROTATIONS = {self._retractedCancoderRotations:.9f}"
        )

    def ingest(self) -> None:
        """Start the intake to pick up fuel."""
        self.runIntake = True
        self.reverseIntake = False

    def vomit(self) -> None:
        """Run the intake in reverse to release fuel."""
        self.runIntake = True
        self.reverseIntake = True

    def stop(self) -> None:
        """Stop the intake rollers."""
        self.runIntake = False

    def extend(self) -> None:
        """Lower the intake mechanism."""
        self._extendState = "extend"

    def retract(self) -> None:
        """Raise the intake mechanism."""
        self._extendState = "retract"

    def isFullyExtended(self) -> bool:
        """Return True if the intake is fully lowered to the field."""
        return self.extensionPosition() >= (self.intakeExtendedMeters - self.intakeToleranceMeters)

    def isFullyRetracted(self) -> bool:
        """Return True if the intake is fully raised to the robot."""
        return self.extensionPosition() <= (self.intakeRetractedMeters + self.intakeToleranceMeters)

    def execute(self) -> None:
        """Called each loop to command the motor."""
        if self.runIntake:
            self.intakeMotorIntake.set(self.intakeSpeed if not self.reverseIntake else -self.intakeSpeed)
            if not self.isFullyExtended():
                self.extend()
        else:
            self.transitMotor.set(0)
            self.intakeMotorIntake.set(0)

        if self.activelyTransit:
            self.transitMotor.set(self.transitSpeed if not self.reverseIntake else -self.transitSpeed)
        else:
            self.transitMotor.set(0)

        if self._extendState == "extend":
            if not self.isFullyExtended():
                self.intakeMotorExtendFore.set(-self.intakeExtendSpeed)
                self.intakeMotorExtendAft.set(self.intakeExtendSpeed)
            else:
                self._extendState = "hold"
        elif self._extendState == "retract":
            if not self.isFullyRetracted():
                self.intakeMotorExtendFore.set(-self.intakeRetractSpeed)
                self.intakeMotorExtendAft.set(self.intakeRetractSpeed)
            else:
                self._extendState = "hold"

        if self._extendState == "hold":
            self.intakeMotorExtendFore.set(0)
            self.intakeMotorExtendAft.set(0)
