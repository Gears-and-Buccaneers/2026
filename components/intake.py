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
    - `setPower()` allows manual control with a value in [-1.0, 1.0].
    """

    # Motors+ injected by MagicBot when the robot sets an attribute of the same name on the robot class.
    intakeMotorExtendFore: p6.hardware.TalonFX
    intakeMotorExtendAft: p6.hardware.TalonFX
    intakeMotorIntake: p6.hardware.TalonFXS
    transitMotor: p6.hardware.TalonFX
    activelyIntake: bool = False
    activelyTransit: bool = False
    intakeCANCoder: p6.hardware.CANcoder

    # Tunable speeds (can be adjusted at runtime via NetworkTables)
    intakeSpeed = magicbot.tunable(-1)  # negative: pick up
    releaseSpeed = magicbot.tunable(0.8)  # positive: release
    transitSpeed = magicbot.tunable(0.75)
    intakeExtendSpeed = magicbot.tunable(0.3)
    intakeRetractSpeed = magicbot.tunable(-0.3)

    # Calibrated CANCoder readings at extension limit (in sensor rotations).
    # Keep this as internal calibration constant; operators tune physical limits in meters.
    _RETRACTED_CANCODER_ROTATIONS = 0.089355469

    # Linear extension setpoints (meters).
    intakeRetractedMeters = magicbot.tunable(units.inchesToMeters(1))  # Physically stops around 0.0
    intakeExtendedMeters = magicbot.tunable(units.inchesToMeters(10))  # Physically stops around 11.875
    # How close to the target extension we need to be to consider it "close enough" for control purposes
    intakeToleranceMeters = magicbot.tunable(units.inchesToMeters(0.75))

    def __init__(self) -> None:
        """Initialize internal state."""
        # _power is the desired motor output [-1.0, 1.0]
        self._power: float = 0.0
        # human friendly state string for telemetry/debugging
        self._state: str = "stopped"
        self._extendState: Literal["extend", "retract", "hold"] = "hold"

        self.activelyIntake = False
        """Set to True to extend the intake fully and run the intake rollers; False to stop the intake and retract."""

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
        encoder_rotations = float(self._position.value) - self._RETRACTED_CANCODER_ROTATIONS
        pinion_rotations = -encoder_rotations * const.RobotDimension.INTAKE_PINION_TO_ENCODER_RATIO
        return pinion_rotations * math.pi * const.RobotDimension.INTAKE_EXTENSION_GEAR_DIAMETER

    def ingest(self, speed: float | None = None) -> None:
        """Start the intake to pick up fuel.

        Args:
                speed: optional manual speed override in [-1.0, 1.0]. If omitted
                           the configured `intakeSpeed` is used.
        """
        self._power = self.intakeSpeed if speed is None else float(max(-1.0, min(1.0, speed)))
        self._state = "intake"

    def release(self, speed: float | None = None) -> None:
        """Run the intake in reverse to release fuel.

        Args:
                speed: optional manual speed override in [-1.0, 1.0]. If omitted
                           the configured `releaseSpeed` is used.
        """
        self._power = self.releaseSpeed if speed is None else float(max(-1.0, min(1.0, speed)))
        self._state = "release"

    def stop(self) -> None:
        """Stop the intake rollers."""
        self._power = 0.0
        self._state = "stopped"

    def extend(self) -> None:
        """Lower the intake mechanism."""
        self._extendState = "extend"

    def retract(self) -> None:
        """Raise the intake mechanism."""
        self._extendState = "retract"

    def manuallyExtend(self) -> None:
        """Hack method to manually control the intake extension."""
        self._manualState = "extend"

    def manuallyRetract(self) -> None:
        """Hack method to manually control the intake retraction."""
        self._manualState = "retract"

    def manuallyHold(self) -> None:
        """Hack method to manually control the intake retraction."""
        self._manualState = "hold"

    def _setPower(self, power: float) -> None:
        """Directly set motor output. Clips to [-1.0, 1.0]."""
        self._power = float(max(-1.0, min(1.0, power)))
        self._state = "manual" if self._power != 0.0 else "stopped"

    def isRunning(self) -> bool:
        """Return True when the intake is applying non-zero output."""
        return abs(self._power) > 1e-6

    def isFullyExtended(self) -> bool:
        """Return True if the intake is fully lowered to the field."""
        return self.extensionPosition() >= (self.intakeExtendedMeters - self.intakeToleranceMeters)

    def isFullyRetracted(self) -> bool:
        """Return True if the intake is fully raised to the robot."""
        return self.extensionPosition() <= (self.intakeRetractedMeters + self.intakeToleranceMeters)

    def execute(self) -> None:
        """Called each loop to command the motor."""
        if self.activelyIntake:
            # self.intakeMotorIntake.set(self._power)
            # self.intakeMotorIntake.set(0.8)
            self.transitMotor.set(self.transitSpeed)
            if not self.isFullyExtended():
                self.extend()
        else:
            if not self.isFullyRetracted():
                self.retract()
            self.transitMotor.set(0)
            # self.intakeMotorIntake.set(0)

        if self.activelyTransit:
            self.transitMotor.set(self.transitSpeed)
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
