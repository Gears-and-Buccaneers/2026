"""Intake for the robot.

This component controls a single motorized intake used to pick up and release game pieces (fuel).
It follows the same simple style as the other MagicBot components in this project: tunable speeds,
simple commands (ingest, release, stop) and an `execute` method which is called each control loop
to apply the desired motor output.
"""

import magicbot
import phoenix6 as p6


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

    # Motor injected by MagicBot when the robot sets an attribute of the
    # same name on the robot class.
    intakeMotorExtend: p6.hardware.TalonFX
    intakeMotorIntake: p6.hardware.TalonFXS
    transitMotor: p6.hardware.TalonFXS
    activelyIntake = magicbot.will_reset_to(False)
    activelyTransit = magicbot.will_reset_to(False)

    # Tunable speeds (can be adjusted at runtime via NetworkTables)
    intakeSpeed = magicbot.tunable(0.8)  # positive: pick up
    releaseSpeed = magicbot.tunable(-0.6)  # negative: release

    def __init__(self) -> None:
        """Initialize internal state."""
        # _power is the desired motor output [-1.0, 1.0]
        self._power: float = 0.0
        # human friendly state string for telemetry/debugging
        self._state: str = "stopped"
        self.activelyIntake = False

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

    def retract(self) -> None:
        """Raise the intake mechanism."""

    def _setPower(self, power: float) -> None:
        """Directly set motor output. Clips to [-1.0, 1.0]."""
        self._power = float(max(-1.0, min(1.0, power)))
        self._state = "manual" if self._power != 0.0 else "stopped"

    def isRunning(self) -> bool:
        """Return True when the intake is applying non-zero output."""
        return abs(self._power) > 1e-6

    def isFullyExtended(self) -> bool:
        """Return True if the intake is fully lowered to the field."""
        return False

    def isFullyRaised(self) -> bool:
        """Return True if the intake is fully raised to the robot."""
        return True

    def execute(self) -> None:
        """Called each loop to command the motor."""
        if self.activelyIntake:
            # make sure its extended, run intake motor, run transit motor
            pass
        else:
            pass
            # retract intake, stop intake motor, stop transit motor
