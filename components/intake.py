"""Intake for the robot.

This component controls a single motorized intake used to pick up and
release game pieces (balls). It follows the same simple style as the
other MagicBot components in this project: tunable speeds, simple
commands (pick_up, release, stop) and an `execute` method which is
called each control loop to apply the desired motor output.
"""

import magicbot
import wpilib


class Intake:
	"""A simple intake subsystem.

	Attributes injected by MagicBot:
	- `intakeMotor`: the motor controller (e.g. `wpilib.Talon`) used for the
	  intake rollers.

	Usage:
	- Call `pickUp()` to run the rollers inward and pick up balls.
	- Call `release()` to run the rollers outward and eject balls.
	- Call `stop()` to stop the rollers.
	- Call `lower()` to rotate the intake down into picking position.
	- Call `raise_()` to rotate the intake up and away from the ground.
	- `setPower()` allows manual control with a value in [-1.0, 1.0].
	"""

	# Motor injected by MagicBot when the robot sets an attribute of the
	# same name on the robot class.
	intakeMotor: wpilib.Talon

	# Tunable speeds (can be adjusted at runtime via NetworkTables)
	intakeSpeed = magicbot.tunable(0.8)  # positive: pick up
	releaseSpeed = magicbot.tunable(-0.6)  # negative: release

	def __init__(self) -> None:
		"""Initialize internal state."""
		# _power is the desired motor output [-1.0, 1.0]
		self._power: float = 0.0
		# human friendly state string for telemetry/debugging
		self._state: str = "stopped"

	def pickUp(self, speed: float | None = None) -> None:
		"""Start the intake to pick up balls.

		Args:
			speed: optional manual speed override in [-1.0, 1.0]. If omitted
				   the configured `intakeSpeed` is used.
		"""
		self._power = self.intakeSpeed if speed is None else float(max(-1.0, min(1.0, speed)))
		self._state = "intake"

	def release(self, speed: float | None = None) -> None:
		"""Run the intake in reverse to release balls.

		Args:
			speed: optional manual speed override in [-1.0, 1.0]. If omitted
				   the configured `release_speed` is used.
		"""
		self._power = self.releaseSpeed if speed is None else float(max(-1.0, min(1.0, speed)))
		self._state = "release"

	def stop(self) -> None:
		"""Stop the intake rollers."""
		self._power = 0.0
		self._state = "stopped"

	def lower(self) -> None:
		"""Rotate the intake down into place for picking up fuel."""
		pass

	def raise_(self) -> None:
		"""Rotate the intake up off the ground and over the robot."""
		pass

	def setPower(self, power: float) -> None:
		"""Directly set motor output. Clips to [-1.0, 1.0]."""
		self._power = float(max(-1.0, min(1.0, power)))
		self._state = "manual" if self._power != 0.0 else "stopped"

	def isRunning(self) -> bool:
		"""Return True when the intake is applying non-zero output."""
		return abs(self._power) > 1e-6

	def execute(self) -> None:
		"""Called each loop to command the motor."""
		# Safety: ensure motor exists and motor.set accepts the value
		try:
			self.intakeMotor.set(self._power)
		except Exception:
			# If motor isn't injected or has a different interface, ensure we 
			# don't crash the robot code. In a real robot we'd log this.
			pass
