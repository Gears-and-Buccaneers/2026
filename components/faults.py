"""Subsystem fault aggregation for the driver dashboard.

Polls battery voltage, brownout state, and Phoenix6 sticky faults on every
registered motor; publishes a single boolean + summary string for Elastic.
"""

import phoenix6 as p6
import wpilib


class FaultMonitor:
    """Aggregates motor sticky faults, brownouts, and low battery into one indicator."""

    LOW_BATTERY_VOLTS = 9

    def __init__(self) -> None:
        self._motors: list[tuple[str, p6.hardware.TalonFX]] = []
        self._brownout_count = 0
        self._was_browned_out = False

    def register(self, name: str, motor: p6.hardware.TalonFX) -> None:
        """Add a motor to the fault watch list. Call once per non-swerve TalonFX."""
        self._motors.append((name, motor))

    def update(self) -> None:
        """Read all watched signals and publish to SmartDashboard. Call from robotPeriodic."""
        problems: list[str] = []

        voltage = wpilib.RobotController.getBatteryVoltage()
        is_browned_out = wpilib.RobotController.isBrownedOut()
        if is_browned_out and not self._was_browned_out:
            self._brownout_count += 1
        self._was_browned_out = is_browned_out

        if is_browned_out:
            problems.append("BROWNOUT")
        elif voltage < self.LOW_BATTERY_VOLTS:
            problems.append(f"LowBatt({voltage:.1f}V)")

        for name, motor in self._motors:
            try:
                if motor.get_sticky_fault_field().value != 0:
                    problems.append(name)
            except AttributeError:
                # API drift safety: skip silently rather than crash robot loop.
                pass

        wpilib.SmartDashboard.putNumber("Faults/BatteryVoltage", voltage)
        wpilib.SmartDashboard.putNumber("Faults/BrownoutCount", self._brownout_count)
        wpilib.SmartDashboard.putBoolean("Faults/LowBattery", voltage < self.LOW_BATTERY_VOLTS)
        wpilib.SmartDashboard.putBoolean("Faults/HasFault", bool(problems))
        wpilib.SmartDashboard.putString("Faults/Summary", ", ".join(problems) if problems else "OK")
