"""Starting point for Robot code."""

import magicbot
import phoenix6
import wpilib

from components.shooter import Shooter

# Maximum rotation speed in rad/s

MOTOR1CANID = 0
MOTOR2CANID = 1
DRIVERCONTROLLERPORTNUM = 0
MOTORSPEED = 0.9

class Scurvy(magicbot.MagicRobot):
    """The main class for the robot."""

    shooter: Shooter
    desired_speed: float
    # Components - the drivetrain now manages motors internally via CTRE swerve API
    # driver_controller: wpilib.XboxController

    # ------------------------------------------------------------------------------------------------------------------
    # MagicBot methods called at the right time; implement these as desired.
    # ------------------------------------------------------------------------------------------------------------------

    def createObjects(self) -> None:
        """Create motors and stuff here."""
        self.desired_speed = 0.0
        self.test_motor1 = phoenix6.hardware.TalonFX(MOTOR1CANID)
        self.test_motor2 = phoenix6.hardware.TalonFX(MOTOR2CANID)

        self.driverController = wpilib.XboxController(DRIVERCONTROLLERPORTNUM)

    def autonomousInit(self) -> None:
        """Called when auto starts (regardless of which one is selected), after every components' on_enable()."""
        pass

    def teleopInit(self) -> None:
        """Called when teleop starts, after all components' on_enable()."""
        pass

    def teleopPeriodic(self) -> None:
        """Called periodically during teleop (and autonomous, if `self.use_teleop_in_autonomous==True`).

        Called before all components' execute().
        """
        wpilib.SmartDashboard.putNumber("Motor1Speed", self.shooter.test_motor1.get())
        wpilib.SmartDashboard.putNumber("Motor2Speed", self.shooter.test_motor2.get())

        if self.driverController.getXButton():
            self.shooter.go(MOTORSPEED)
        elif self.driverController.getAButton():
            self.shooter.go(MOTORSPEED * -1)
        else:
            self.shooter.go(0)


    def disabledInit(self) -> None:
        """Called afer the on_disable() of all components."""
        pass

    def disabledPeriodic(self) -> None:
        """Called periodically while the robot is disabled, before all components' execute()."""
        pass

    def testInit(self) -> None:
        """Called when starting test mode."""
        pass

    def testPeriodic(self) -> None:
        """Called periodically while in test mode."""
        pass

    def robotPeriodic(self) -> None:
        """Called periodically regardless of mode, after the mode-specific xxxPeriodic() is called."""
        pass
