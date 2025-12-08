"""Starting point for Robot code."""

import math

import magicbot
import wpilib

import components
import constants as const
from generated.tuner_constants import TunerConstants

# Maximum rotation speed in rad/s
MAX_ROTATION_SPEED = math.pi


class Scurvy(magicbot.MagicRobot):
    """The main class for the robot."""

    # Components - the drivetrain now manages motors internally via CTRE swerve API
    drivetrain: components.Drivetrain
    pewpew: components.Shooter
    driver_controller: components.DriverController

    # ------------------------------------------------------------------------------------------------------------------
    # MagicBot methods called at the right time; implement these as desired.
    # ------------------------------------------------------------------------------------------------------------------

    def createObjects(self) -> None:
        """Create motors and stuff here."""
        self.createMotors()
        self.createControllers()
        self.createLights()

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
        self.manuallyDrive()  # Assumes we always want to drive manually in teleop
        # self.driveForward()

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

    # ------------------------------------------------------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------------------------------------------------------

    def createMotors(self) -> None:
        """Instantiate all the motors.

        Note: Swerve drive motors are now created internally by the CTRE SwerveDrivetrain API.
        Only create motors for non-swerve mechanisms here.
        """
        self.shooter_motor = wpilib.Talon(const.CANID.SHOOTER_MOTOR)

    def createControllers(self) -> None:
        """Set up joystick and gamepad objects here."""
        self.driver_controller = components.DriverController(const.ControllerPort.DRIVER_CONTROLLER)

    def createLights(self) -> None:
        """Set up CAN objects for lights."""
        pass

    def manuallyDrive(self) -> None:
        """Drive the robot based on controller input."""
        # Joystick values are positive to the right and down
        strafe_right_percent, reverse_percent = self.driver_controller.getLeftStick()
        rotate_right_percent = self.driver_controller.getRightX()

        # Check if brake button is pressed
        if self.driver_controller.should_brake():
            self.drivetrain.brake()
        else:
            # We invert joystick values to get the desired robot motion
            # Joystick: down=positive, right=positive
            # Robot: forward=positive, left=positive, CCW=positive
            max_speed = TunerConstants.speed_at_12_volts
            self.drivetrain.drive(
                forward_speed=-reverse_percent * max_speed,
                left_speed=-strafe_right_percent * max_speed,
                ccw_speed=-rotate_right_percent * MAX_ROTATION_SPEED,
            )
