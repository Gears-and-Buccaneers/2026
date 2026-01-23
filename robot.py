"""Starting point for Robot code."""

import math
import os

import magicbot
import wpilib
from wpimath.geometry import Pose2d, Rotation2d

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

    def __init__(self) -> None:
        """Initialize the robot."""
        super().__init__()

        # Have we told the Drivetrain which alliance we are on yet?
        self._has_applied_operator_perspective = False

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

    def disabledInit(self) -> None:
        """Called afer the on_disable() of all components."""
        pass

    def disabledPeriodic(self) -> None:
        """Called periodically while the robot is disabled, before all components' execute()."""
        pass

    def testInit(self) -> None:
        """Called when starting test mode."""
        # Reset pose to (0,0,0) so our distance check works
        self.drivetrain.reset_pose(Pose2d(0, 0, Rotation2d(0)))

        self.test_timer = wpilib.Timer()
        self.test_timer.restart()
        self.test_state = "forward"
        print("Test Mode Started: Driving Forward 1m")

    def testPeriodic(self) -> None:
        """Called periodically while in test mode."""
        # Simple ping-pong for tuning drive velocity
        # Drive forward for 1 meter (approx 3ft), then backward

        pose = self.drivetrain.get_pose()
        test_speed = 2.0  # m/s

        if self.test_state == "forward":
            if pose.X() < 1.0:
                self.drivetrain.drive_field_centric(velocity_x=test_speed)
            else:
                self.test_state = "wait_forward"
                self.test_timer.restart()
                print("Reached 1m, Waiting...")

        elif self.test_state == "wait_forward":
            self.drivetrain.drive_field_centric()
            if self.test_timer.hasElapsed(1.0):
                self.test_state = "backward"
                print("Driving Backward")

        elif self.test_state == "backward":
            if pose.X() > 0.0:
                self.drivetrain.drive_field_centric(velocity_x=-test_speed)
            else:
                self.test_state = "wait_backward"
                self.test_timer.restart()
                print("Reached 0m, Waiting...")

        elif self.test_state == "wait_backward":
            self.drivetrain.drive_field_centric(velocity_x=0, velocity_y=0, rotation_rate=0)
            if self.test_timer.hasElapsed(1.0):
                self.test_state = "forward"
                print("Driving Forward")

    def robotPeriodic(self) -> None:
        """Called periodically regardless of mode, after the mode-specific xxxPeriodic() is called."""
        self.maybe_set_operator_perspective()

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
        # Check if we're supposed to be using a USB gamepad for the driver, or XBox controller
        if os.getenv("USE_DRIVER_USB_GAMEPAD") == "1":
            self.driver_controller = components.DriverUSBGamepad(const.ControllerPort.DRIVER_CONTROLLER)
        else:
            self.driver_controller = components.DriverController(const.ControllerPort.DRIVER_CONTROLLER)

    def createLights(self) -> None:
        """Set up CAN objects for lights."""
        pass

    def manuallyDrive(self) -> None:
        """Drive the robot based on controller input."""
        # Check if X-stance button is pressed
        if self.driver_controller.should_brake():
            self.drivetrain.brake()
        else:
            max_speed = TunerConstants.speed_at_12_volts

            # Note that the drivetrain automatically handles field-centric control
            # so that "forward" on the joystick is always away from the driver,
            # regardless of which alliance the team is assigned to.
            self.drivetrain.drive_field_centric(
                velocity_x=self.driver_controller.get_move_forward_percent() * max_speed,
                velocity_y=self.driver_controller.get_move_left_percent() * max_speed,
                rotation_rate=self.driver_controller.get_rotate_counter_clockwise_percent() * MAX_ROTATION_SPEED,
            )

        if self.driver_controller.should_zero_gyro():
            self.drivetrain.zero_heading()

    def maybe_set_operator_perspective(self) -> None:
        """See if we need to set the "perspective" for operator-centric control."""
        if self._has_applied_operator_perspective:
            return

        alliance: wpilib.DriverStation.Alliance | None = wpilib.DriverStation.getAlliance()
        if alliance is not None and alliance in const.ALLIANCE_PERSPECTIVE_ROTATION:
            self.drivetrain.set_operator_perspective_forward_orientation(const.ALLIANCE_PERSPECTIVE_ROTATION[alliance])
            self._has_applied_operator_perspective = True
