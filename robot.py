"""Starting point for Robot code."""

import math
import os

import magicbot
import wpilib
import wpimath.geometry as geom
from magicbot import feedback

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
    operator_controller: components.OperatorController

    def __init__(self) -> None:
        """Initialize the robot."""
        super().__init__()

        # Have we told the Drivetrain which alliance we are on yet?
        self._alliance_perspective: wpilib.DriverStation.Alliance | None = None

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
        self.manuallyOperate()  # Assumes we always want to operate manually in teleop
        self.hubIsActive()

    def disabledInit(self) -> None:
        """Called afer the on_disable() of all components."""
        pass

    def disabledPeriodic(self) -> None:
        """Called periodically while the robot is disabled, before all components' execute()."""
        pass

    def testInit(self) -> None:
        """Called when starting test mode."""
        # Reset pose to (0,0,0) so our distance check works
        self.drivetrain.reset_pose(geom.Pose2d(0, 0, geom.Rotation2d(0)))

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
                self.drivetrain.drive(velocity_x=test_speed)
            else:
                self.test_state = "wait_forward"
                self.test_timer.restart()
                print("Reached 1m, Waiting...")

        elif self.test_state == "wait_forward":
            self.drivetrain.drive()
            if self.test_timer.hasElapsed(1.0):
                self.test_state = "backward"
                print("Driving Backward")

        elif self.test_state == "backward":
            if pose.X() > 0.0:
                self.drivetrain.drive(velocity_x=-test_speed)
            else:
                self.test_state = "wait_backward"
                self.test_timer.restart()
                print("Reached 0m, Waiting...")

        elif self.test_state == "wait_backward":
            self.drivetrain.drive(velocity_x=0, velocity_y=0, rotation_rate=0)
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

        if os.getenv("USE_OPERATOR_USB_GAMEPAD") == "1":
            self.operator_controller = components.OperatorUSBGamepad(const.ControllerPort.OPERATOR_CONTROLLER)
        else:
            self.operator_controller = components.OperatorController(const.ControllerPort.OPERATOR_CONTROLLER)

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
            self.drivetrain.drive(
                velocity_x=self.driver_controller.get_move_forward_percent() * max_speed,
                velocity_y=self.driver_controller.get_move_left_percent() * max_speed,
                rotation_rate=self.driver_controller.get_rotate_counter_clockwise_percent() * MAX_ROTATION_SPEED,
            )

        if self.driver_controller.should_zero_gyro():
            self.drivetrain.zero_heading()

    def manuallyOperate(self) -> None:
        """Operate the robot based on controller input."""
        # Handle shooter spin-up modes
        if self.operator_controller.shouldSetFallbackShooterSpinSpeed():
            self.pewpew.fallbackSpin()
        elif self.operator_controller.shouldSmartAim():
            # Rotate the bot and calculate flywheel speed to aim at the hub
            self.dynamicallyTargetHub()
        else:
            self.pewpew.spinDown()

    def dynamicallyTargetHub(self) -> None:
        """Aim at the hub and set flywheel speed for shoot-while-moving.

        Uses iterative solver to calculate both aim direction and muzzle speed,
        accounting for robot velocity so the fuel lands in the hub while moving.
        """
        # Get shooting solution accounting for robot velocity
        robotVelocity = self.drivetrain.get_velocity()
        solution = self.pewpew.calculateShootingSolution(robotVelocity)

        # Always update flywheel speed to track the solution
        self.pewpew.setTargetMuzzleSpeed(solution.muzzleSpeed)

        # Note: Even if solution.isValid is False, we keep rotating toward the hub
        # so we're ready when the robot slows down. Shooting is prevented by
        # isReadyToFire() which checks solution validity.

        # Use drivetrain's facing-angle mode--with built-in PID--to rotate towards our desired heading.
        max_speed = TunerConstants.speed_at_12_volts
        self.drivetrain.drive_facing_angle(
            velocity_x=self.driver_controller.get_move_forward_percent() * max_speed,
            velocity_y=self.driver_controller.get_move_left_percent() * max_speed,
            target_angle=solution.targetHeading,
        )

    def maybe_set_operator_perspective(self) -> None:
        """See if we need to set the "perspective" for operator-centric control."""
        alliance: wpilib.DriverStation.Alliance | None = wpilib.DriverStation.getAlliance()
        if alliance == self._alliance_perspective:
            return

        if alliance is None:
            return  # Mostly to make static type checking happy.

        # To be safe, be sure we didn't get put on an unknown alliance
        if alliance in const.ALLIANCE_PERSPECTIVE_ROTATION:
            self.drivetrain.set_operator_perspective_forward_orientation(const.ALLIANCE_PERSPECTIVE_ROTATION[alliance])
            self._alliance_perspective = alliance

    @feedback
    def hubIsActive(self) -> bool:
        """Check if our alliance's hub is currently active for scoring."""
        alliance = wpilib.DriverStation.getAlliance()
        data = wpilib.DriverStation.getGameSpecificMessage()
        if data in ("B", "R"):  # Checks if we won auto
            self.won_auto = (data == "B") == (alliance == wpilib.DriverStation.Alliance.kBlue)
        else:
            return False

        time_remaining = wpilib.Timer.getMatchTime()
        can_score = True

        if time_remaining < 30:
            can_score = True

        elif time_remaining < 130:  # Checks what block we are and if we can score
            block = int((130 - time_remaining) // 25)
            can_score = (block % 2 == 0) != self.won_auto

        else:
            can_score = True

        return can_score
