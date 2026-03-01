"""Starting point for Robot code."""

import math
import os

import magicbot
import phoenix6 as p6
import wpilib
import wpimath.geometry as geom
import wpimath.units as units
from magicbot import feedback
from wpilib import RobotBase

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
    vision: components.Vision
    intake: components.Intake
    driverController: components.DriverController
    operatorController: components.OperatorController
    lighting: components.Lighting

    def __init__(self) -> None:
        """Initialize the robot."""
        super().__init__()

        # Track whether the Operator is allowed to show arbitrary LED colors.
        self._operatorCanShowArbitraryLEDColors: bool = False

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

    def disabledInit(self) -> None:
        """Called afer the on_disable() of all components."""
        pass

    def disabledPeriodic(self) -> None:
        """Called periodically while the robot is disabled, before all components' execute()."""
        pass

    def testInit(self) -> None:
        """Called when starting test mode."""
        # Reset pose to (0,0,0) so our distance check works
        self.drivetrain.resetPose(geom.Pose2d(0, 0, geom.Rotation2d(0)))

        self.testTimer = wpilib.Timer()
        self.testTimer.restart()
        self.testState = "forward"
        print("Test Mode Started: Driving Forward 1m")

    def testPeriodic(self) -> None:
        """Called periodically while in test mode."""
        # Simple ping-pong for tuning drive velocity
        # Drive forward for 1 meter (approx 3ft), then backward

        pose = self.drivetrain.getPose()
        testSpeed = 2.0  # m/s

        if self.testState == "forward":
            if pose.X() < 1.0:
                self.drivetrain.drive(velocityX=testSpeed)
            else:
                self.testState = "wait_forward"
                self.testTimer.restart()
                print("Reached 1m, Waiting...")

        elif self.testState == "wait_forward":
            self.drivetrain.drive()
            if self.testTimer.hasElapsed(1.0):
                self.testState = "backward"
                print("Driving Backward")

        elif self.testState == "backward":
            if pose.X() > 0.0:
                self.drivetrain.drive(velocityX=-testSpeed)
            else:
                self.testState = "wait_backward"
                self.testTimer.restart()
                print("Reached 0m, Waiting...")

        elif self.testState == "wait_backward":
            self.drivetrain.drive(velocityX=0, velocityY=0, rotationRate=0)
            if self.testTimer.hasElapsed(1.0):
                self.testState = "forward"
                print("Driving Forward")

    def robotPeriodic(self) -> None:
        """Called periodically regardless of mode, after the mode-specific xxxPeriodic() is called."""
        # Update vision simulation with current robot pose
        if RobotBase.isSimulation():
            pose_2d = self.drivetrain.getPose()
            pose_3d = geom.Pose3d(pose_2d.X(), pose_2d.Y(), 0.0, geom.Rotation3d(0, 0, pose_2d.rotation().radians()))
            self.vision.update_sim(pose_3d)

        # Feed vision measurements to drivetrain for pose estimation fusion
        # Each measurement includes distance-scaled standard deviations
        for measurement in self.vision.get_measurements():
            self.drivetrain.addVisionMeasurement(
                measurement.pose,
                measurement.timestamp,
                measurement.std_devs,
            )
        self.maybeSetOperatorPerspective()
        self.updateLights()

    # ------------------------------------------------------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------------------------------------------------------

    def createMotors(self) -> None:
        """Instantiate all the motors.

        Note: Swerve drive motors are now created internally by the CTRE SwerveDrivetrain API.
        Only create motors for non-swerve mechanisms here.
        """
        self.kickerMotor = p6.hardware.TalonFX(const.CANID.KICKER_MOTOR, const.CANBUS_NAME)
        self.shooterMotorTop = p6.hardware.TalonFX(const.CANID.SHOOTER_MOTOR_TOP, const.CANBUS_NAME)
        self.shooterMotorBottom = p6.hardware.TalonFX(const.CANID.SHOOTER_MOTOR_BOTTOM, const.CANBUS_NAME)

        self.intakeMotorExtend = p6.hardware.TalonFX(const.CANID.INTAKE_MOTOR_EXTEND, const.CANBUS_NAME)
        self.intakeMotorIntake = p6.hardware.TalonFXS(const.CANID.INTAKE_MOTOR_INTAKE, const.CANBUS_NAME)
        self.transitMotor = p6.hardware.TalonFXS(const.CANID.TRANSIT_MOTOR, const.CANBUS_NAME)
        # TODO: Add cancoders

    def createControllers(self) -> None:
        """Set up joystick and gamepad objects here.

        Controller profiles are selected via environment variables. The default for both is "wired".
        - "DRIVER_CONTROLLER": Profile for driver controller
        - "OPERATOR_CONTROLLER": Profile for operator controller

        Use `export DRIVER_CONTROLLER=macwireless` on macOS to set.
        Use `setx DRIVER_CONTROLLER wireless` on Windows to set.

        See components/controllers.py for available profiles.
        """
        self.driverController = components.DriverController(
            const.ControllerPort.DRIVER_CONTROLLER,
            os.getenv("DRIVER_CONTROLLER", "wired"),
        )
        self.operatorController = components.OperatorController(
            const.ControllerPort.OPERATOR_CONTROLLER,
            os.getenv("OPERATOR_CONTROLLER", "wired"),
        )

    def createLights(self) -> None:
        """Set up objects for lighting."""
        self.ledController = wpilib.AddressableLED(const.LED_PWM_PORT)

    def manuallyDrive(self) -> None:
        """Drive the robot based on controller input."""
        # Check if X-stance button is pressed
        if self.driverController.shouldBrake():
            self.drivetrain.brake()
        else:
            max_speed = TunerConstants.speed_at_12_volts

            # Note that the drivetrain automatically handles field-centric control
            # so that "forward" on the joystick is always away from the driver,
            # regardless of which alliance the team is assigned to.
            self.drivetrain.drive(
                velocityX=self.driverController.getMoveForwardPercent() * max_speed,
                velocityY=self.driverController.getMoveLeftPercent() * max_speed,
                rotationRate=self.driverController.getRotateCounterClockwisePercent() * MAX_ROTATION_SPEED,
            )
        self.intake.activelyIntake = self.driverController.shouldIntake()

        if self.driverController.shouldZeroGyro():
            self.drivetrain.zeroHeading()

    def manuallyOperate(self) -> None:
        """Operate the robot based on controller input."""
        if self.operatorController.shouldToggleLEDMode():
            self._operatorCanShowArbitraryLEDColors = not self._operatorCanShowArbitraryLEDColors

        # Handle shooter spin-up modes
        if self.operatorController.shouldSetFallbackShooterSpinSpeed():
            self.pewpew.fallbackSpin()
        elif self.operatorController.shouldSmartAim():
            # Rotate the bot and calculate flywheel speed to aim at the hub
            self.dynamicallyTargetHub()
        else:
            self.pewpew.spinDown()

        self.pewpew.activelyShoot = self.operatorController.shouldShoot()

    def dynamicallyTargetHub(self) -> None:
        """Aim at the hub and set flywheel speed for shoot-while-moving.

        Uses iterative solver to calculate both aim direction and muzzle speed,
        accounting for robot velocity so the fuel lands in the hub while moving.
        """
        # Get shooting solution accounting for robot velocity
        robotVelocity = self.drivetrain.getVelocity()
        solution = self.pewpew.calculateShootingSolution(robotVelocity)

        # Always update flywheel speed to track the solution
        self.pewpew.setTargetMuzzleSpeed(solution.muzzleSpeed)

        # Note: Even if solution.isValid is False, we keep rotating toward the hub
        # so we're ready when the robot slows down. Shooting is prevented by
        # isReadyToFire() which checks solution validity.

        # Use drivetrain's facing-angle mode--with built-in PID--to rotate towards our desired heading.
        maxSpeed = TunerConstants.speed_at_12_volts

        # FIXME: If the driver is braking, we should not strafe at full speed just because smart aim is on.
        # Figure out how to re-use what we already asked the drivetrain to do in manual drive, but change rotation
        self.drivetrain.driveFacingAngle(
            velocityX=self.driverController.getMoveForwardPercent() * maxSpeed,
            velocityY=self.driverController.getMoveLeftPercent() * maxSpeed,
            targetAngle=solution.targetHeading,
        )

    def maybeSetOperatorPerspective(self) -> None:
        """See if we need to set the "perspective" for operator-centric control."""
        alliance: wpilib.DriverStation.Alliance | None = wpilib.DriverStation.getAlliance()
        if alliance == self._alliance_perspective:
            return

        if alliance is None:
            return  # Mostly to make static type checking happy.

        # To be safe, be sure we didn't get put on an unknown alliance
        if alliance in const.ALLIANCE_PERSPECTIVE_ROTATION:
            self.drivetrain.setOperatorPerspectiveForwardOrientation(const.ALLIANCE_PERSPECTIVE_ROTATION[alliance])
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

    @feedback
    def currentShift(self) -> const.TeleopShift:
        """Return the current shift of the match.

        Returns:
            The current shift as a TeleopShift object which works as an enum
            but also includes its name, start time, end time, and duration.

            If not in teleop, returns TeleopShift.UNKNOWN
        """
        if self.isTeleop():
            timeLeftInPhase: units.seconds = wpilib.Timer.getMatchTime()
            # Walk through the shifts from last to first, and return the first one that matches
            for shift in const.TeleopShift.byEndTime():
                if timeLeftInPhase >= shift.endTime:
                    return shift

        return const.TeleopShift.UNKNOWN

    def updateLights(self) -> None:
        """Update the lights based on robot state."""
        if self._operatorCanShowArbitraryLEDColors:
            # Let the operator pick any color they want using the controller
            self.lighting.setColor(self.operatorController.customLEDColor())
            self.lighting.showProgress(1.0)
        else:
            # Pick the color based on the current shift
            shift: const.TeleopShift = self.currentShift()
            self.lighting.showShift(shift=shift, canScoreInHub=self.hubIsActive())
