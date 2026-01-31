"""All the code needed to simulate the robot and fuel behavior."""

import math
from dataclasses import dataclass, field

import wpilib
import wpimath.geometry as geom
import wpimath.units as units
from ntcore import NetworkTableInstance, PubSubOptions

import components
import constants as const
from robot import Scurvy

# Publish to NetworkTables at 50Hz (20ms) to match robot loop rate
_NT_PUBLISH_OPTIONS = PubSubOptions(periodic=0.02)


class DrivetrainSim(components.Drivetrain):
    """Simulation-friendly version of the drivetrain that publishes pose to NetworkTables."""

    def __init__(self) -> None:
        """Initialize the drivetrain simulation with NetworkTables publisher."""
        super().__init__()
        # Publish robot pose for AdvantageScope visualization
        self._posePublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic("/Simulation/RobotPose", geom.Pose2d)
            .publish(_NT_PUBLISH_OPTIONS)
        )

    def execute(self) -> None:
        """Update simulation and publish pose to NetworkTables."""
        super().execute()
        self._posePublisher.set(self.get_pose())
        NetworkTableInstance.getDefault().flush()


@dataclass
class Ball:
    """A simulated fuel ball with 3D position and velocity."""

    position: geom.Translation3d
    velocity: geom.Translation3d = field(default_factory=lambda: geom.Translation3d())

    def to_pose3d(self) -> geom.Pose3d:
        """Convert to Pose3d for NetworkTables publishing."""
        return geom.Pose3d(self.position, geom.Rotation3d())


class FuelSim:
    """Simulates fuel (balls) in 3D space with physics and publishes to NetworkTables.

    AdvantageScope can visualize these as 2026 game pieces on the 3D field.
    """

    # MagicBot will inject the drivetrain so we can get the robot's pose
    drivetrain: DrivetrainSim

    def __init__(self) -> None:
        """Initialize the fuel simulation with NetworkTables publisher."""
        # Create a StructArrayPublisher to publish Pose3d array to NetworkTables
        self._fuelPublisher = (
            NetworkTableInstance.getDefault()
            .getStructArrayTopic("/Simulation/Fuel", geom.Pose3d)
            .publish(_NT_PUBLISH_OPTIONS)
        )

        # List of active balls in flight
        self._balls: list[Ball] = []

        # Timer for physics delta time
        self._timer = wpilib.Timer()
        self._timer.start()
        self._lastTime: float = 0.0

    def launchBall(self, speed: units.meters_per_second) -> None:
        """Launch a new ball from the robot at the given speed.

        Uses a fixed 65° launch angle and launches from the shooter location
        (transformed to field coordinates), in the direction the robot is facing.

        Args:
            speed: The launch speed in m/s.
        """
        # Get robot pose for position and heading
        robotPose = self.drivetrain.get_pose()
        robotHeading = robotPose.rotation().radians()

        # Transform shooter location from robot-relative to field-relative
        # Rotate the shooter's X/Y offset by robot heading, then add robot position
        shooterLocal = const.RobotDimension.SHOOTER_LOCATION
        cosHeading = math.cos(robotHeading)
        sinHeading = math.sin(robotHeading)

        fieldX = robotPose.X() + shooterLocal.X() * cosHeading - shooterLocal.Y() * sinHeading
        fieldY = robotPose.Y() + shooterLocal.X() * sinHeading + shooterLocal.Y() * cosHeading

        position = geom.Translation3d(fieldX, fieldY, shooterLocal.Z())

        # Calculate velocity components using shooter angle from constants
        # Horizontal speed = speed * cos(launch_angle)
        # Vertical speed = speed * sin(launch_angle)
        horizontalSpeed = speed * math.cos(const.RobotDimension.SHOOTER_ANGLE)
        verticalSpeed = speed * math.sin(const.RobotDimension.SHOOTER_ANGLE)

        # Split horizontal into X/Y based on robot heading
        velocity = geom.Translation3d(
            horizontalSpeed * cosHeading,
            horizontalSpeed * sinHeading,
            verticalSpeed,
        )

        self._balls.append(Ball(position=position, velocity=velocity))

    def launchBallWithVelocity(self, position: geom.Translation3d, velocity: geom.Translation3d) -> None:
        """Launch a new ball with explicit position and velocity.

        Args:
            position: Initial 3D position of the ball.
            velocity: Initial 3D velocity of the ball in m/s.
        """
        self._balls.append(Ball(position=position, velocity=velocity))

    def execute(self) -> None:
        """Update ball physics and publish positions to NetworkTables."""
        # Calculate delta time
        currentTime = self._timer.get()
        dt = currentTime - self._lastTime
        self._lastTime = currentTime

        # Skip physics on first frame or if dt is too large (e.g., after pause)
        if dt <= 0 or dt > 0.1:
            self._publish_balls()
            return

        # Update each ball's physics
        updatedBalls: list[Ball] = []
        for ball in self._balls:
            # Apply gravity to velocity (negative Z)
            newVz = ball.velocity.Z() - const.Simulation.GRAVITY * dt
            newVelocity = geom.Translation3d(
                ball.velocity.X(),
                ball.velocity.Y(),
                newVz,
            )

            # Integrate position
            newPosition = geom.Translation3d(
                ball.position.X() + ball.velocity.X() * dt,
                ball.position.Y() + ball.velocity.Y() * dt,
                ball.position.Z() + ball.velocity.Z() * dt,
            )

            # Keep ball if it hasn't hit the ground
            if newPosition.Z() > 0:
                updatedBalls.append(Ball(position=newPosition, velocity=newVelocity))

        self._balls = updatedBalls
        self._publish_balls()

    def _publish_balls(self) -> None:
        """Publish all ball positions to NetworkTables."""
        poses = [ball.to_pose3d() for ball in self._balls]
        self._fuelPublisher.set(poses)
        NetworkTableInstance.getDefault().flush()


class ShooterSim(components.Shooter):
    """Simulation version of the shooter with flywheel dynamics.

    Tracks flywheel speed, simulates spin-up/spin-down, and emits balls
    at periodic intervals while shooting.
    """

    # MagicBot will inject these
    fuelSim: FuelSim

    def __init__(self) -> None:
        """Initialize the shooter simulation."""
        super().__init__()

        # Current (actual) flywheel speed in rad/s - lags behind target due to inertia
        self._actualFlywheelSpeed: units.radians_per_second = 0.0

        # Timer for ball emission interval
        self._shootTimer = wpilib.Timer()
        self._shootTimer.start()
        self._lastBallTime: float = 0.0

        # Whether we're actively feeding balls to the shooter
        self._isShooting: bool = False

    def setShooting(self, shooting: bool) -> None:
        """Set whether balls should be emitted (feeder is running)."""
        if shooting and not self._isShooting:
            # Just started shooting - reset timer for immediate first ball
            self._lastBallTime = self._shootTimer.get() - const.Simulation.BALL_EMIT_INTERVAL
        self._isShooting = shooting

    def execute(self) -> None:
        """Update flywheel simulation and emit balls."""
        super().execute()

        # Update flywheel speed toward target
        self._updateFlywheelSpeed()

        # Emit balls at regular intervals while shooting
        if self._isShooting:
            self._maybeEmitBall()

    def _updateFlywheelSpeed(self) -> None:
        """Simulate flywheel spin-up/spin-down dynamics."""
        # Target speed comes from parent's _targetFlywheelSpeed (set by setTargetFuelSpeed)
        targetSpeed = self._targetFlywheelSpeed

        # Calculate max flywheel speed for acceleration rate
        maxFlywheelSpeed = self._fuelSpeedToFlywheelSpeed(self.maxFuelSpeed)

        # Calculate acceleration rate (reach max speed in FLYWHEEL_SPINUP_TIME)
        accelRate = maxFlywheelSpeed / const.Simulation.FLYWHEEL_SPINUP_TIME

        # Assume 20ms loop time
        dt = 0.02

        if self._actualFlywheelSpeed < targetSpeed:
            # Spin up
            self._actualFlywheelSpeed = min(targetSpeed, self._actualFlywheelSpeed + accelRate * dt)
        elif self._actualFlywheelSpeed > targetSpeed:
            # Spin down (same rate for now)
            self._actualFlywheelSpeed = max(targetSpeed, self._actualFlywheelSpeed - accelRate * dt)

    def _maybeEmitBall(self) -> None:
        """Emit a ball if enough time has passed since the last one."""
        currentTime = self._shootTimer.get()

        if currentTime - self._lastBallTime >= const.Simulation.BALL_EMIT_INTERVAL:
            self._emitBall()
            self._lastBallTime = currentTime

    def _emitBall(self) -> None:
        """Emit a single ball and slow down the flywheel."""
        # Only emit if flywheel is spinning
        if self._actualFlywheelSpeed < 1.0:
            return

        # Calculate exit velocity from flywheel surface speed
        exitSpeed = self._flywheelSpeedToFuelSpeed(self._actualFlywheelSpeed)

        # Slow down flywheel by configured percentage
        self._actualFlywheelSpeed *= const.Simulation.FLYWHEEL_SLOWDOWN_PER_SHOT

        # Launch the ball
        self.fuelSim.launchBall(speed=exitSpeed)

    def getActualFlywheelSpeed(self) -> units.radians_per_second:
        """Get the current actual flywheel speed in rad/s."""
        return self._actualFlywheelSpeed

    def getActualFlywheelRPM(self) -> float:
        """Get the current actual flywheel speed in RPM."""
        return self._actualFlywheelSpeed * 60.0 / (2.0 * math.pi)


class ScurvySim(Scurvy):
    """A simulation version of the robot for use in the WPILib robot simulator."""

    # Simulation-specific components
    fuelSim: FuelSim

    # Override drivetrain to use simulation-friendly version
    # Type narrowing is intentional for sim-specific functionality
    drivetrain: DrivetrainSim  # type: ignore[assignment]

    # Override shooter to use simulation-friendly version with flywheel dynamics
    pewpew: ShooterSim  # type: ignore[assignment]

    def manuallyOperate(self) -> None:
        """Override to control shooting via ShooterSim."""
        super().manuallyOperate()

        # Handle ball emission - setShooting controls whether feeder runs
        self.pewpew.setShooting(self.operator_controller.shouldShoot())
