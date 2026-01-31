"""All the code needed to simulate the robot and fuel behavior."""

import math
import random
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
class Fuel:
    """A simulated fuel with 3D position and velocity."""

    position: geom.Translation3d
    velocity: geom.Translation3d = field(default_factory=lambda: geom.Translation3d())
    bounceCount: int = 0

    def to_pose3d(self) -> geom.Pose3d:
        """Convert to Pose3d for NetworkTables publishing."""
        return geom.Pose3d(self.position, geom.Rotation3d())


class FuelSim:
    """Simulates fuel in 3D space with physics; publishes their positions to NetworkTables."""

    # MagicBot injects the drivetrain, so we can get the robot's pose
    drivetrain: DrivetrainSim

    def __init__(self) -> None:
        """Initialize the fuel simulation with NetworkTables publisher."""
        # Create a StructArrayPublisher to publish Pose3d array to NetworkTables
        self._fuelPublisher = (
            NetworkTableInstance.getDefault()
            .getStructArrayTopic("/Simulation/Fuel", geom.Pose3d)
            .publish(_NT_PUBLISH_OPTIONS)
        )

        # List of active fuel in flight
        self._fuel: list[Fuel] = []

        # Timer for physics delta time
        self._timer = wpilib.Timer()
        self._timer.start()
        self._lastTime: float = 0.0

    def launchFuel(
        self, speed: units.meters_per_second, shooterPosition: geom.Translation2d, wheelRecoveryFactor: float = 1.0
    ) -> None:
        """Launch a new fuel from the shooter at the given speed.

        Adds random variation to speed and launch angle to simulate
        real-world inconsistencies including:
        - Base speed variation (mechanical tolerances)
        - Ball-to-ball jitter (ball compression, grip)
        - Wheel slip (contact inconsistency)
        - Angle variations (yaw and pitch)

        Args:
            speed: The base launch speed in m/s (before random variation).
            shooterPosition: The shooter's 2D position in field coordinates.
            wheelRecoveryFactor: 0-1 indicating wheel speed recovery (1 = full speed).
        """
        # Apply wheel slip (ball exits slower than wheel surface speed)
        slipFactor = 1.0 - const.Simulation.WHEEL_SLIP_BASE
        slipVariation = random.uniform(
            -const.Simulation.WHEEL_SLIP_VARIATION,
            const.Simulation.WHEEL_SLIP_VARIATION,
        )
        # More slip when wheels haven't fully recovered
        recoveryPenalty = (1.0 - wheelRecoveryFactor) * 0.02  # Up to 2% extra slip
        effectiveSlip = slipFactor - slipVariation - recoveryPenalty

        # Apply base speed variation (mechanical tolerances)
        speedVariation = 1.0 + random.uniform(
            -const.Simulation.LAUNCH_SPEED_VARIATION,
            const.Simulation.LAUNCH_SPEED_VARIATION,
        )

        # Apply ball-to-ball jitter (compression, grip inconsistency)
        ballJitter = 1.0 + random.uniform(
            -const.Simulation.BALL_SPEED_JITTER,
            const.Simulation.BALL_SPEED_JITTER,
        )

        actualSpeed = speed * speedVariation * ballJitter * effectiveSlip

        # Get robot heading for launch direction
        robotPose = self.drivetrain.get_pose()
        robotHeading = robotPose.rotation().radians()

        # Use shooter position (2D) and add height from constants
        position = geom.Translation3d(
            shooterPosition.X(),
            shooterPosition.Y(),
            const.RobotDimension.SHOOTER_LOCATION.Z(),
        )

        # Apply random angle variations (pitch and yaw)
        yawOffset = random.uniform(
            -const.Simulation.LAUNCH_YAW_VARIATION,
            const.Simulation.LAUNCH_YAW_VARIATION,
        )
        pitchOffset = random.uniform(
            -const.Simulation.LAUNCH_PITCH_VARIATION,
            const.Simulation.LAUNCH_PITCH_VARIATION,
        )

        # Add ball-to-ball angle jitter
        ballYawJitter = random.uniform(
            -const.Simulation.BALL_ANGLE_JITTER,
            const.Simulation.BALL_ANGLE_JITTER,
        )
        ballPitchJitter = random.uniform(
            -const.Simulation.BALL_ANGLE_JITTER,
            const.Simulation.BALL_ANGLE_JITTER,
        )

        # Base launch angle with pitch variation
        launchPitch = const.RobotDimension.SHOOTER_ANGLE + pitchOffset + ballPitchJitter

        # Calculate velocity components
        # Vertical speed from pitch angle
        verticalSpeed = actualSpeed * math.sin(launchPitch)

        # Horizontal speed from pitch angle, directed by heading + yaw offset
        horizontalSpeed = actualSpeed * math.cos(launchPitch)
        effectiveHeading = geom.Rotation2d(robotHeading + yawOffset + ballYawJitter)

        # Use Translation2d polar constructor (distance, angle) for horizontal velocity
        horizontalVelocity = geom.Translation2d(horizontalSpeed, effectiveHeading)

        # Add robot velocity to fuel velocity (fuel inherits robot's momentum)
        robotVelocity = self.drivetrain.get_velocity()

        velocity = geom.Translation3d(
            horizontalVelocity.X() + robotVelocity.X(),
            horizontalVelocity.Y() + robotVelocity.Y(),
            verticalSpeed,
        )

        self._fuel.append(Fuel(position=position, velocity=velocity))

    def launchFuelWithVelocity(self, position: geom.Translation3d, velocity: geom.Translation3d) -> None:
        """Launch a new fuel with explicit position and velocity.

        Args:
            position: Initial 3D position of the fuel.
            velocity: Initial 3D velocity of the fuel in m/s.
        """
        self._fuel.append(Fuel(position=position, velocity=velocity))

    def execute(self) -> None:
        """Update fuel physics and publish positions to NetworkTables."""
        # Calculate delta time
        currentTime = self._timer.get()
        dt = currentTime - self._lastTime
        self._lastTime = currentTime

        # Skip physics on first frame or if dt is too large (e.g., after pause)
        if dt <= 0 or dt > 0.1:
            self._publishFuel()
            return

        # Update each fuel's physics
        fuelRadius = const.Field.FUEL_DIAMETER / 2.0
        updatedFuel: list[Fuel] = []
        for fuel in self._fuel:
            # Apply gravity to velocity (negative Z)
            newVz = fuel.velocity.Z() - const.Simulation.GRAVITY * dt
            newVelocity = geom.Translation3d(
                fuel.velocity.X(),
                fuel.velocity.Y(),
                newVz,
            )

            # Integrate position
            newPosition = geom.Translation3d(
                fuel.position.X() + fuel.velocity.X() * dt,
                fuel.position.Y() + fuel.velocity.Y() * dt,
                fuel.position.Z() + fuel.velocity.Z() * dt,
            )

            # Check for ground collision (fuel center at radius height)
            newBounceCount = fuel.bounceCount
            if newPosition.Z() <= fuelRadius:
                # Bounce! Reflect velocity and reduce speed
                newBounceCount += 1

                # Only keep fuel if under max bounces
                if newBounceCount > const.Simulation.FUEL_MAX_BOUNCES:
                    continue  # Remove this fuel

                # Reflect Z velocity and apply damping to all components
                retention = const.Simulation.FUEL_BOUNCE_VELOCITY_RETENTION
                newVelocity = geom.Translation3d(
                    newVelocity.X() * retention,
                    newVelocity.Y() * retention,
                    -newVelocity.Z() * retention,  # Reflect Z
                )

                # Clamp position to ground level
                newPosition = geom.Translation3d(
                    newPosition.X(),
                    newPosition.Y(),
                    fuelRadius,
                )

            # Safety: ensure fuel never penetrates ground
            if newPosition.Z() < fuelRadius:
                newPosition = geom.Translation3d(
                    newPosition.X(),
                    newPosition.Y(),
                    fuelRadius,
                )

            updatedFuel.append(
                Fuel(
                    position=newPosition,
                    velocity=newVelocity,
                    bounceCount=newBounceCount,
                )
            )

        self._fuel = updatedFuel
        self._publishFuel()

    def _publishFuel(self) -> None:
        """Publish all fuel positions to NetworkTables."""
        poses = [fuel.to_pose3d() for fuel in self._fuel]
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
        self._nextEmitTime: float = 0.0

        # Whether we're actively feeding balls to the shooter
        self._isShooting: bool = False

    def setShooting(self, shooting: bool) -> None:
        """Set whether balls should be emitted (feeder is running)."""
        if shooting and not self._isShooting:
            # Just started shooting - schedule immediate first ball
            self._nextEmitTime = self._shootTimer.get()
        self._isShooting = shooting

    def execute(self) -> None:
        """Update flywheel simulation and emit fuel."""
        super().execute()

        # Update flywheel speed toward target
        self._updateFlywheelSpeed()

        # Emit fuel at regular intervals while shooting
        if self._isShooting:
            self._maybeEmitFuel()

    def _updateFlywheelSpeed(self) -> None:
        """Simulate flywheel spin-up/spin-down dynamics.

        Uses realistic spin-up times based on ShooterSpec measurements,
        interpolated by distance:
        - 8ft (2.44m): 56ms full spin-up, 28ms between shots
        - 20ft (6.10m): 96ms full spin-up, 40ms between shots

        Kraken X60 motor with 2:1 gear ratio, 3.6 lb-in² MOI.
        """
        # Target speed comes from parent's _targetFlywheelSpeed (set by setTargetFuelSpeed)
        targetSpeed = self._targetFlywheelSpeed

        # Get current distance to hub for distance-dependent spin-up
        distance = self.distanceToHub()

        # Interpolate spin-up times based on distance
        # 8ft = 2.438m, 20ft = 6.096m
        DIST_8FT = 2.438
        DIST_20FT = 6.096

        # Clamp distance to range and calculate interpolation factor (0 = 8ft, 1 = 20ft)
        t = max(0.0, min(1.0, (distance - DIST_8FT) / (DIST_20FT - DIST_8FT)))

        # Interpolate spin-up times
        fullSpinupTime = const.ShooterSpec.SPINUP_TIME_8FT + t * (
            const.ShooterSpec.SPINUP_TIME_20FT - const.ShooterSpec.SPINUP_TIME_8FT
        )
        betweenShotsTime = const.ShooterSpec.SPINUP_BETWEEN_SHOTS_8FT + t * (
            const.ShooterSpec.SPINUP_BETWEEN_SHOTS_20FT - const.ShooterSpec.SPINUP_BETWEEN_SHOTS_8FT
        )

        # RPM range from spec: 1479-2215 RPM
        # Convert to rad/s for calculations
        maxRPM = const.ShooterSpec.RPM_MAX
        maxRadPerSec = maxRPM * 2.0 * math.pi / 60.0

        # Use between-shots spin-up time for recovery (more realistic)
        # Full spin-up from 0 uses the longer time
        speedDelta = abs(targetSpeed - self._actualFlywheelSpeed)
        speedDropFromMax = maxRadPerSec * (const.ShooterSpec.SPEED_DROP_20FT_PERCENT / 100.0)

        if self._actualFlywheelSpeed < 0.1:
            # Starting from stopped - use full spin-up time (distance-dependent)
            accelRate = maxRadPerSec / fullSpinupTime
        elif speedDelta <= speedDropFromMax:
            # Recovering from shot - use between-shots spin-up time (distance-dependent)
            accelRate = speedDropFromMax / betweenShotsTime
        else:
            # Large speed change - use full spin-up time
            accelRate = maxRadPerSec / fullSpinupTime

        # Assume 20ms loop time
        dt = 0.02

        # Calculate motor RPM limit at wheel (after gear reduction)
        # Kraken X60 free speed is 6065 RPM, with 2:1 ratio wheel max is 3032.5 RPM
        maxWheelRadPerSec = const.ShooterSpec.WHEEL_MAX_RPM * 2.0 * math.pi / 60.0

        # Clamp target speed to motor capability
        clampedTarget = min(targetSpeed, maxWheelRadPerSec)

        if self._actualFlywheelSpeed < clampedTarget:
            # Spin up
            self._actualFlywheelSpeed = min(clampedTarget, self._actualFlywheelSpeed + accelRate * dt)
        elif self._actualFlywheelSpeed > clampedTarget:
            # Spin down (same rate for now)
            self._actualFlywheelSpeed = max(clampedTarget, self._actualFlywheelSpeed - accelRate * dt)

        # Hard clamp to motor limit (safety)
        self._actualFlywheelSpeed = min(self._actualFlywheelSpeed, maxWheelRadPerSec)

    def _maybeEmitFuel(self) -> None:
        """Emit fuel if enough time has passed since the last one."""
        currentTime = self._shootTimer.get()

        if currentTime >= self._nextEmitTime:
            self._emitFuel()
            # Schedule next fuel with random interval
            self._nextEmitTime: units.seconds = currentTime + random.uniform(
                const.Simulation.FUEL_EMIT_INTERVAL_MIN,
                const.Simulation.FUEL_EMIT_INTERVAL_MAX,
            )

    def _emitFuel(self) -> None:
        """Emit a single fuel and slow down the flywheel.

        Only fires if:
        1. Flywheel is spinning
        2. Wheels are at least 95% of target speed
        3. Robot heading is within 3° of target (if aiming)
        """
        # Only emit if flywheel is spinning
        if self._actualFlywheelSpeed < 0.2:
            return

        # Check if wheels are up to speed (must be at threshold % of target)
        if self._targetFlywheelSpeed > 0.1:
            speedRatio = self._actualFlywheelSpeed / self._targetFlywheelSpeed
            if speedRatio < const.ShooterSpec.WHEEL_READY_THRESHOLD:
                return  # Wheels not ready, don't fire

        # Check if robot is aimed at target (heading within threshold)
        if not self._isAimedAtTarget():
            return  # Not aimed, don't fire

        # Calculate exit velocity from flywheel surface speed
        exitSpeed = self._flywheelSpeedToFuelSpeed(self._actualFlywheelSpeed)

        # Calculate wheel recovery factor (how close to target speed)
        # This affects slip variation - wheels at full speed have better grip
        if self._targetFlywheelSpeed > 0.1:
            wheelRecoveryFactor = min(1.0, self._actualFlywheelSpeed / self._targetFlywheelSpeed)
        else:
            wheelRecoveryFactor = 1.0

        # Slow down flywheel by configured percentage
        self._actualFlywheelSpeed *= const.Simulation.FLYWHEEL_SLOWDOWN_PER_SHOT

        # Launch the fuel from the shooter's current position
        self.fuelSim.launchFuel(
            speed=exitSpeed,
            shooterPosition=self.getPosition(),
            wheelRecoveryFactor=wheelRecoveryFactor,
        )

    def getActualFlywheelSpeed(self) -> units.radians_per_second:
        """Get the current actual flywheel speed in rad/s."""
        return self._actualFlywheelSpeed

    def getActualFlywheelRPM(self) -> float:
        """Get the current actual flywheel speed in RPM."""
        return self._actualFlywheelSpeed * 60.0 / (2.0 * math.pi)

    def _isAimedAtTarget(self) -> bool:
        """Check if the robot is aimed at the hub within the heading threshold.

        Returns:
            True if robot heading is within HEADING_READY_THRESHOLD of the hub.
        """
        # Get robot's current heading
        robotPose = self.drivetrain.get_pose()
        currentHeading = robotPose.rotation().radians()

        # Calculate angle to hub
        shooterPos = self.getPosition()
        alliance = wpilib.DriverStation.getAlliance()
        hubPos = const.Field.getHubPosition(alliance)

        # Vector from shooter to hub
        dx = hubPos.X() - shooterPos.X()
        dy = hubPos.Y() - shooterPos.Y()
        targetHeading = math.atan2(dy, dx)

        # Calculate heading error (wrapped to -π to π)
        headingError = targetHeading - currentHeading
        while headingError > math.pi:
            headingError -= 2 * math.pi
        while headingError < -math.pi:
            headingError += 2 * math.pi

        # Check if within threshold
        return abs(headingError) <= const.ShooterSpec.HEADING_READY_THRESHOLD

    def isReadyToFire(self) -> bool:
        """Check if the shooter is ready to fire (wheels up and aimed).

        Returns:
            True if wheels are at speed and robot is aimed at target.
        """
        # Check wheel speed
        if self._targetFlywheelSpeed > 0.1:
            speedRatio = self._actualFlywheelSpeed / self._targetFlywheelSpeed
            if speedRatio < const.ShooterSpec.WHEEL_READY_THRESHOLD:
                return False

        # Check heading
        if not self._isAimedAtTarget():
            return False

        return True


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
