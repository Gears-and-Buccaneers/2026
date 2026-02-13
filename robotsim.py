"""All the code needed to simulate the robot and fuel behavior."""

import math
import random
from typing import ClassVar

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

        # Publish robot pose (for AdvantageScope visualization)
        self._posePublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic("/Simulation/RobotPose", geom.Pose2d)
            .publish(_NT_PUBLISH_OPTIONS)
        )

    def execute(self) -> None:
        """Publish pose to NetworkTables every update."""
        super().execute()
        self._posePublisher.set(self.getPose())
        NetworkTableInstance.getDefault().flush()


class Fuel:
    """A simulated fuel with 3D position and velocity.

    Probably should be using helper libraries that do this for us, instead of manually integrating physics.
    """

    def __init__(self, position: geom.Translation3d, velocity: geom.Translation3d) -> None:
        """Initialize a fuel with position and optional velocity.

        Args:
            position: Initial 3D position of the fuel.
            velocity: Initial 3D velocity (defaults to zero).
        """
        self.position: geom.Translation3d = position
        self.velocity: geom.Translation3d = velocity
        self.bounceCount = 0  # How many times has this fuel bounced on the ground? Used to determine when to remove it.
        self.shouldRemove = False  # Flag to indicate when fuel should be removed from simulation

    def toPose3d(self) -> geom.Pose3d:
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

        # List of active fuel in the simulation
        self._fuel: list[Fuel] = []

        # Timer for physics delta time
        self._timer = wpilib.Timer()
        self._timer.start()
        self._lastUpdateTimestamp: float = (
            0.0  # So we can calculate how long each frame took, for physics to apply velocity
        )

    def launchFuel(
        self,
        speed: units.meters_per_second,
        shooterPosition: geom.Translation2d,
        wheelRecoveryFactor: float = 1.0,
    ) -> None:
        """Launch a new fuel from the shooter at the given speed.

        Adds random variation to speed and launch angle to simulate real-world inconsistencies including:
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

        # How fast this ball will launch
        actualSpeed: units.meters_per_second = speed * speedVariation * ballJitter * effectiveSlip

        # Get robot heading for launch direction
        robotPose = self.drivetrain.getPose()
        robotHeading = robotPose.rotation().radians()

        # Use shooter position (2D) and add height from constants to know where it is in 3D space
        shooterPosition3d = geom.Translation3d(
            shooterPosition.x,
            shooterPosition.y,
            const.RobotDimension.SHOOTER_LOCATION.z,
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
        launchPitch = const.RobotDimension.SHOOTER_ANGLE + ballPitchJitter

        # Calculate velocity components
        # Vertical speed from pitch angle
        verticalSpeed = actualSpeed * math.sin(launchPitch)

        # Horizontal speed from pitch angle, directed by heading + yaw offset
        horizontalSpeed = actualSpeed * math.cos(launchPitch)
        effectiveHeading = geom.Rotation2d(robotHeading + ballYawJitter)

        # Use Translation2d polar constructor (distance, angle) for velocity along the ground plane
        planarVelocity = geom.Translation2d(horizontalSpeed, effectiveHeading)

        # Add robot velocity to fuel velocity (fuel inherits robot's momentum)
        robotVelocity = self.drivetrain.getVelocity()

        # Calculate the full 3D velocity of the fuel by combining planar and vertical components, plus robot's velocity
        velocity = geom.Translation3d(
            planarVelocity.x + robotVelocity.x,
            planarVelocity.y + robotVelocity.y,
            verticalSpeed,
        )

        # Create a new fuel instance that starts at the shooter's position and flies with the calculated velocity
        newFuel = Fuel(position=shooterPosition3d, velocity=velocity)

        # Add this fuel object to the list of active fuel in the simulation
        self._fuel.append(newFuel)

    def execute(self) -> None:
        """Update fuel physics and publish positions to NetworkTables."""
        # Calculate how long it's been since the last update, so we can apply physics correctly based on time elapsed
        now: units.seconds = self._timer.get()
        elapsedTime: units.seconds = now - self._lastUpdateTimestamp

        # Record the time of this update so we can calculate elapsedTime on the next frame
        self._lastUpdateTimestamp = now

        # Skip physics on first frame or if elapsedTime is too large (e.g., after pause)
        if elapsedTime <= 0 or elapsedTime > 0.1:
            self._publishFuel()
            return

        # Update each fuel's physics
        fuelRadius = const.Field.FUEL_DIAMETER / 2.0

        for fuel in self._fuel:
            # Apply gravity to velocity (negative Z)
            fuel.velocity = geom.Translation3d(
                fuel.velocity.x,
                fuel.velocity.y,
                fuel.velocity.z - const.Simulation.GRAVITY * elapsedTime,
            )

            # Update position based on velocity and time elapsed
            fuel.position = geom.Translation3d(
                fuel.position.x + fuel.velocity.x * elapsedTime,
                fuel.position.y + fuel.velocity.y * elapsedTime,
                fuel.position.z + fuel.velocity.z * elapsedTime,
            )

            # Check for ground collision (fuel center at radius height)
            if fuel.position.z <= fuelRadius:
                # Bounce! Reflect velocity and reduce speed
                fuel.bounceCount += 1

                # Only keep fuel if under max bounces
                if fuel.bounceCount > const.Simulation.FUEL_MAX_BOUNCES:
                    fuel.shouldRemove = True
                    continue  # Skip further processing for this fuel

                # Reflect Z velocity and apply damping to all components
                retention = const.Simulation.FUEL_BOUNCE_VELOCITY_RETENTION
                fuel.velocity = geom.Translation3d(
                    fuel.velocity.x * retention,
                    fuel.velocity.y * retention,
                    -fuel.velocity.z * retention,  # Reflect Z, since we bounced
                )

                # However far we went into the "floor"--below the radius--move it that far above the radius
                fuel.position = geom.Translation3d(
                    fuel.position.x,
                    fuel.position.y,
                    fuelRadius + (fuelRadius - fuel.position.z),
                )

        # Use a list comprehension to keep only the fuel that was not supposed to be removed
        self._fuel = [fuel for fuel in self._fuel if not fuel.shouldRemove]
        self._publishFuel()

    def _publishFuel(self) -> None:
        """Publish all fuel positions to NetworkTables, for AdvantageScope visualization."""
        poses: list[geom.Pose3d] = [fuel.toPose3d() for fuel in self._fuel]
        self._fuelPublisher.set(poses)
        NetworkTableInstance.getDefault().flush()


class ShooterSim(components.Shooter):
    """Simulation version of the shooter with flywheel dynamics.

    Tracks flywheel speed, simulates spin-up/spin-down, and emits balls at periodic intervals while shooting.
    """

    # MagicBot will inject this
    fuelSim: FuelSim

    def __init__(self) -> None:
        """Initialize the shooter simulation."""
        super().__init__()

        # Current (actual) flywheel speed in rad/s - lags behind target due to inertia
        self._actualFlywheelSpeed: units.radians_per_second = 0.0

        # Timer so we can periodically launch another ball while actively firing (holding down the "shoot" button)
        self._shootTimer = wpilib.Timer()
        self._shootTimer.start()
        self._nextEmitTime: units.seconds = 0.0

        # Whether we're actively feeding balls to the shooter, aka "shooting"
        self._isShooting: bool = False

    def setShooting(self, shooting: bool) -> None:
        """Set whether balls should be emitted (whether feeder is running)."""
        if shooting and not self._isShooting:
            # Just started shooting - schedule immediate first ball
            self._nextEmitTime = self._shootTimer.get()
        self._isShooting = shooting

    def execute(self) -> None:
        """Update flywheel simulation and emit fuel."""
        super().execute()

        # Update flywheel speed toward requested target speed, simulating inertia
        self._updateFlywheelSpeed()

        # Launch fuel at ~regular intervals while we're actively shooting
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
        targetSpeed: units.radians_per_second = self._targetFlywheelSpeed

        # Get current distance to hub for distance-dependent spin-up
        distance: units.meters = self.distanceToHub()

        # Interpolate spin-up times based on distance
        DIST_8FT: units.meters = units.feetToMeters(8)  # 2.438m
        DIST_20FT: units.meters = units.feetToMeters(20)  # 6.096m

        # TODO(rami): Why does this need to be clamped?
        # Clamp distance to range and calculate interpolation factor (0 = 8ft, 1 = 20ft)
        t: float = max(0.0, min(1.0, (distance - DIST_8FT) / (DIST_20FT - DIST_8FT)))

        # Interpolate spin-up times
        fullSpinupTime: units.seconds = const.ShooterSpec.SPINUP_TIME_8FT + t * (
            const.ShooterSpec.SPINUP_TIME_20FT - const.ShooterSpec.SPINUP_TIME_8FT
        )
        betweenShotsTime: units.seconds = const.ShooterSpec.SPINUP_BETWEEN_SHOTS_8FT + t * (
            const.ShooterSpec.SPINUP_BETWEEN_SHOTS_20FT - const.ShooterSpec.SPINUP_BETWEEN_SHOTS_8FT
        )

        # RPM range from spec: 1479-2215 RPM
        # Convert to rad/s for calculations
        # maxRadPerSec = const.ShooterSpec.RPM_MAX * 2.0 * math.pi / 60.0
        maxRadPerSec = units.rotationsPerMinuteToRadiansPerSecond(const.ShooterSpec.RPM_MAX)

        # Use between-shots spin-up time for recovery (more realistic)
        # Full spin-up from 0 uses the longer time
        speedDelta: units.radians_per_second = abs(targetSpeed - self._actualFlywheelSpeed)
        speedDropFromMax: units.radians_per_second = maxRadPerSec * (const.ShooterSpec.SPEED_DROP_20FT_PERCENT / 100.0)

        if self._actualFlywheelSpeed < 0.1:
            # Starting from stopped - use full spin-up time (distance-dependent)
            accelRate: units.radians_per_second_squared = maxRadPerSec / fullSpinupTime
        elif speedDelta <= speedDropFromMax:
            # Recovering from shot - use between-shots spin-up time (distance-dependent)
            accelRate: units.radians_per_second_squared = speedDropFromMax / betweenShotsTime
        else:
            # Large speed change - use full spin-up time
            accelRate: units.radians_per_second_squared = maxRadPerSec / fullSpinupTime

        # Assume 20ms loop time
        dt = 0.02

        # Calculate motor RPM limit at wheel (after gear reduction)
        # Kraken X60 free speed is 6065 RPM, with 2:1 ratio wheel max is 3032.5 RPM
        # maxWheelRadPerSec = const.ShooterSpec.WHEEL_MAX_RPM * 2.0 * math.pi / 60.0
        maxWheelRadPerSec = units.rotationsPerMinuteToRadiansPerSecond(const.ShooterSpec.WHEEL_MAX_RPM)

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
        now = self._shootTimer.get()

        if now >= self._nextEmitTime:
            self._emitFuel()
            # Schedule next fuel shot time with random interval
            self._nextEmitTime: units.seconds = now + random.uniform(
                const.Simulation.FUEL_EMIT_INTERVAL_MIN,
                const.Simulation.FUEL_EMIT_INTERVAL_MAX,
            )

    def _emitFuel(self) -> None:
        """Emit a single fuel and slow down the flywheel.

        Only fires if isReadyToFire() returns True (which checks wheel speed,
        valid solution, and heading alignment).
        """
        # Check all firing conditions (wheel speed, valid solution, heading)
        if not self.isReadyToFire():
            return

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

    def getActualFlywheelRPM(self) -> units.revolutions_per_minute:
        """Get the current actual flywheel speed in RPM."""
        # return self._actualFlywheelSpeed * 60.0 / (2.0 * math.pi)
        return units.radiansPerSecondToRotationsPerMinute(self._actualFlywheelSpeed)


class ScurvySim(Scurvy):
    """A simulation version of the robot for use in the WPILib robot simulator."""

    # Simulation-specific components
    fuelSim: FuelSim

    # Override drivetrain to use simulation-friendly version
    # Type narrowing is intentional for sim-specific functionality
    drivetrain: DrivetrainSim  # pyright: ignore[reportIncompatibleVariableOverride]

    # Override shooter to use simulation-friendly version with flywheel dynamics
    pewpew: ShooterSim  # pyright: ignore[reportIncompatibleVariableOverride]

    def manuallyOperate(self) -> None:
        """Do what we normally do, but also control shooting on ShooterSim."""
        super().manuallyOperate()

        # Handle ball emission - setShooting controls whether feeder runs
        # If the "shoot" button is held, we want to simulate shooting fuel.
        self.pewpew.setShooting(self.operatorController.shouldShoot())
