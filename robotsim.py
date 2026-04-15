"""All the code needed to simulate the robot and fuel behavior."""

import math
import random

import wpilib
import wpimath.geometry as geom
import wpimath.units as units
from ntcore import NetworkTableInstance, PubSubOptions

import components
import constants as const
from robot import Theseus

# Publish to NetworkTables at 50Hz (20ms) to match robot loop rate
_NT_PUBLISH_OPTIONS = PubSubOptions(periodic=0.02)

_SHOOTER_COAST_DECEL_MULT: float = 0.2


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
    """Simulation shooter: independent top/bottom wheel dynamics from duties; emits fuel via FuelSim."""

    fuelSim: FuelSim

    def __init__(self) -> None:
        """Initialize sim flywheel state and shot timer."""
        super().__init__()
        self._actual_top_wheel_rad_s: units.radians_per_second = 0.0
        self._actual_bottom_wheel_rad_s: units.radians_per_second = 0.0
        self._shootTimer = wpilib.Timer()
        self._shootTimer.start()
        self._nextEmitTime: units.seconds = 0.0
        self._isShooting: bool = False
        self._flywheel_physics_timer = wpilib.Timer()
        self._flywheel_physics_timer.start()
        self._last_flywheel_physics_t: float | None = None

    def _target_wheel_speeds(self) -> tuple[units.radians_per_second, units.radians_per_second]:
        """Steady-state wheel speeds (rad/s) from velocity PID targets (RPS) / gear ratio."""
        ratio = const.RobotDimension.SHOOTER_MOTOR_TO_AXLE_TEETH_RATIO
        top_wheel = abs(self._targetTopRPS) * (2.0 * math.pi) / ratio
        bottom_wheel = abs(self._targetBottomRPS) * (2.0 * math.pi) / ratio
        return top_wheel, bottom_wheel

    def _target_avg_wheel_speed(self) -> units.radians_per_second:
        """Average target wheel speed (rad/s) for energy loss on shot and telemetry."""
        top_t, bottom_t = self._target_wheel_speeds()
        return (top_t + bottom_t) / 2.0

    def _avg_actual_wheel_speed(self) -> units.radians_per_second:
        return (self._actual_top_wheel_rad_s + self._actual_bottom_wheel_rad_s) / 2.0

    @staticmethod
    def _integrate_wheel_toward_target(
        current: units.radians_per_second,
        target: units.radians_per_second,
        accel_rate: units.radians_per_second_squared,
        decel_rate: units.radians_per_second_squared,
        max_magnitude: units.radians_per_second,
        dt: units.seconds,
    ) -> units.radians_per_second:
        """First-order slew toward clamped target; faster decel when coasting down."""
        clamped = max(0.0, min(target, max_magnitude))
        if current < clamped - 1e-9:
            return min(clamped, current + accel_rate * dt)
        if current > clamped + 1e-9:
            return max(clamped, current - decel_rate * dt)
        return clamped

    def setShooting(self, shooting: bool) -> None:
        """Enable or disable periodic fuel emission while the shoot control is held."""
        if shooting and not self._isShooting:
            self._nextEmitTime = self._shootTimer.get()
        self._isShooting = shooting

    def execute(self) -> None:
        """Run hardware outputs, integrate flywheel, and maybe launch fuel."""
        super().execute()
        self._updateFlywheelSpeed()
        if self._isShooting:
            self._maybeEmitFuel()

    def isFlywheelNearTargetSpeed(self) -> bool:
        """Both wheels within tolerance of their duty-implied steady-state speeds."""
        top_tgt, bottom_tgt = self._target_wheel_speeds()
        top_act = self._actual_top_wheel_rad_s
        bottom_act = self._actual_bottom_wheel_rad_s
        tol = self.flywheelSpeedTolerance
        for tgt, act in ((top_tgt, top_act), (bottom_tgt, bottom_act)):
            if abs(tgt) < 1e-6:
                if abs(act) >= 1e-6:
                    return False
            elif abs(act - tgt) / abs(tgt) > tol:
                return False
        return True

    def _updateFlywheelSpeed(self) -> None:
        """Integrate top and bottom wheels toward open-loop targets with distance-tuned accel."""
        now = self._flywheel_physics_timer.get()
        if self._last_flywheel_physics_t is None:
            self._last_flywheel_physics_t = now
            dt: units.seconds = 0.02
        else:
            dt = now - self._last_flywheel_physics_t
            self._last_flywheel_physics_t = now
            if dt <= 0.0 or dt > 0.1:
                dt = 0.02

        top_tgt, bottom_tgt = self._target_wheel_speeds()
        distance: units.meters = self.distanceToHub()

        DIST_8FT: units.meters = units.feetToMeters(8)
        DIST_20FT: units.meters = units.feetToMeters(20)
        t_dist: float = max(0.0, min(1.0, (distance - DIST_8FT) / (DIST_20FT - DIST_8FT)))

        full_spinup: units.seconds = const.ShooterSpec.SPINUP_TIME_8FT + t_dist * (
            const.ShooterSpec.SPINUP_TIME_20FT - const.ShooterSpec.SPINUP_TIME_8FT
        )
        between_shots: units.seconds = const.ShooterSpec.SPINUP_BETWEEN_SHOTS_8FT + t_dist * (
            const.ShooterSpec.SPINUP_BETWEEN_SHOTS_20FT - const.ShooterSpec.SPINUP_BETWEEN_SHOTS_8FT
        )

        ref_max_wheel = units.rotationsPerMinuteToRadiansPerSecond(const.ShooterSpec.RPM_MAX)
        # FIXME: I do not know the new ratio between shooter motor and wheel axle speed
        max_wheel_cap = units.rotationsPerMinuteToRadiansPerSecond(10)
        speed_drop = ref_max_wheel * (const.ShooterSpec.SPEED_DROP_20FT_PERCENT / 100.0)

        def pick_accel(current: float, target: float) -> units.radians_per_second_squared:
            delta = abs(target - current)
            if current < 0.1:
                return ref_max_wheel / full_spinup
            if delta <= speed_drop:
                return speed_drop / between_shots
            return ref_max_wheel / full_spinup

        a_top = pick_accel(self._actual_top_wheel_rad_s, min(top_tgt, max_wheel_cap))
        a_bot = pick_accel(self._actual_bottom_wheel_rad_s, min(bottom_tgt, max_wheel_cap))
        d_top = a_top * _SHOOTER_COAST_DECEL_MULT
        d_bot = a_bot * _SHOOTER_COAST_DECEL_MULT

        self._actual_top_wheel_rad_s = self._integrate_wheel_toward_target(
            self._actual_top_wheel_rad_s,
            top_tgt,
            a_top,
            d_top,
            max_wheel_cap,
            dt,
        )
        self._actual_bottom_wheel_rad_s = self._integrate_wheel_toward_target(
            self._actual_bottom_wheel_rad_s,
            bottom_tgt,
            a_bot,
            d_bot,
            max_wheel_cap,
            dt,
        )

    def _maybeEmitFuel(self) -> None:
        """Launch one fuel if the shot interval has elapsed."""
        now = self._shootTimer.get()
        if now >= self._nextEmitTime:
            self._emitFuel()
            self._nextEmitTime = now + random.uniform(
                const.Simulation.FUEL_EMIT_INTERVAL_MIN,
                const.Simulation.FUEL_EMIT_INTERVAL_MAX,
            )

    def _emitFuel(self) -> None:
        """Spawn fuel at average wheel surface speed; both wheels lose energy to the ball."""
        if not self.isReadyToFire():
            return

        omega_avg = self._avg_actual_wheel_speed()
        exit_speed = omega_avg * const.RobotDimension.FLYWHEEL_RADIUS
        target_avg = self._target_avg_wheel_speed()
        if target_avg > 0.1:
            wheel_recovery_factor = min(1.0, omega_avg / target_avg)
        else:
            wheel_recovery_factor = 1.0

        retain = const.Simulation.FLYWHEEL_SLOWDOWN_PER_SHOT
        self._actual_top_wheel_rad_s *= retain
        self._actual_bottom_wheel_rad_s *= retain
        self.fuelSim.launchFuel(
            speed=exit_speed,
            shooterPosition=self.getPosition(),
            wheelRecoveryFactor=wheel_recovery_factor,
        )

    def getActualFlywheelSpeed(self) -> units.radians_per_second:
        """Simulated average wheel angular velocity (rad/s)."""
        return self._avg_actual_wheel_speed()

    def getActualFlywheelRPM(self) -> units.revolutions_per_minute:
        """Simulated average wheel speed in RPM."""
        return units.radiansPerSecondToRotationsPerMinute(self._avg_actual_wheel_speed())


class TheseusSim(Theseus):
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
