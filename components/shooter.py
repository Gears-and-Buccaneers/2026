"""A hypothetical shooter component on the robot."""

import math
from typing import NamedTuple

import magicbot
import phoenix6 as p6
import wpilib
import wpimath.geometry as geom
import wpimath.units as units

import constants as const
from components.swerve import Drivetrain


class ShootingSolution(NamedTuple):
    """Complete solution for a shot, stationary or moving."""

    targetHeading: geom.Rotation2d  # Robot heading to aim at hub
    muzzleSpeed: units.meters_per_second  # Required fuel exit velocity
    timeOfFlight: units.seconds  # Estimated flight time
    isValid: bool  # Whether a valid trajectory exists


class Shooter:
    """The robot's shooter.

    Pew! Pew!

    The shooter stores a target flywheel speed (rad/s) that can be set by:
    - fallbackSpin(): Sets a known good speed for a fixed distance
    - autoShooterMotorPower(): Calculates ideal speed for the current distance
    - spinDown(): Sets target flywheel speed to zero
    """

    # MagicBot will inject these
    drivetrain: Drivetrain
    kickerMotor: p6.hardware.TalonFX
    shooterMotorTop: p6.hardware.TalonFX
    shooterMotorBottom: p6.hardware.TalonFX
    activelyShoot = magicbot.will_reset_to(False)

    # Fallback fuel exit speed for a known shooting position (m/s)
    fallbackFuelSpeed = magicbot.tunable(7.5)
    maxFuelSpeed = magicbot.tunable(15.0)

    # Desired launched fuel backspin (RPM).
    # Positive values make bottom wheel faster than top wheel.
    backspinFuelRPM: magicbot.tunable[units.revolutions_per_minute] = magicbot.tunable(60)

    # Target kicker surface speed while actively feeding (m/s)
    targetKickerFuelSpeed: magicbot.tunable[units.meters_per_second] = magicbot.tunable(3.0)

    # Maximum heading error (degrees) to allow shooting
    maxHeadingError = magicbot.tunable(2.0)

    # Distance below the funnel rim to aim for reliable scoring (m)
    targetDistanceBelowRim = magicbot.tunable(0.1)

    def __init__(self):
        """Initialize the shooter."""
        # Target flywheel angular velocity (rad/s)
        self._targetFlywheelSpeed: units.radians_per_second = 0.0

        # Most recent shooting solution (updated by calculateShootingSolution)
        self._currentSolution: ShootingSolution | None = None

    def _fuelSpeedToFlywheelSpeed(self, fuelSpeed: units.meters_per_second) -> units.radians_per_second:
        """Convert fuel exit speed to flywheel angular velocity.

        Args:
            fuelSpeed: Desired fuel exit velocity in m/s.

        Returns:
            Required flywheel angular velocity in rad/s.
        """
        # v = ωr, so ω = v / r
        return fuelSpeed / const.RobotDimension.FLYWHEEL_RADIUS

    def _flywheelSpeedToFuelSpeed(self, flywheelSpeed: units.radians_per_second) -> units.meters_per_second:
        """Convert flywheel angular velocity to fuel exit speed.

        Args:
            flywheelSpeed: Flywheel angular velocity in rad/s.

        Returns:
            Fuel exit velocity in m/s.
        """
        # v = ωr
        return flywheelSpeed * const.RobotDimension.FLYWHEEL_RADIUS

    def _surfaceSpeedToMotorSpeed(
        self,
        surfaceSpeed: units.meters_per_second,
        wheelRadius: units.meters,
        motorToAxleTeethRatio: float,
    ) -> units.radians_per_second:
        """Convert mechanism surface speed to TalonFX motor angular velocity.

        The conversion is:
            wheel_rad_per_sec = surface_m_per_sec / wheel_radius
            motor_rad_per_sec = wheel_rad_per_sec / motor_to_axle_teeth_ratio

        where motor_to_axle_teeth_ratio = motor_teeth / axle_teeth.
        """
        wheelAngularSpeed = surfaceSpeed / wheelRadius
        return wheelAngularSpeed / motorToAxleTeethRatio

    def _wheelSpeedToMotorSpeed(
        self,
        wheelSpeed: units.radians_per_second,
        motorToAxleTeethRatio: float,
    ) -> units.radians_per_second:
        """Convert mechanism wheel angular velocity to TalonFX motor angular velocity."""
        return wheelSpeed / motorToAxleTeethRatio

    def _motorSpeedToDutyCycle(self, motorSpeed: units.radians_per_second) -> float:
        """Convert target motor angular velocity to open-loop duty cycle [-1, 1]."""
        maxMotorAngularSpeed = const.ShooterSpec.MOTOR_FREE_SPEED_RPM * 2.0 * math.pi / 60.0
        if maxMotorAngularSpeed <= 0:
            return 0.0
        return max(-1.0, min(1.0, motorSpeed / maxMotorAngularSpeed))

    def _backspinRpmToFuelSpeedDelta(self, backspinRpm: units.revolutions_per_minute) -> units.meters_per_second:
        """Convert launched fuel spin rate (RPM) to wheel surface speed differential (m/s).

        For two-wheel shooter contact:
            delta_surface_speed = omega_fuel * fuel_diameter
        where omega_fuel is in rad/s.
        """
        fuelAngularSpeed = backspinRpm * 2.0 * math.pi / 60.0
        return fuelAngularSpeed * const.Field.FUEL_DIAMETER

    def _getShooterWheelTargets(self) -> tuple[units.radians_per_second, units.radians_per_second]:
        """Get top/bottom shooter wheel targets including backspin differential."""
        backspinFuelSpeedDelta = self._backspinRpmToFuelSpeedDelta(self.backspinFuelRPM)
        backspinDeltaWheelSpeed = backspinFuelSpeedDelta / const.RobotDimension.FLYWHEEL_RADIUS
        topWheelTargetAngularSpeed = max(0.0, self._targetFlywheelSpeed - 0.5 * backspinDeltaWheelSpeed)
        bottomWheelTargetAngularSpeed = max(0.0, self._targetFlywheelSpeed + 0.5 * backspinDeltaWheelSpeed)
        return topWheelTargetAngularSpeed, bottomWheelTargetAngularSpeed

    def getShooterTargetMotorSpeeds(self) -> tuple[units.radians_per_second, units.radians_per_second]:
        """Get top/bottom shooter motor targets in rad/s."""
        topWheelTargetAngularSpeed, bottomWheelTargetAngularSpeed = self._getShooterWheelTargets()
        return (
            self._wheelSpeedToMotorSpeed(
                topWheelTargetAngularSpeed,
                const.RobotDimension.SHOOTER_MOTOR_TO_AXLE_TEETH_RATIO,
            ),
            self._wheelSpeedToMotorSpeed(
                bottomWheelTargetAngularSpeed,
                const.RobotDimension.SHOOTER_MOTOR_TO_AXLE_TEETH_RATIO,
            ),
        )

    def getKickerTargetMotorSpeed(self) -> units.radians_per_second:
        """Get kicker motor target in rad/s."""
        return self._surfaceSpeedToMotorSpeed(
            self.targetKickerFuelSpeed,
            const.RobotDimension.KICKER_RADIUS,
            const.RobotDimension.KICKER_MOTOR_TO_AXLE_TEETH_RATIO,
        )

    def setTargetMuzzleSpeed(self, muzzleSpeed: units.meters_per_second) -> None:
        """Set the target flywheel speed to achieve the desired muzzle velocity.

        Args:
            muzzleSpeed: Desired fuel exit velocity in m/s.
        """
        self._targetFlywheelSpeed = self._fuelSpeedToFlywheelSpeed(muzzleSpeed)

    def isReadyToFire(self) -> bool:
        """Check if the shooter is ready to fire.

        Returns True if:
        - We have a valid shooting solution
        - Robot heading is within tolerance of target heading

        Returns:
            True if conditions are met for an accurate shot.
        """
        if self._currentSolution is None or not self._currentSolution.isValid:
            return False

        # Check if robot heading is close enough to target
        currentHeading = self.drivetrain.getHeading()
        targetHeading = self._currentSolution.targetHeading
        headingError = abs((currentHeading - targetHeading).degrees())

        # Normalize to [-180, 180]
        if headingError > 180:
            headingError = 360 - headingError

        return headingError <= self.maxHeadingError

    def getTargetFlywheelSpeed(self) -> units.radians_per_second:
        """Get the current target flywheel angular velocity."""
        return self._targetFlywheelSpeed

    def getPosition(self) -> geom.Translation2d:
        """Get the shooter's position in field coordinates (at the ground level)."""
        robotPose = self.drivetrain.getPose()
        return robotPose.translation() + geom.Translation2d(
            const.RobotDimension.SHOOTER_LOCATION.X(),
            const.RobotDimension.SHOOTER_LOCATION.Y(),
        ).rotateBy(robotPose.rotation())

    def distanceToHub(self) -> units.meters:
        """Calculate distance from shooter to hub in meters."""
        alliance = wpilib.DriverStation.getAlliance()
        hubPos = const.Field.getHubPosition(alliance)
        return self.getPosition().distance(hubPos)

    def calculateShootingSolution(self, robotVelocity: geom.Translation2d) -> ShootingSolution:
        """Calculate aim direction and muzzle speed, accounting for robot motion.

        Uses an iterative solver because time-of-flight depends on launch speed,
        which depends on the required muzzle velocity to compensate for robot motion.

        (How far we aim behind the hub depends on how long the fuel is in the air and how fast the robot is moving,
        but how long the fuel is in the air depends on how far away the spot we're aiming at is...
        ...which is what we're trying to find!)

        The fuel's total velocity = muzzle velocity + robot velocity.
        We solve for muzzle velocity such that total velocity lands in the hub.

        Args:
            robotVelocity: Robot's field-centric velocity (m/s). None for stationary.

        Returns:
            ShootingSolution with target heading, muzzle speed, ToF, and validity.
        """
        # Get positions of the shooter and hub (on the ground plane)
        hub_pos: geom.Translation2d = const.Field.getHubPosition(wpilib.DriverStation.getAlliance())
        shooter_pos: geom.Translation2d = self.getPosition()

        # How far (along the ground) is it between the shooter and the hub?
        ground_delta: geom.Translation2d = hub_pos - shooter_pos
        distance: units.meters = ground_delta.norm()  # The length of the offset vector

        # Physical constants
        theta = const.RobotDimension.SHOOTER_ANGLE
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        g = const.Simulation.GRAVITY
        h = const.Field.HUB_TOP_Z - self.targetDistanceBelowRim - const.RobotDimension.SHOOTER_LOCATION.Z()

        # Initial guess: stationary solution
        v_m = self._calculateStationaryLaunchSpeed(distance, h, theta, g)

        # Iterative solver
        max_iterations = 10
        tolerance = 0.001  # m/s

        def _invalidSolution() -> ShootingSolution:
            self._currentSolution = ShootingSolution(
                targetHeading=geom.Rotation2d(ground_delta.X(), ground_delta.Y()),
                muzzleSpeed=self.maxFuelSpeed,
                timeOfFlight=0.0,
                isValid=False,
            )
            return self._currentSolution

        # Iteratively refine the muzzle speed guess until we're close enough, or we hit the max iterations
        for _ in range(max_iterations):
            # Calculate time of flight with current muzzle speed
            # Vertical motion: h = v_z*t - 0.5*g*t^2
            # Solving quadratic: t = (v_z + sqrt(v_z^2 - 2*g*h)) / g
            v_z = v_m * sin_theta
            discriminant = v_z * v_z - 2 * g * h

            # No valid trajectory - target unreachable
            if discriminant < 0:
                return _invalidSolution()

            # Calculate time of flight with current muzzle speed. Return an invalid solution
            # if time is zero or negative (launch speed is too low, the angle is too steep).
            t = (v_z + math.sqrt(discriminant)) / g
            if t <= 0:
                return _invalidSolution()

            # Required muzzle velocity (horizontal) in field frame:
            # v_muzzle = delta/t - robot_velocity
            v_muzzle_required = geom.Translation2d(
                ground_delta.X() / t - robotVelocity.X(),
                ground_delta.Y() / t - robotVelocity.Y(),
            )

            # The muzzle speed from this requirement
            v_m_horizontal = v_muzzle_required.norm()
            v_m_new = v_m_horizontal / cos_theta

            # Check convergence; stop iterating if the change in muzzle speed is within the tolerance
            if abs(v_m_new - v_m) < tolerance:
                v_m = v_m_new
                break

            v_m = v_m_new

        # Final calculation with converged speed
        v_z = v_m * sin_theta
        discriminant = v_z * v_z - 2 * g * h

        if discriminant < 0 or v_m > self.maxFuelSpeed:
            return _invalidSolution()

        t = (v_z + math.sqrt(discriminant)) / g
        v_muzzle_required = geom.Translation2d(
            ground_delta.X() / t - robotVelocity.X(),
            ground_delta.Y() / t - robotVelocity.Y(),
        )

        # The target heading is the direction of the required muzzle velocity
        target_heading = geom.Rotation2d(v_muzzle_required.X(), v_muzzle_required.Y())

        self._currentSolution = ShootingSolution(
            targetHeading=target_heading,
            muzzleSpeed=v_m,
            timeOfFlight=t,
            isValid=True,
        )
        return self._currentSolution

    def _calculateStationaryLaunchSpeed(
        self,
        distance: units.meters,
        height_diff: units.meters,
        theta: units.radians,
        gravity: units.meters_per_second_squared,
    ) -> units.meters_per_second:
        """Calculate launch speed for stationary shot (used as initial guess).

        Args:
            distance: Horizontal distance to target.
            height_diff: Target height minus shooter height.
            theta: Launch angle in radians.
            gravity: Gravitational acceleration.

        Returns:
            Required launch speed, or maxFuelSpeed if impossible.
        """
        denom_term = distance * math.tan(theta) - height_diff
        if denom_term <= 0:
            return self.maxFuelSpeed

        cos_theta = math.cos(theta)
        v_squared = (gravity * distance * distance) / (2 * cos_theta * cos_theta * denom_term)

        if v_squared < 0:
            return self.maxFuelSpeed

        return min(math.sqrt(v_squared), self.maxFuelSpeed)

    def calculateLaunchSpeed(self, distance: units.meters) -> units.meters_per_second:
        """Calculate the required launch speed to reach the hub from a given distance.

        Uses projectile motion physics with a fixed launch angle.
        For a projectile at angle θ aiming at target (distance d, height h above launch):
            v₀ = sqrt(g·d² / (2·cos²(θ)·(d·tan(θ) - h)))

        Args:
            distance: Horizontal distance to hub center in meters.

        Returns:
            Required fuel exit velocity in m/s, or maxFuelSpeed if trajectory is impossible.
        """
        theta = const.RobotDimension.SHOOTER_ANGLE
        g = const.Simulation.GRAVITY

        # Height difference: target height minus shooter height
        h = const.Field.HUB_TOP_Z - self.targetDistanceBelowRim - const.RobotDimension.SHOOTER_LOCATION.Z()

        # The denominator term: d·tan(θ) - h
        # If this is ≤ 0, the target is too high for this angle (trajectory impossible)
        denom_term = distance * math.tan(theta) - h
        if denom_term <= 0:
            # Target unreachable at this angle - return max speed
            return self.maxFuelSpeed

        cos_theta = math.cos(theta)
        v_squared = (g * distance * distance) / (2 * cos_theta * cos_theta * denom_term)

        if v_squared < 0:
            # Shouldn't happen given the check above, but safety first
            return self.maxFuelSpeed

        v = math.sqrt(v_squared)

        # Clamp to max speed
        return min(v, self.maxFuelSpeed)

    def execute(self):
        """This gets called at the end of the control loop."""
        if self._targetFlywheelSpeed <= 0.0:
            self.shooterMotorTop.set(0)
            self.shooterMotorBottom.set(0)
        else:
            topMotorTargetAngularSpeed, bottomMotorTargetAngularSpeed = self.getShooterTargetMotorSpeeds()
            self.shooterMotorTop.set(self._motorSpeedToDutyCycle(topMotorTargetAngularSpeed))
            self.shooterMotorBottom.set(self._motorSpeedToDutyCycle(bottomMotorTargetAngularSpeed))

        if self.activelyShoot and self.isReadyToFire():
            kickerMotorTargetAngularSpeed = self.getKickerTargetMotorSpeed()
            self.kickerMotor.set(self._motorSpeedToDutyCycle(kickerMotorTargetAngularSpeed))
        else:
            # TODO: should we actively brake the kicker motor to ensure it stops, in case we're not ready to shoot?
            self.kickerMotor.set(0)

    def fallbackSpin(self) -> None:
        """Set the shooter to a known good speed for a fixed shooting position."""
        self._targetFlywheelSpeed = self._fuelSpeedToFlywheelSpeed(self.fallbackFuelSpeed)

    def autoShooterMotorPower(self) -> None:
        """Calculate and set the ideal flywheel speed based on distance to hub."""
        distance = self.distanceToHub()
        fuelSpeed = self.calculateLaunchSpeed(distance)
        self._targetFlywheelSpeed = self._fuelSpeedToFlywheelSpeed(fuelSpeed)

    def spinDown(self) -> None:
        """Stop the flywheel (set target speed to 0)."""
        self._targetFlywheelSpeed = 0.0

    def shoot(self) -> None:
        """Shoots the fuel from the robot (triggers feeder)."""
        # TODO: activate the feeder mechanism to launch fuel
        pass
