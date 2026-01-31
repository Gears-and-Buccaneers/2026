"""A hypothetical shooter component on the robot."""

import math

import magicbot
import wpilib
import wpimath.geometry as geom
import wpimath.units as units

import constants as const
from components.swerve import Drivetrain


class Shooter:
    """The robot's shooter.

    Pew! Pew!

    The shooter stores a target flywheel speed (rad/s) that can be set by:
    - fallbackSpin(): Sets a known good speed for a fixed distance
    - autoShooterMotorPower(distance): Calculates ideal speed for a given distance

    If neither method is called each frame, the target speed decays to 0.
    """

    # MagicBot will inject these
    drivetrain: Drivetrain
    shooter_motor: wpilib.Talon

    # Fallback fuel exit speed for a known shooting position (m/s)
    fallbackFuelSpeed = magicbot.tunable(7.5)
    maxFuelSpeed = magicbot.tunable(15.0)

    def __init__(self):
        """Initialize the shooter."""
        # Target flywheel angular velocity (rad/s)
        self._targetFlywheelSpeed: units.radians_per_second = 0.0

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

    def getTargetFlywheelSpeed(self) -> units.radians_per_second:
        """Get the current target flywheel angular velocity."""
        return self._targetFlywheelSpeed

    def getPosition(self) -> geom.Translation2d:
        """Get the shooter's position in field coordinates."""
        robotPose = self.drivetrain.get_pose()
        return robotPose.translation() + geom.Translation2d(
            const.RobotDimension.SHOOTER_LOCATION.X(),
            const.RobotDimension.SHOOTER_LOCATION.Y(),
        ).rotateBy(robotPose.rotation())

    def distanceToHub(self) -> units.meters:
        """Calculate distance from shooter to hub in meters."""
        alliance = wpilib.DriverStation.getAlliance()
        hubPos = const.Field.getHubPosition(alliance)
        return self.getPosition().distance(hubPos)

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
        # Use HUB_TARGET_Z (below rim) so balls land IN the funnel
        h = const.Field.HUB_TARGET_Z - const.RobotDimension.SHOOTER_LOCATION.Z()

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

    # TODO: Dual flywheel implementation
    # The real shooter has two TalonFX motors: upperShooterMotor and lowerShooterMotor.
    # They control flywheels above and below the fuel. To impart backspin:
    # - Upper motor runs slightly slower than target
    # - Lower motor runs slightly faster than target
    # The differential creates backspin while maintaining equivalent launch speed.
    # For now, we simulate a single flywheel that imparts all speed with no rotation.

    def execute(self):
        """This gets called at the end of the control loop."""
        # TODO: figure out how to drive the motor(s) towards the target rotational speed(s)
        self.shooter_motor.set(1)

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
