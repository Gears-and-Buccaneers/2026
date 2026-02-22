"""Classes related to Serve drive."""

import math

import magicbot
import phoenix6 as p6
import wpimath.geometry as geom
import wpimath.kinematics as kinematics
import wpimath.units as units
import constants as const


# TODO: see https://github.com/robotpy/examples/blob/main/SwerveBot/swervemodule.py
class SwerveModule:
    """A single swerve module."""

    # These attributes are automatically set by MagicBot's variable injection
    # when attributes of the same name are set on the robot.
    drive_motor: p6.hardware.TalonFX
    steer_motor: p6.hardware.TalonFX

    def setup(self) -> None:
        """Called after motor injection. Configure motors here."""
        # Configure drive motor
        self.drive_motor.set_position(0)
        self.drive_motor.configurator.apply(
            p6.configs.CurrentLimitsConfigs()
            .with_stator_current_limit(60)  # Max amps that can flow through the motor
            .with_supply_current_limit(40)  # Max amps that can be drawn by the motor
            .with_stator_current_limit_enable(True)
            .with_supply_current_limit_enable(True)
            .with_supply_current_lower_limit(30)  # Current to drop to after the current has exceeded the supply...
            .with_supply_current_lower_time(0.1)  # ...for this many seconds.
        )

        # Configure steer motor
        self.steer_motor.set_position(0)
        self.steer_motor.configurator.apply(
            p6.configs.CurrentLimitsConfigs()
            .with_stator_current_limit(40)  # Max amps that can flow through the motor
            .with_supply_current_limit(30)  # Max amps that can be drawn by the motor
            .with_stator_current_limit_enable(True)
            .with_supply_current_limit_enable(True)
            .with_supply_current_lower_limit(20)  # Current to drop to after the current has exceeded the supply...
            .with_supply_current_lower_time(0.5)  # ...for this many seconds.
        )

        self._desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(0))
        self._current_state = kinematics.SwerveModuleState(0, geom.Rotation2d(0))

    @property
    def desired_state(self) -> kinematics.SwerveModuleState:
        """The state we want this module to achieve."""
        return self._desired_state

    @desired_state.setter
    def desired_state(self, state: kinematics.SwerveModuleState) -> None:
        """Set the desired state, optimizing the rotation."""
        state.optimize(geom.Rotation2d(self.steering_angle))
        self._desired_state = state

    @property
    def current_state(self) -> kinematics.SwerveModuleState:
        """The actual current state of this module.

        Note:
            The state object returned by this method will be mutated on subsequent calls.
        """
        self._current_state.speed = self.drive_velocity
        self._current_state.angle = geom.Rotation2d(self.steering_angle)
        return self._current_state

    @property
    def steering_angle(self) -> units.radians:
        """Get the current steering angle in radians."""
        return self.steer_motor.get_position().value * (2 * math.pi)

    @property
    def drive_velocity(self) -> units.meters_per_second:
        """Get the current drive velocity in meters per second."""
        return self.drive_motor.get_velocity().value

    def execute(self) -> None:
        """Send commands to the motors to achieve desired state."""
        # Control steering motor - PID to desired angle
        angle_error = self._desired_state.angle.radians() - self.steering_angle

        # Normalize angle error to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Simple P controller for steering
        steer_output = angle_error * 2.0  # FIXME: Move this P gain of 2.0 to a constant or tunable
        self.steer_motor.setVoltage(steer_output)

        # Control drive motor - direct velocity control
        self.drive_motor.set_control(p6.controls.VelocityVoltage(self._desired_state.speed))


# TODO: see https://github.com/robotpy/examples/blob/main/SwerveBot/drivetrain.py
class Drivetrain:
    """The full swerve drivetrain."""

    # These attributes are automatically populated by MagicBot's variable injection
    # when variables of the same name are set on the robot.
    front_left_swerve: SwerveModule
    front_right_swerve: SwerveModule
    rear_left_swerve: SwerveModule
    rear_right_swerve: SwerveModule

    # All units are ALWAYS in SI units (meters, seconds, radians, etc.)
    max_free_drive_meters_per_second = magicbot.tunable(1.0)
    max_rotation_radians_per_second = magicbot.tunable(math.pi)  # 180 degrees/second
    max_drive_speed_possible = magicbot.tunable(3.0)

    def __init__(self):
        """Initialize the drivetrain."""
        self.front_left_loc = geom.Translation2d(
            const.RobotDimension.SWERVE_FRONT_LEFT_LOCATION_X,
            const.RobotDimension.SWERVE_FRONT_LEFT_LOCATION_Y,
        )
        self.front_right_loc = geom.Translation2d(
            const.RobotDimension.SWERVE_FRONT_RIGHT_LOCATION_X,
            const.RobotDimension.SWERVE_FRONT_RIGHT_LOCATION_Y,
        )
        self.rear_left_loc = geom.Translation2d(
            const.RobotDimension.SWERVE_REAR_LEFT_LOCATION_X,
            const.RobotDimension.SWERVE_REAR_LEFT_LOCATION_Y,
        )
        self.rear_right_loc = geom.Translation2d(
            const.RobotDimension.SWERVE_REAR_RIGHT_LOCATION_X,
            const.RobotDimension.SWERVE_REAR_RIGHT_LOCATION_Y,
        )
        self.desired_velocity = kinematics.ChassisSpeeds()

    def setup(self) -> None:
        """Called after component injection. Initialize kinematics here."""
        # Create kinematics object for swerve calculations
        self.kinematics = kinematics.SwerveDrive4Kinematics(
            self.front_left_loc, self.front_right_loc, self.rear_left_loc, self.rear_right_loc
        )

    @property
    def module_states(self) -> list[kinematics.SwerveModuleState]:
        """Get current states of all modules."""
        return [
            module.current_state
            for module in (
                self.front_left_swerve,
                self.front_right_swerve,
                self.rear_left_swerve,
                self.rear_right_swerve,
            )
        ]

    def drive(
        self,
        *,
        forward_speed: units.meters_per_second = 0,
        left_speed: units.meters_per_second = 0,
        ccw_speed: units.radians_per_second = 0,
    ) -> None:
        """Drive the robot using field-relative speeds."""
        self.desired_velocity.vx = forward_speed
        self.desired_velocity.vy = left_speed
        self.desired_velocity.omega = ccw_speed

    @property
    def cross_brake(self) -> bool:
        """Whether to use cross-brake mode."""
        return self._cross_brake

    @cross_brake.setter
    def cross_brake(self, enable_cross_brake: bool) -> None:
        """Set cross-brake mode."""
        self._cross_brake = enable_cross_brake
        if enable_cross_brake:
            # In cross-brake mode, set desired velocity to zero...
            self.desired_velocity = kinematics.ChassisSpeeds(0, 0, 0)

            # ...and set swerve module steering to cross-brake angles
            self.front_left_swerve.desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(math.pi / 4))
            self.front_right_swerve.desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(-math.pi / 4))
            self.rear_left_swerve.desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(-math.pi / 4))
            self.rear_right_swerve.desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(math.pi / 4))

    def execute(self) -> None:
        """Calculate and command module states based on desired velocity."""
        # TODO: calculate the center of rotation and pass it along also
        swerve_module_states = self.kinematics.toSwerveModuleStates(self.desired_velocity)

        # Normalize wheel speeds if any exceed maximum speed
        max_speed = max(state.speed for state in swerve_module_states)
        if max_speed > self.max_drive_speed_possible:
            factor = self.max_drive_speed_possible / max_speed
            for state in swerve_module_states:
                state.speed *= factor

        # Command each module
        self.front_left_swerve.desired_state = swerve_module_states[0]
        self.front_right_swerve.desired_state = swerve_module_states[1]
        self.rear_left_swerve.desired_state = swerve_module_states[2]
        self.rear_right_swerve.desired_state = swerve_module_states[3]
