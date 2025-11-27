"""Hand-tuned constants that override Tuner X generated values.

This file contains all the values that need to be manually tuned after
running Phoenix Tuner X for optimal robot performance.

Modify these values based on real-world testing, then import this class
instead of TunerConstants to use your tuned values.
"""

import math

from phoenix6 import configs, signals, swerve, units

from generated.tuner_constants import TunerConstants


class TunedConstants(TunerConstants):
    """Tuned constants that override the Tuner X generated values.

    Inherit from TunerConstants and override only the values that need
    hand tuning. This keeps the Tuner X generated file clean for regeneration.
    """

    # ==========================================================================
    # CLOSED-LOOP OUTPUT TYPE
    # TorqueCurrentFOC provides better torque control and efficiency
    # Requires Phoenix Pro license - falls back to Voltage without Pro
    # NOTE: PID gains need retuning when switching modes (different units)
    # ==========================================================================
    _steer_closed_loop_output = swerve.ClosedLoopOutputType.TORQUE_CURRENT_FOC
    _drive_closed_loop_output = swerve.ClosedLoopOutputType.TORQUE_CURRENT_FOC

    # ==========================================================================
    # MECHANICAL RATIOS
    # Verify these match your specific swerve module configuration
    # ==========================================================================
    # TODO: Verify couple ratio matches your specific swerve module gearing
    _couple_ratio = 3.125

    # TODO: Measure actual wheel radius after wear for accurate odometry
    _wheel_radius: units.meter = 0.0508  # 2 inch wheels (in meters)

    # Gear ratios (SDS MK4i L2)
    _drive_gear_ratio = 6.75
    _steer_gear_ratio = 150.0 / 7.0  # ~21.43

    # ==========================================================================
    # MOTOR CHARACTERISTICS
    # ==========================================================================
    # Kraken X60 specs (use Falcon 500 values in comments if using those motors)
    _motor_free_speed_rpm = 6000  # Kraken X60: 6000 RPM | Falcon 500: 6380 RPM
    _motor_free_speed_rps = _motor_free_speed_rpm / 60.0  # Convert to rotations per second
    _motor_stall_current = 366.0  # Kraken X60: 366A | Falcon 500: 257A

    # ==========================================================================
    # SPEED AT 12 VOLTS
    # Derived from motor free speed, gear ratio, and wheel size
    # ==========================================================================
    # Derived: (motor RPS) * (2 * pi * wheel_radius) / gear_ratio
    speed_at_12_volts: units.meters_per_second = (
        _motor_free_speed_rps * (2 * math.pi * _wheel_radius) / _drive_gear_ratio
    )

    # ==========================================================================
    # DERIVED FEEDFORWARD VALUES (FOR FOC MODE)
    # In TorqueCurrentFOC mode, kV = Amps per unit velocity
    # Derived from motor characteristics: kV = stall_current / free_speed_rps
    # This gives the motor's torque constant in Amps/RPS, then scaled by gear ratio
    # ==========================================================================
    # Motor kV (Amps per RPS at motor shaft)
    _motor_kv = _motor_stall_current / _motor_free_speed_rps  # ~3.66 for Kraken X60

    # Drive kV: Motor kV * gear ratio (Amps per RPS at wheel)
    # Higher gear ratio = more torque needed = higher kV
    _drive_kv = _motor_kv * _drive_gear_ratio  # ~24.7 for Kraken with 6.75:1

    # Steer kV: Motor kV * gear ratio (Amps per RPS at azimuth)
    _steer_kv = _motor_kv * _steer_gear_ratio  # ~78.4 for Kraken with 21.43:1

    # ==========================================================================
    # STEER MOTOR GAINS (FOC MODE - ALL VALUES IN AMPS)
    # ==========================================================================
    # TUNING GUIDE FOR FOC MODE:
    #
    # kS (Static Friction Feedforward) - Amps to overcome friction
    #   - How to tune: Command a very slow constant velocity. If module doesn't move,
    #     increase kS. If it jerks, decrease kS.
    #   - Start: 0.25 Amps | Increment: 0.05 Amps | Typical range: 0.1 - 0.5 Amps
    #
    # kV (Velocity Feedforward) - DERIVED from motor specs, usually no tuning needed
    #   - Derived as: (stall_current / free_speed) * gear_ratio
    #   - If actual velocity consistently undershoots/overshoots target, adjust ±10%
    #
    # kP (Proportional Gain) - Amps per rotation of error
    #   - How to tune: Start low. Increase until module responds quickly to position
    #     commands. If it oscillates, reduce kP or increase kD.
    #   - Start: 50 Amps/rot | Increment: 10 Amps/rot | Typical range: 30 - 150 Amps/rot
    #
    # kD (Derivative Gain) - Amps per RPS of error change
    #   - How to tune: If kP causes oscillation, add kD to dampen. Too much = sluggish.
    #   - Start: 0.5 Amps/RPS | Increment: 0.1 Amps/RPS | Typical range: 0.2 - 2.0 Amps/RPS
    #
    # kI (Integral Gain) - Avoid unless absolutely necessary
    #   - Only use if there's persistent steady-state error after tuning kP/kD/kS.
    #   - Start: 0 | If needed: 0.1 | Typical: 0 (not recommended for steering)
    #
    # kA (Acceleration Feedforward) - Amps per RPS² - usually leave at 0
    #   - Only tune if you need very aggressive acceleration profiles
    # ==========================================================================
    _steer_gains = (
        configs.Slot0Configs()
        .with_k_p(50)  # Start: 50 | Increment: ±10 | Amps per rotation of error
        .with_k_i(0)  # Start: 0 | Avoid unless steady-state error persists
        .with_k_d(0.5)  # Start: 0.5 | Increment: ±0.1 | Dampens oscillation
        .with_k_s(0.25)  # Start: 0.25 | Increment: ±0.05 | Amps to overcome friction
        .with_k_v(_steer_kv)  # DERIVED ~78.4 | Adjust ±10% if needed | Amps per RPS
        .with_k_a(0)  # Usually leave at 0 unless tuning acceleration
        .with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
    )

    # ==========================================================================
    # DRIVE MOTOR GAINS (FOC MODE - ALL VALUES IN AMPS)
    # ==========================================================================
    # TUNING GUIDE FOR FOC MODE:
    #
    # kS (Static Friction Feedforward) - Amps to overcome friction
    #   - How to tune: With robot on blocks, command low velocity. Increase kS until
    #     wheels just start spinning. Back off slightly.
    #   - Start: 0.2 Amps | Increment: 0.05 Amps | Typical range: 0.1 - 0.4 Amps
    #
    # kV (Velocity Feedforward) - DERIVED from motor specs, usually no tuning needed
    #   - Derived as: (stall_current / free_speed) * gear_ratio
    #   - If actual velocity consistently undershoots/overshoots target, adjust ±10%
    #
    # kP (Proportional Gain) - Amps per RPS of velocity error
    #   - How to tune: After kV is set, increase kP for faster error correction.
    #     If wheels oscillate or chatter, reduce kP.
    #   - Start: 0.1 Amps/RPS | Increment: 0.05 Amps/RPS | Typical range: 0.05 - 0.5 Amps/RPS
    #
    # kD (Derivative Gain) - Usually leave at 0 for drive motors
    #   - Drive motors rarely need kD. Only add if you see velocity oscillation
    #     that kP reduction doesn't fix.
    #   - Start: 0 | If needed: 0.001 | Typical: 0
    #
    # kI (Integral Gain) - Avoid for drive motors
    #   - Can cause integral windup during acceleration. Not recommended.
    #   - Start: 0 | Keep at 0
    # ==========================================================================
    _drive_gains = (
        configs.Slot0Configs()
        .with_k_p(0.1)  # Start: 0.1 | Increment: ±0.05 | Amps per RPS of error
        .with_k_i(0)  # Keep at 0 - can cause windup issues
        .with_k_d(0)  # Usually not needed for drive motors
        .with_k_s(0.2)  # Start: 0.2 | Increment: ±0.05 | Amps to overcome friction
        .with_k_v(_drive_kv)  # DERIVED ~24.7 | Adjust ±10% if needed | Amps per RPS
    )

    # ==========================================================================
    # SLIP CURRENT
    # The stator current at which the wheels start to slip
    # ==========================================================================
    # TUNING GUIDE:
    #   - How to tune: Enable the robot, have it against the wall so that wheels slip.
    #     Monitor stator current in Phoenix Tuner. Note the current when wheels slip.
    #     Set _slip_current slightly below that value.
    #   - Alternative: Start at 40A, drive into a wall at low speed, increase until
    #     wheels slip, then back off 10%.
    #   - Start: 40 Amps | Increment: 5 Amps | Typical range: 40 - 80 Amps
    #   - Higher = more pushing power but risks wheel damage
    #   - Lower = less slip but reduced acceleration/pushing force
    # ==========================================================================
    _slip_current: units.ampere = 60.0

    # ==========================================================================
    # CURRENT LIMITS
    # Tune to prevent brownouts while maintaining performance
    # ==========================================================================
    # TUNING GUIDE:
    #   - Steer motors: Don't need much current. 40-60A is typical.
    #   - Drive motors: Default TalonFX limit is 80A. Reduce if brownouts occur.
    #   - Monitor battery voltage during aggressive maneuvers. If it drops below
    #     ~7V, reduce current limits.
    #   - Start: 60A steer, 80A drive | Adjust based on brownout behavior
    # ==========================================================================
    _steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs().with_stator_current_limit(60.0).with_stator_current_limit_enable(True)
    )
