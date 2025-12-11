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
    # PRE-REQUISITE:
    #   To use Tuner X Control Tab without interference, the robot code must NOT
    #   be commanding the motors.
    #
    #   Deploy an empty project (or comment out `self.manuallyDrive()` in robot.py).
    # PHOENIX TUNER X INSTRUCTIONS:
    # 1. Open Phoenix Tuner X and connect to the robot.
    # 2. Put the robot on the ground (carpet is best).
    #    (Note: Tuning on the ground accounts for real-world tire scrub friction).
    # 3. Select one of the Steer motors (e.g., "Front Left Steer").
    # 4. Go to the "Control" tab.
    #    - Control Mode: TorqueCurrentFOC
    #    - Gains: Slot 0
    # 5. Go to the "Plot" tab (or "Graph").
    #    - Add Signals: Position, ClosedLoopTarget, ClosedLoopError, StatorCurrent.
    # 6. Enable the robot (Driver Station Enabled).
    #
    # TUNING STEPS:
    #
    # kS (Static Friction Feedforward) - Amps to overcome friction
    #   - Test: In Control tab, set Control Mode to "TorqueCurrentFOC".
    #     Slowly increase the main control slider
    #     until the wheel just starts to move (rotates in place against the carpet).
    #     That current value is your kS.
    #   - Start: 0.0 Amps | Increment: 0.05 Amps | Typical range: 0.1 - 0.5 Amps
    #
    # kV (Velocity Feedforward) - DERIVED from motor specs, usually no tuning needed
    #   - Derived as: (stall_current / free_speed) * gear_ratio
    #   - If actual velocity consistently undershoots/overshoots target, adjust ±10%
    #
    # kP (Proportional Gain) - Amps per rotation of error
    #   - Test: Set Control Mode back to "Position TorqueCurrentFOC".
    #     Change the "Position" slider abruptly (step response).
    #     Watch the "Position" graph chase the "ClosedLoopTarget".
    #   - Goal: Fast response with minimal overshoot.
    #   - Start: 0.0 Amps/rot | Increment: 10 Amps/rot | Typical range: 30 - 150 Amps/rot
    #
    # kD (Derivative Gain) - Amps per RPS of error change
    #   - Test: Same as kP. If the wheel oscillates (wobbles) when reaching target, add kD.
    #   - Start: 0.0 Amps/RPS | Increment: 0.1 Amps/RPS | Typical range: 0.2 - 2.0 Amps/RPS
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
        .with_k_p(0)  # Start: 0 | Tune up in Tuner X | Amps per rotation of error
        .with_k_i(0)  # Start: 0 | Avoid unless steady-state error persists
        .with_k_d(0)  # Start: 0 | Tune up in Tuner X | Dampens oscillation
        .with_k_s(0)  # Start: 0 | Tune up in Tuner X | Amps to overcome friction
        .with_k_v(_steer_kv)  # DERIVED ~78.4 | Adjust ±10% if needed | Amps per RPS
        .with_k_a(0)  # Usually leave at 0 unless tuning acceleration
        .with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
    )

    # ==========================================================================
    # DRIVE MOTOR GAINS (FOC MODE - ALL VALUES IN AMPS)
    # ==========================================================================
    # TUNING GUIDE FOR FOC MODE:
    #
    # PHOENIX TUNER X INSTRUCTIONS (kS and kV ONLY):
    # 1. Open Phoenix Tuner X and connect to the robot.
    # 2. Put the robot on BLOCKS (wheels off the ground).
    #    (CRITICAL: We only use Tuner X to find kS (gearbox friction) and verify kV.
    #     We DO NOT tune kP here because the wheel has no load/mass on it).
    # 3. Select one of the Drive motors (e.g., "Front Left Drive").
    # 4. Go to the "Control" tab.
    #    - Control Mode: Velocity TorqueCurrentFOC
    #    - Gains: Slot 0
    # 5. Go to the "Plot" tab.
    #    - Add Signals: Velocity, ClosedLoopTarget, ClosedLoopError, StatorCurrent.
    # 6. Enable the robot.
    #
    # TUNING STEPS:
    #
    # kS (Static Friction Feedforward) - Amps to overcome friction
    #   - Test: In Control tab, set Control Mode to "TorqueCurrentFOC".
    #     Slowly increase the main control slider (labeled "Feedforward" or "Output")
    #     until wheel spins consistently.
    #     That current value is your kS (for the gearbox/motor internal friction).
    #   - Start: 0.0 Amps | Increment: 0.05 Amps | Typical range: 0.1 - 0.4 Amps
    #
    # kV (Velocity Feedforward) - DERIVED from motor specs, usually no tuning needed
    #   - Derived as: (stall_current / free_speed) * gear_ratio
    #   - Test: Set Control Mode to "Velocity TorqueCurrentFOC".
    #     Set Velocity slider to 50% of max speed.
    #     If "Velocity" is consistently below "ClosedLoopTarget", increase kV.
    #     If above, decrease kV.
    #
    # kP (Proportional Gain) - Amps per RPS of velocity error
    #   - NOTE: Do NOT tune this fully on blocks. The wheel has no load.
    #   - Tuner X: Set a safe starting value (e.g., 2.0) just to verify the wheel spins.
    #   - Real World Tuning: Put robot on ground. Drive in Teleop.
    #     (Note: Tuner X controls individual motors, not the whole robot. You cannot
    #      use it to "drive" the robot on the ground without dragging wheels).
    #     If robot feels sluggish or velocity lags target, increase kP.
    #     If robot stutters or oscillates, decrease kP.
    #   - Start: 0.0 Amps/RPS | Increment: 1.0 Amps/RPS | Typical range: 1.0 - 10.0 Amps/RPS
    #
    # kD (Derivative Gain) - Usually leave at 0 for drive motors
    #   - Drive motors rarely need kD. Only add if you see velocity oscillation
    #     that kP reduction doesn't fix.
    #   - Start: 0 | If needed: 0.1 | Typical: 0
    #
    # kI (Integral Gain) - Avoid for drive motors
    #   - Can cause integral windup during acceleration. Not recommended.
    #   - Start: 0 | Keep at 0
    # ==========================================================================
    _drive_gains = (
        configs.Slot0Configs()
        .with_k_p(0)  # Start: 0 | Tune up in Tuner X | Amps per RPS of error
        .with_k_i(0)  # Keep at 0 - can cause windup issues
        .with_k_d(0)  # Usually not needed for drive motors
        .with_k_s(0)  # Start: 0 | Tune up in Tuner X | Amps to overcome friction
        .with_k_v(_drive_kv)  # DERIVED ~24.7 | Adjust ±10% if needed | Amps per RPS
    )

    # ==========================================================================
    # SLIP CURRENT
    # The stator current at which the wheels start to slip
    # ==========================================================================
    # TUNING GUIDE:
    # PHOENIX TUNER X INSTRUCTIONS:
    # 1. Place robot on carpet (or competition surface) against a solid wall.
    # 2. Select a Drive motor in Tuner X.
    # 3. Go to "Control" tab.
    #    - Control Mode: TorqueCurrentFOC
    # 4. Go to "Plot" tab.
    #    - Add Signal: StatorCurrent.
    # 5. Enable robot.
    # 6. Slowly increase "Output" slider (Current) until the wheel breaks traction and slips.
    # 7. Note the StatorCurrent value on the plot where slip occurred.
    # 8. Set _slip_current to slightly less than that value (e.g., 10% less).
    #
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
