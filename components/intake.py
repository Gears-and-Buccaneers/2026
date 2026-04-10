"""Intake for the robot.

This component controls a single motorized intake used to pick up and release game pieces (fuel).
It follows the same simple style as the other MagicBot components in this project: tunable speeds,
simple commands (ingest, release, stop) and an `execute` method which is called each control loop
to apply the desired motor output.
"""

import math
from typing import Final, Literal

import magicbot
import phoenix6 as p6
import phoenix6.units as p6_units
import wpimath.units as units

import constants as const


class Intake:
    """A simple intake subsystem.

    Attributes injected by MagicBot:
    - `intakeMotor`: the motor controller (e.g. `wpilib.Talon`) used for the
      intake rollers.

    Usage:
    - Call `ingest()` to run the rollers inward and pick up fuel.
    - Call `release()` to run the rollers outward and eject fuel.
    - Call `stop()` to stop the rollers.
    """

    # Motors+ injected by MagicBot when the robot sets an attribute of the same name on the robot class.
    intakeMotorExtendFore: p6.hardware.TalonFX
    intakeMotorExtendAft: p6.hardware.TalonFX
    intakeMotorIntake: p6.hardware.TalonFX
    transitMotor: p6.hardware.TalonFX
    runIntake: bool = False
    """Set to True to extend the intake fully and run the intake rollers; False to stop the intake and retract."""

    activelyTransit: bool = False
    reverseIntake: bool = False
    intakeCANCoder: p6.hardware.CANcoder

    # Tunable speeds (can be adjusted at runtime via NetworkTables)
    intakeSpeed = magicbot.tunable(-1)  # negative: pick up; positive: vomit
    transitSpeed = magicbot.tunable(0.75)
    intakeExtendSpeed = magicbot.tunable(0.45)
    intakeRetractSpeed = magicbot.tunable(-0.45)

    # Calibrated CANCoder readings at extension limit (in sensor rotations).
    # Keep this as internal calibration constant; operators tune physical limits in meters.
    _RETRACTED_CANCODER_ROTATIONS = 0.889850959

    # Linear extension setpoints (meters).
    intakeRetractedMeters = magicbot.tunable(units.inchesToMeters(1))  # Physically stops around 0.0
    intakeExtendedMeters = magicbot.tunable(units.inchesToMeters(10))  # Physically stops around 11.875
    # How close to the target extension we need to be to consider it "close enough" for control purposes
    intakeToleranceMeters = magicbot.tunable(units.inchesToMeters(0.75))

    # ----- Motion Magic profile for the extension motors -----
    # Mechanism units below are CANcoder rotations because we use the CANcoder as the
    # remote feedback source (sensor_to_mechanism_ratio = 1.0).
    #
    # Geometry: 30T pinion @ 3.75" PD → 11.78" circumference, 5:1 motor→pinion,
    # so 1 motor rev = 2.356" linear, 1 pinion rev = 11.78" linear, total travel 15.7".
    # CANcoder reads the pinion through a 40:60 reduction (encoder slower than pinion;
    # the existing position math has ``pinion_rotations = encoder_rotations * 1.5``).
    #     1 cancoder rev = 1.5 pinion revs = 1.5 * 5 = 7.5 motor revs
    # so the rotor-to-sensor ratio is 7.5:1.
    _MOTOR_TO_ENCODER_RATIO: Final[float] = 5.0 * const.RobotDimension.INTAKE_PINION_TO_ENCODER_RATIO

    # Inches of linear travel for one full CANcoder rotation, derived from the calibrated PD.
    _INCHES_PER_ENCODER_REV: Final[float] = (
        const.RobotDimension.INTAKE_PINION_TO_ENCODER_RATIO
        * math.pi
        * units.metersToInches(const.RobotDimension.INTAKE_EXTENSION_GEAR_DIAMETER)
    )

    # Smooth-motion profile (linear in/s, in/s², in/s³) — slow gentle stops on hard stops.
    # 0.5 s @ 15.7" → 31.4 in/s peak; we cruise at 12 in/s (≈76% of peak) for headroom.
    _MM_CRUISE_INCHES_PER_SEC: Final[float] = 12.0
    _MM_ACCEL_INCHES_PER_SEC2: Final[float] = 30.0  # ~0.4 s ramp to cruise
    _MM_JERK_INCHES_PER_SEC3: Final[float] = 150.0  # S-curve smoothing

    # Same profile expressed in CANcoder rev/s, rev/s², rev/s³.
    _MM_CRUISE_VELOCITY: Final[float] = _MM_CRUISE_INCHES_PER_SEC / _INCHES_PER_ENCODER_REV
    _MM_ACCELERATION: Final[float] = _MM_ACCEL_INCHES_PER_SEC2 / _INCHES_PER_ENCODER_REV
    _MM_JERK: Final[float] = _MM_JERK_INCHES_PER_SEC3 / _INCHES_PER_ENCODER_REV

    def __init__(self) -> None:
        """Initialize internal state."""
        # human friendly state string for telemetry/debugging
        self._extendState: Literal["extend", "retract", "hold"] = "hold"

        # Runtime calibration offset for the CANCoder's retracted reference point.
        self._retractedCancoderRotations = self._RETRACTED_CANCODER_ROTATIONS

        self.runIntake = False
        self.activelyTransit = False
        """Set to True to run the transit mechanism; False to stop it."""

        self._position: p6.StatusSignal[p6_units.rotation] | None = None

        # Reusable Motion Magic / Follower control requests; built in setup().
        self._motionMagicRequest: p6.controls.MotionMagicVoltage | None = None
        self._followerRequest: p6.controls.Follower | None = None

        # Last commanded extension target (mechanism = CANcoder rotations). None means
        # we have not been told to move yet, so we leave the motors in brake-neutral.
        self._lastTargetEncoderRotations: float | None = None

    def setup(self) -> None:
        """Configure the CANcoder and Motion Magic on the extension motors.

        We intentionally do not call set_position(0) on the CANcoder here, as this is an
        absolute encoder. The two extension motors are configured to track the CANcoder
        as a remote sensor so Motion Magic operates directly on real linear position.
        """
        magnet_sensor = p6.configs.MagnetSensorConfigs()
        config = p6.configs.CANcoderConfiguration().with_magnet_sensor(magnet_sensor)
        status = self.intakeCANCoder.configurator.apply(config)
        if status.is_error():
            print(f"Intake CANcoder config failed: {status.name}: {status.description}")
            return

        self._position = self.intakeCANCoder.get_position(False)
        self._position.set_update_frequency(50.0)

        self._configureExtensionMotors()

    def _configureExtensionMotors(self) -> None:
        """Apply Motion Magic + remote-CANcoder feedback config to both extension motors.

        Mechanism units below are CANcoder rotations (sensor_to_mechanism_ratio = 1.0),
        which keeps the conversion to/from linear meters in one place: the helper
        ``_metersToEncoderRotations`` (and its inverse, ``extensionPosition``).
        """
        # Feedback: lock both motors onto the shared intake CANcoder.
        feedback_cfg = p6.configs.FeedbackConfigs()
        feedback_cfg.feedback_sensor_source = p6.signals.FeedbackSensorSourceValue.REMOTE_CANCODER
        feedback_cfg.feedback_remote_sensor_id = int(const.CANID.INTAKE_MOTOR_FORE_CANCODER)
        feedback_cfg.rotor_to_sensor_ratio = self._MOTOR_TO_ENCODER_RATIO
        feedback_cfg.sensor_to_mechanism_ratio = 1.0

        # Slot 0 gains, sized for mechanism units of CANcoder rotations.
        # kV is V per (CANcoder rev/s); a Kraken X60 free-spins ~13.5 CANcoder rev/s
        # at 12 V, so kV ≈ 12/13.5 ≈ 0.9. Starting points — tune on the real bot.
        slot0_cfg = p6.configs.Slot0Configs()
        slot0_cfg.k_s = 0.25  # static friction
        slot0_cfg.k_v = 0.90  # velocity feedforward
        slot0_cfg.k_a = 0.10  # acceleration feedforward
        slot0_cfg.k_p = 60.0  # ~6 V per 0.1 enc-rev of position error
        slot0_cfg.k_i = 0.0
        slot0_cfg.k_d = 2.0

        # Smooth Motion Magic profile (gentle ramps so we don't slam the hard stops).
        mm_cfg = p6.configs.MotionMagicConfigs()
        mm_cfg.motion_magic_cruise_velocity = self._MM_CRUISE_VELOCITY
        mm_cfg.motion_magic_acceleration = self._MM_ACCELERATION
        mm_cfg.motion_magic_jerk = self._MM_JERK

        # Both motors keep their default (CCW positive) inversion — that already gives
        # the right convention here: positive fore output retracts, which raises the
        # CANcoder reading, which is what Motion Magic expects (+output → +position).
        # Brake mode is set explicitly so the intake holds when the loop is idle.
        output_cfg = p6.configs.MotorOutputConfigs()
        output_cfg.neutral_mode = p6.signals.NeutralModeValue.BRAKE

        for cfg in (output_cfg, feedback_cfg, slot0_cfg, mm_cfg):
            status = self.intakeMotorExtendFore.configurator.apply(cfg)
            if status.is_error():
                print(f"Intake fore extend config failed: {status.name}: {status.description}")

        status = self.intakeMotorExtendAft.configurator.apply(output_cfg)
        if status.is_error():
            print(f"Intake aft extend config failed: {status.name}: {status.description}")

        # Build the control requests once and reuse them every loop. The aft motor is
        # mounted opposite the fore, so the follower runs OPPOSED to the leader — when
        # the leader pushes +V to retract, the follower pushes -V and also retracts.
        self._motionMagicRequest = p6.controls.MotionMagicVoltage(0).with_slot(0)
        self._followerRequest = p6.controls.Follower(
            int(const.CANID.INTAKE_MOTOR_EXTEND_FORE),
            p6.signals.MotorAlignmentValue.OPPOSED,
        )

    def _metersToEncoderRotations(self, meters: units.meters) -> float:
        """Inverse of ``extensionPosition``: linear meters → absolute CANcoder rotations."""
        meters_per_encoder_rotation = (
            -const.RobotDimension.INTAKE_PINION_TO_ENCODER_RATIO
            * math.pi
            * const.RobotDimension.INTAKE_EXTENSION_GEAR_DIAMETER
        )
        return self._retractedCancoderRotations + (meters / meters_per_encoder_rotation)

    @magicbot.feedback
    def extensionPosition(self) -> units.meters:
        """Linear extension position in meters."""
        if self._position is None:
            return 0
        self._position.refresh()
        encoder_rotations = float(self._position.value) - self._retractedCancoderRotations
        pinion_rotations = -encoder_rotations * const.RobotDimension.INTAKE_PINION_TO_ENCODER_RATIO
        return pinion_rotations * math.pi * const.RobotDimension.INTAKE_EXTENSION_GEAR_DIAMETER

    def calibrateFullyExtendedNow(self) -> None:
        """Treat the current CANcoder reading as the fully-extended position.

        This updates the internal calibration offset used by `extensionPosition()`.
        """
        if self._position is None:
            return

        self._position.refresh()
        encoder_rotations = float(self._position.value)

        meters_per_encoder_rotation = (
            -const.RobotDimension.INTAKE_PINION_TO_ENCODER_RATIO
            * math.pi
            * const.RobotDimension.INTAKE_EXTENSION_GEAR_DIAMETER
        )

        if meters_per_encoder_rotation == 0:
            return

        # Solve for retracted reference so current reading maps to intakeExtendedMeters.
        self._retractedCancoderRotations = encoder_rotations - (self.intakeExtendedMeters / meters_per_encoder_rotation)
        print(
            "Intake calibration captured. "
            f"Use this in code: _RETRACTED_CANCODER_ROTATIONS = {self._retractedCancoderRotations:.9f}"
        )

    def ingest(self) -> None:
        """Start the intake to pick up fuel."""
        self.runIntake = True
        self.reverseIntake = False

    def vomit(self) -> None:
        """Run the intake in reverse to release fuel."""
        self.runIntake = True
        self.reverseIntake = True

    def stop(self) -> None:
        """Stop the intake rollers."""
        self.runIntake = False

    def extend(self) -> None:
        """Lower the intake mechanism."""
        self._extendState = "extend"

    def retract(self) -> None:
        """Raise the intake mechanism."""
        self._extendState = "retract"

    def isFullyExtended(self) -> bool:
        """Return True if the intake is fully lowered to the field."""
        return self.extensionPosition() >= (self.intakeExtendedMeters - self.intakeToleranceMeters)

    def isFullyRetracted(self) -> bool:
        """Return True if the intake is fully raised to the robot."""
        return self.extensionPosition() <= (self.intakeRetractedMeters + self.intakeToleranceMeters)

    def execute(self) -> None:
        """Called each loop to command the motor."""
        if self.runIntake:
            self.intakeMotorIntake.set(self.intakeSpeed if not self.reverseIntake else -self.intakeSpeed)
            if not self.isFullyExtended():
                self.extend()
        else:
            self.transitMotor.set(0)
            self.intakeMotorIntake.set(0)

        if self.activelyTransit:
            self.transitMotor.set(self.transitSpeed if not self.reverseIntake else -self.transitSpeed)
        else:
            self.transitMotor.set(0)

        # Drive extension via Motion Magic position control. The leader (fore) runs the
        # closed loop against the CANcoder; the aft motor mirrors it as a follower.
        if self._extendState == "extend":
            self._lastTargetEncoderRotations = self._metersToEncoderRotations(self.intakeExtendedMeters)
        elif self._extendState == "retract":
            self._lastTargetEncoderRotations = self._metersToEncoderRotations(self.intakeRetractedMeters)
        # "hold" leaves _lastTargetEncoderRotations alone so the controller keeps holding
        # the most recent target instead of drifting.

        if (
            self._lastTargetEncoderRotations is not None
            and self._motionMagicRequest is not None
            and self._followerRequest is not None
        ):
            self.intakeMotorExtendFore.set_control(
                self._motionMagicRequest.with_position(self._lastTargetEncoderRotations)
            )
            self.intakeMotorExtendAft.set_control(self._followerRequest)
