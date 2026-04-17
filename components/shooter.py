"""Shooter: flywheel velocity control with distance-lerped target RPMs and transit feeding the shooter."""

from typing import Literal, NamedTuple

import magicbot
import ntcore
import phoenix6 as p6
import wpilib
import wpimath.geometry as geom
import wpimath.units as units

import constants as const
from components.swerve import Drivetrain

_NT_PUBLISH_OPTIONS = ntcore.PubSubOptions(periodic=0.02)


class ShootingSolution2(NamedTuple):
    """Aim solution without velocity lead or ballistics."""

    targetHeading: geom.Rotation2d
    isValid: bool


class Shooter:
    """Shooter with top/bottom velocity targets lerped between 10 ft and 20 ft; transit feeds the wheels."""

    drivetrain: Drivetrain
    transitMotor: p6.hardware.TalonFX
    shooterMotorTop: p6.hardware.TalonFX
    shooterMotorBottom: p6.hardware.TalonFX
    activelyShoot = False
    activelyUnshoot = False

    # Motor velocity targets (RPS) at reference distances (lerped and extrapolated outside range).
    # Bottom > top gives backspin. 75% duty ≈ 76 RPS reached ~20 ft in testing.
    topRPS10ft = magicbot.tunable(30.0)
    bottomRPS10ft = magicbot.tunable(46.0)
    topRPS20ft = magicbot.tunable(40.0)
    bottomRPS20ft = magicbot.tunable(55.0)

    fallbackTopRPS = magicbot.tunable(52.0)
    fallbackBottomRPS = magicbot.tunable(58.0)

    transitKickerSpeed = magicbot.tunable(0.75)
    manualUnshootTransitScale = magicbot.tunable(0.5)

    maxHeadingError = magicbot.tunable(5.0)
    """Maximum heading error (degrees) to allow shooting"""

    flywheelSpeedTolerance = magicbot.tunable(0.05)
    """Maximum fractional deviation from target flywheel speed to consider "near" """

    def __init__(self) -> None:
        """Initialize flywheel targets and aim state."""
        self._targetTopRPS: float = 0.0
        self._targetBottomRPS: float = 0.0
        self._topVelocityRequest = p6.controls.VelocityVoltage(0)
        self._bottomVelocityRequest = p6.controls.VelocityVoltage(0)
        self._neutralRequest = p6.controls.NeutralOut()
        self._currentSolution: ShootingSolution2 | None = None
        self._shooterMode: Literal["auto", "fallback", None] = None

    def setup(self) -> None:
        """Create NetworkTables publishers (flywheel telemetry under components/pewpew; AdvantageScope poses)."""
        nt = ntcore.NetworkTableInstance.getDefault()
        st = nt.getTable("components").getSubTable("pewpew")
        self._ntTargetTopRPS = st.getDoubleTopic("TopRPSTarget").publish(_NT_PUBLISH_OPTIONS)
        self._ntActualTopRPS = st.getDoubleTopic("TopRPSActual").publish(_NT_PUBLISH_OPTIONS)
        self._ntTargetBottomRPS = st.getDoubleTopic("BottomRPSTarget").publish(_NT_PUBLISH_OPTIONS)
        self._ntActualBottomRPS = st.getDoubleTopic("BottomRPSActual").publish(_NT_PUBLISH_OPTIONS)
        self._ntTopError = st.getDoubleTopic("TopRPSError").publish(_NT_PUBLISH_OPTIONS)
        self._ntBottomError = st.getDoubleTopic("BottomRPSError").publish(_NT_PUBLISH_OPTIONS)
        self._ntAtSpeed = st.getBooleanTopic("AtSpeed").publish(_NT_PUBLISH_OPTIONS)
        self._hubPublisher = nt.getStructTopic("/AdvantageScope/Hub", geom.Pose2d).publish(_NT_PUBLISH_OPTIONS)
        self._shooterPosePublisher = nt.getStructTopic("/AdvantageScope/ShooterPose", geom.Pose2d).publish(
            _NT_PUBLISH_OPTIONS
        )
        self._smartAimTargetPublisher = nt.getStructTopic("/AdvantageScope/SmartAimTarget", geom.Pose2d).publish(
            _NT_PUBLISH_OPTIONS
        )

    def _publishSmartAimTelemetry(self) -> None:
        """Publish hub, shooter, and aim-debug poses."""
        alliance = wpilib.DriverStation.getAlliance()
        hubTrans = const.Field.getHubPosition(alliance)
        self._hubPublisher.set(geom.Pose2d(hubTrans, geom.Rotation2d()))

        robotPose = self.drivetrain.getPose()
        shooterTrans = self.getPosition()
        self._shooterPosePublisher.set(geom.Pose2d(shooterTrans, robotPose.rotation()))

        targetRotation = (
            self._currentSolution.targetHeading if self._currentSolution is not None else self.getTargetHeading()
        )
        self._smartAimTargetPublisher.set(geom.Pose2d(robotPose.translation(), targetRotation))

    def _getTargetMotorSpeeds(self, distance: units.meters) -> tuple[float, float]:
        """Interpolate top/bottom velocity (RPS) between 10 ft and 20 ft (extrapolate outside)."""
        dist_10 = units.feetToMeters(10)
        dist_20 = units.feetToMeters(20)
        t = (distance - dist_10) / (dist_20 - dist_10)
        top = self.topRPS10ft + t * (self.topRPS20ft - self.topRPS10ft)
        bottom = self.bottomRPS10ft + t * (self.bottomRPS20ft - self.bottomRPS10ft)
        return top, bottom

    def getTargetHeading(self) -> geom.Rotation2d:
        """Heading to point at hub from current shooter position (no velocity lead)."""
        hub_pos = const.Field.getHubPosition(wpilib.DriverStation.getAlliance())
        shooter_pos = self.getPosition()
        ground_delta = hub_pos - shooter_pos
        heading = geom.Rotation2d(ground_delta.X(), ground_delta.Y())
        self._currentSolution = ShootingSolution2(targetHeading=heading, isValid=ground_delta.norm() > 1e-6)
        return heading

    def getPosition(self) -> geom.Translation2d:
        """Field-frame launch point on the ground under the shooter."""
        robotPose = self.drivetrain.getPose()
        return robotPose.translation() + geom.Translation2d(
            const.RobotDimension.SHOOTER_LOCATION.X(),
            const.RobotDimension.SHOOTER_LOCATION.Y(),
        ).rotateBy(robotPose.rotation())

    def distanceToHub(self) -> units.meters:
        """Ground-plane distance from launch point to hub center."""
        alliance = wpilib.DriverStation.getAlliance()
        hub_pos = const.Field.getHubPosition(alliance)
        return self.getPosition().distance(hub_pos)

    def isAimedNearTarget(self) -> bool:
        """True if robot heading is within ``maxHeadingError`` of hub bearing."""
        if self._currentSolution is None or not self._currentSolution.isValid:
            return False

        current_heading = self.drivetrain.getHeading()
        target_heading = self._currentSolution.targetHeading
        heading_error = abs((current_heading - target_heading).degrees())
        if heading_error > 180:
            heading_error = 360 - heading_error
        return heading_error <= self.maxHeadingError

    def isFlywheelNearTargetSpeed(self) -> bool:
        """True if motor velocities are within tolerance of their PID targets."""
        if abs(self._targetTopRPS) < 1e-6 and abs(self._targetBottomRPS) < 1e-6:
            return True
        real_top_rps = self.shooterMotorTop.get_velocity().value
        real_bottom_rps = self.shooterMotorBottom.get_velocity().value
        for target, actual in (
            (self._targetTopRPS, real_top_rps),
            (self._targetBottomRPS, -real_bottom_rps),
        ):
            if abs(target) > 1e-6 and abs(actual - target) / abs(target) > self.flywheelSpeedTolerance:
                return False
        return True

    def isReadyToFire(self) -> bool:
        """True if a spin mode is active, wheels are on target, and (in auto) aimed at hub."""
        if self._shooterMode is None:
            return False
        if not self.isFlywheelNearTargetSpeed():
            return False
        if self._shooterMode == "auto" and not self.isAimedNearTarget():
            return False
        return True

    def spinUpAndTargetHub(self) -> geom.Rotation2d:
        """Set flywheel velocity targets from distance to hub (lerp 10-20 ft) and return hub heading."""
        self._shooterMode = "auto"
        distance = self.distanceToHub()
        self._targetTopRPS, self._targetBottomRPS = self._getTargetMotorSpeeds(distance)
        return self.getTargetHeading()

    def fallbackSpin(self) -> None:
        """Spin at fixed fallback velocities for a known pose."""
        self._shooterMode = "fallback"
        self._targetTopRPS = self.fallbackTopRPS
        self._targetBottomRPS = self.fallbackBottomRPS
        self.getTargetHeading()

    def spinDown(self) -> None:
        """Command zero flywheel target and stop shooting."""
        self._targetTopRPS = 0.0
        self._targetBottomRPS = 0.0
        self.activelyShoot = False
        self._shooterMode = None

    def shoot(self) -> None:
        """Begin feeding with transit (kicker)."""
        self.activelyShoot = True

    def stopShooting(self) -> None:
        """Stop feeding with transit."""
        self.activelyShoot = False

    def execute(self) -> None:
        """Run flywheels and transit each period."""
        if self._targetTopRPS <= 0.0 and self._targetBottomRPS <= 0.0:
            self.shooterMotorTop.set_control(self._neutralRequest)
            self.shooterMotorBottom.set_control(self._neutralRequest)
        else:
            self._topVelocityRequest.velocity = -self._targetTopRPS
            self._bottomVelocityRequest.velocity = self._targetBottomRPS
            self.shooterMotorTop.set_control(self._topVelocityRequest)
            self.shooterMotorBottom.set_control(self._bottomVelocityRequest)

        # if self.activelyShoot and self.isReadyToFire():
        # TODO: enable isReadyToFire() check when we believe in it
        if self.activelyShoot:
            self.transitMotor.set(self.transitKickerSpeed)
        elif self.activelyUnshoot:
            self.transitMotor.set(-self.transitKickerSpeed * self.manualUnshootTransitScale)
        else:
            self.transitMotor.set(0.0)

        actual_top: p6.units.rotations_per_second = self.shooterMotorTop.get_velocity().value
        actual_bottom: p6.units.rotations_per_second = -self.shooterMotorBottom.get_velocity().value
        self._ntTargetTopRPS.set(self._targetTopRPS)
        self._ntActualTopRPS.set(actual_top)
        self._ntTargetBottomRPS.set(self._targetBottomRPS)
        self._ntActualBottomRPS.set(actual_bottom)
        # Use fixed fallback targets for error telemetry so PID tuning has a stable reference.
        self._ntTopError.set(self.fallbackTopRPS - actual_top)
        self._ntBottomError.set(self.fallbackBottomRPS - actual_bottom)
        self._ntAtSpeed.set(self.isFlywheelNearTargetSpeed())
        self._publishSmartAimTelemetry()
