"""Vision subsystem using PhotonVision for AprilTag pose estimation.

This module handles multiple PhotonVision cameras and provides pose estimates
to the drivetrain for sensor fusion with distance-based confidence scaling.
"""

import math
from dataclasses import dataclass

import ntcore
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.targeting import PhotonPipelineResult
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import DriverStation, RobotBase, SmartDashboard, Timer
from wpimath.geometry import Pose3d, Rotation2d, Transform3d

import constants as const


@dataclass
class VisionMeasurement:
    """A single vision pose measurement from a camera."""

    pose: Pose3d
    timestamp: float
    ambiguity: float
    camera_name: str
    tag_count: int  # Number of tags used for this estimate
    avg_tag_distance: float  # Average distance to tags in meters
    std_devs: tuple[float, float, float]  # (x, y, rotation) standard deviations


class Vision:
    """Vision subsystem that manages multiple PhotonVision cameras.

    This class creates PhotonPoseEstimators for each camera and provides
    pose estimates that can be fused with the drivetrain's odometry.

    Standard deviations scale with distance - far measurements are trusted less.
    Multi-tag estimates are trusted more than single-tag estimates.

    Cameras are configured in constants.py under VisionConfig.
    """

    # Ambiguity threshold - reject poses with higher ambiguity (0-1 scale, lower is better)
    MAX_POSE_AMBIGUITY_SINGLE_TAG = 0.1  # Stricter for single tag
    MAX_POSE_AMBIGUITY_MULTI_TAG = 0.3  # More lenient for multi-tag

    # Distance thresholds (meters)
    MAX_SINGLE_TAG_DISTANCE = 4.0  # Don't trust single-tag beyond 4m (~13ft)
    MAX_MULTI_TAG_DISTANCE = 9.0  # Trust multi-tag up to 9m (~29.5ft)

    # Base standard deviations (meters for x/y, radians for rotation)
    # These get scaled by distance
    BASE_XY_STD_DEV = 0.1  # 10cm base uncertainty
    BASE_ROT_STD_DEV = 0.05  # ~3 degrees base uncertainty

    # How much std dev increases per meter of distance
    XY_STD_DEV_PER_METER = 0.15  # +15cm per meter
    ROT_STD_DEV_PER_METER = 0.03  # +~2 degrees per meter

    # Multi-tag bonus - divide std devs by this when using multiple tags
    MULTI_TAG_STD_DEV_DIVISOR = 2.0

    # Gyro consistency check - only applied to single-tag measurements
    MAX_GYRO_ROTATION_DIFF_RADIANS = math.radians(15.0)

    # Lenient threshold used after enable until vision has provided initial corrections
    STARTUP_GYRO_ROTATION_DIFF_RADIANS = math.radians(180.0)

    # Number of accepted vision measurements before tightening the gyro check
    STARTUP_MEASUREMENT_THRESHOLD = 40

    # Cross-camera outlier detection
    CROSS_CAMERA_MAX_DISAGREEMENT_METERS = 0.5
    CROSS_CAMERA_STD_DEV_PENALTY_MULTIPLIER = 3.0

    def __init__(self) -> None:
        """Initialize the vision subsystem with all configured cameras."""
        # Load the AprilTag field layout for the current game
        self._field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)

        # Track if we're in simulation
        self._is_simulation = RobotBase.isSimulation()

        # Create cameras and pose estimators
        self._cameras: list[tuple[PhotonCamera, PhotonPoseEstimator, Transform3d, str]] = []

        # Simulation support
        self._vision_sim: "VisionSystemSim | None" = None
        if self._is_simulation:
            from photonlibpy.simulation import VisionSystemSim

            self._vision_sim = VisionSystemSim("main")
            self._vision_sim.addAprilTags(self._field_layout)

        # Initialize all three cameras
        self._setup_camera(
            const.VisionConfig.FRONT_CAMERA_NAME,
            const.RobotDimension.FRONT_CAMERA_TRANSFORM,
        )
        self._setup_camera(
            const.VisionConfig.LEFT_CAMERA_NAME,
            const.RobotDimension.LEFT_CAMERA_TRANSFORM,
        )
        self._setup_camera(
            const.VisionConfig.REAR_CAMERA_NAME,
            const.RobotDimension.REAR_CAMERA_TRANSFORM,
        )

        # Store latest measurements for each camera
        self._latest_measurements: list[VisionMeasurement | None] = [None] * len(self._cameras)

        # Gyro heading for consistency checks (set each cycle by robot.py)
        self._gyro_heading: Rotation2d | None = None

        self._nt = ntcore.NetworkTableInstance.getDefault().getTable("components").getSubTable("vision")
        # Count accepted measurements after enable to know when vision has corrected the pose
        self._accepted_measurement_count: int = 0

    def _setup_camera(self, name: str, robot_to_camera: Transform3d) -> None:
        """Set up a single camera with its pose estimator.

        Args:
            name: The camera name (must match PhotonVision configuration).
            robot_to_camera: Transform from robot center to camera position.

        Note:
            In simulation, PhotonCamera will log a warning about not finding the coprocessor.
            This is expected - VisionSystemSim provides the data instead.
        """
        camera = PhotonCamera(name)
        # New photonlibpy API - just pass field layout and transform
        estimator = PhotonPoseEstimator(self._field_layout, robot_to_camera)

        # Add simulated camera if in simulation
        if self._is_simulation and self._vision_sim is not None:
            from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties

            # Configure simulated camera properties (adjust for your cameras)
            camera_props = SimCameraProperties()
            # Set calibration: width, height, diagonal FOV
            camera_props.setCalibrationFromFOV(1280, 720, Rotation2d.fromDegrees(70))  # 720p, 70° diagonal FOV
            camera_props.setCalibError(0.25, 0.08)  # Avg/stddev pixel error
            camera_props.setFPS(30.0)
            camera_props.setAvgLatency(35.0)  # Average latency in milliseconds
            camera_props.setLatencyStdDev(5.0)  # Latency std dev in milliseconds

            camera_sim = PhotonCameraSim(camera, camera_props)
            # Note: enableDrawWireframe() is not implemented in current photonlibpy
            # camera_sim.enableDrawWireframe(True)
            self._vision_sim.addCamera(camera_sim, robot_to_camera)

        self._cameras.append((camera, estimator, robot_to_camera, name))

    def setup(self) -> None:
        """Called by MagicBot after injection."""
        pass

    def on_enable(self) -> None:
        """Called by MagicBot when the robot is enabled."""
        self._accepted_measurement_count = 0

    def update_sim(self, robot_pose: Pose3d) -> None:
        """Update the vision simulation with the robot's current pose.

        Call this every loop when running in simulation.

        Args:
            robot_pose: The robot's current pose (from drivetrain).
        """
        if self._vision_sim is not None:
            self._vision_sim.update(robot_pose)

    def set_gyro_heading(self, heading: Rotation2d) -> None:
        """Set the current gyro heading for vision validation.

        Call this each cycle before get_measurements().
        """
        self._gyro_heading = heading

    def get_measurements(self) -> list[VisionMeasurement]:
        """Get all valid vision measurements from this cycle.

        Applies gyro consistency check (single-tag only) and cross-camera
        validation, then returns all surviving measurements for Kalman filter fusion.

        Returns:
            List of VisionMeasurements from all cameras that pass validation.
        """
        valid = [m for m in self._latest_measurements if m is not None]

        # Gyro consistency filter (single-tag only)
        valid = [m for m in valid if self._passes_gyro_check(m)]

        # Cross-camera outlier detection (penalizes std_devs, does not reject)
        valid = self._apply_cross_camera_validation(valid)

        self._nt.putNumber("MeasurementsReturned", len(valid))
        camera_names = ", ".join(m.camera_name for m in valid) if valid else "none"
        self._nt.putString("ActiveCameras", camera_names)

        # Driver-facing lock indicator. 0 = NONE, 1 = SINGLE, 2 = MULTI.
        # Computed from `valid` so it reflects only measurements the drivetrain trusts.
        if not valid:
            lock_quality = 0
        elif any(m.tag_count >= 2 for m in valid):
            lock_quality = 2
        else:
            lock_quality = 1
        SmartDashboard.putNumber("Vision/LockQuality", lock_quality)
        SmartDashboard.putBoolean("Vision/HasLock", lock_quality > 0)
        SmartDashboard.putString(
            "Vision/LockStatus",
            {0: "NO LOCK", 1: "SINGLE TAG", 2: "MULTI TAG"}[lock_quality],
        )

        return valid

    def _passes_gyro_check(self, measurement: VisionMeasurement) -> bool:
        """Check if a single-tag measurement's rotation is consistent with the gyro.

        Multi-tag measurements always pass (they don't suffer from rotation flips,
        and must pass on startup before the gyro is corrected).

        While disabled, all measurements pass so vision can correct the pose
        before the gyro is trustworthy.
        """
        if not DriverStation.isEnabled():
            return True

        # Multi-tag always passes
        if measurement.tag_count >= 2:
            return True

        if self._gyro_heading is None:
            return True

        # Use lenient threshold until vision has provided enough initial corrections
        if self._accepted_measurement_count < self.STARTUP_MEASUREMENT_THRESHOLD:
            threshold = self.STARTUP_GYRO_ROTATION_DIFF_RADIANS
        else:
            threshold = self.MAX_GYRO_ROTATION_DIFF_RADIANS

        vision_rotation = measurement.pose.toPose2d().rotation()
        diff = abs((vision_rotation - self._gyro_heading).radians())
        # Normalize to [0, pi]
        if diff > math.pi:
            diff = 2 * math.pi - diff

        if diff > threshold:
            self._nt.getSubTable(measurement.camera_name).putString(
                "GyroReject",
                f"diff={math.degrees(diff):.1f}deg",
            )
            return False
        return True

    def _apply_cross_camera_validation(self, measurements: list[VisionMeasurement]) -> list[VisionMeasurement]:
        """Penalize outlier cameras when multiple cameras disagree.

        When 2+ cameras have valid measurements, compute the median X and Y.
        If any camera disagrees by more than the threshold, inflate its std_devs.
        """
        if len(measurements) < 2:
            return measurements

        xs = sorted(m.pose.toPose2d().X() for m in measurements)
        ys = sorted(m.pose.toPose2d().Y() for m in measurements)
        median_x = xs[len(xs) // 2]
        median_y = ys[len(ys) // 2]

        result = []
        for m in measurements:
            pose_2d = m.pose.toPose2d()
            dist_from_median = math.hypot(pose_2d.X() - median_x, pose_2d.Y() - median_y)

            if dist_from_median > self.CROSS_CAMERA_MAX_DISAGREEMENT_METERS:
                penalty = self.CROSS_CAMERA_STD_DEV_PENALTY_MULTIPLIER
                penalized = VisionMeasurement(
                    pose=m.pose,
                    timestamp=m.timestamp,
                    ambiguity=m.ambiguity,
                    camera_name=m.camera_name,
                    tag_count=m.tag_count,
                    avg_tag_distance=m.avg_tag_distance,
                    std_devs=(m.std_devs[0] * penalty, m.std_devs[1] * penalty, m.std_devs[2] * penalty),
                )
                self._nt.getSubTable(m.camera_name).putString(
                    "CrossCamStatus",
                    f"OUTLIER dist={dist_from_median:.2f}m",
                )
                result.append(penalized)
            else:
                self._nt.getSubTable(m.camera_name).putString("CrossCamStatus", "OK")
                result.append(m)

        return result

    def _calculate_std_devs(self, tag_count: int, avg_distance: float) -> tuple[float, float, float]:
        """Calculate standard deviations based on tag count and distance.

        Args:
            tag_count: Number of AprilTags used in the estimate.
            avg_distance: Average distance to the tags in meters.

        Returns:
            Tuple of (x_std_dev, y_std_dev, rotation_std_dev).
        """
        # Base std devs scale linearly with distance
        xy_std = self.BASE_XY_STD_DEV + (avg_distance * self.XY_STD_DEV_PER_METER)
        rot_std = self.BASE_ROT_STD_DEV + (avg_distance * self.ROT_STD_DEV_PER_METER)

        # Multi-tag estimates are more reliable - reduce std devs
        if tag_count >= 2:
            xy_std /= self.MULTI_TAG_STD_DEV_DIVISOR
            rot_std /= self.MULTI_TAG_STD_DEV_DIVISOR

            # Extra bonus for 3+ tags
            if tag_count >= 3:
                xy_std /= 1.5
                rot_std /= 1.5

        return (xy_std, xy_std, rot_std)

    def _is_valid_measurement(self, pose: Pose3d, ambiguity: float, tag_count: int, avg_distance: float) -> bool:
        """Check if a vision measurement is valid and should be used.

        Args:
            pose: The estimated robot pose.
            ambiguity: Pose ambiguity (0-1).
            tag_count: Number of tags used.
            avg_distance: Average distance to tags.

        Returns:
            True if the measurement passes all validation checks.
        """
        is_multi_tag = tag_count >= 2

        # Check ambiguity threshold
        max_ambiguity = self.MAX_POSE_AMBIGUITY_MULTI_TAG if is_multi_tag else self.MAX_POSE_AMBIGUITY_SINGLE_TAG
        if ambiguity > max_ambiguity:
            self._nt.putString("RejectReason", f"ambiguity {ambiguity:.3f} > {max_ambiguity}")
            return False

        # Check distance threshold (stricter for single-tag)
        max_distance = self.MAX_MULTI_TAG_DISTANCE if is_multi_tag else self.MAX_SINGLE_TAG_DISTANCE
        if avg_distance > max_distance:
            self._nt.putString("RejectReason", f"distance {avg_distance:.2f} > {max_distance}")
            return False

        # Check if pose is on the field (basic sanity check)
        pose_2d = pose.toPose2d()
        if pose_2d.X() < -2.0 or pose_2d.X() > 18.0:  # Field is ~16.5m long
            self._nt.putString("RejectReason", f"X out of bounds: {pose_2d.X():.2f}")
            return False
        if pose_2d.Y() < -2.0 or pose_2d.Y() > 10.0:  # Field is ~8m wide
            self._nt.putString("RejectReason", f"Y out of bounds: {pose_2d.Y():.2f}")
            return False

        return True

    def _get_tag_distances(self, result: PhotonPipelineResult, robot_to_camera: Transform3d) -> tuple[int, float]:
        """Get the number of tags and average distance to them.

        Args:
            result: PhotonPipelineResult from the camera.
            robot_to_camera: Transform from robot to camera.

        Returns:
            Tuple of (tag_count, average_distance_meters).
        """
        if not result.hasTargets():
            return (0, 0.0)

        targets = result.getTargets()
        tag_count = len(targets)

        if tag_count == 0:
            return (0, 0.0)

        # Calculate average distance to all visible tags
        total_distance = 0.0
        for target in targets:
            # Get the transform from camera to tag
            camera_to_target = target.bestCameraToTarget
            # Distance is the translation magnitude
            distance = math.sqrt(camera_to_target.X() ** 2 + camera_to_target.Y() ** 2 + camera_to_target.Z() ** 2)
            total_distance += distance

        avg_distance = total_distance / tag_count
        return (tag_count, avg_distance)

    def _estimate_pose(self, estimator: PhotonPoseEstimator, result: PhotonPipelineResult, tag_count: int):
        """Estimate robot pose using the appropriate strategy.

        Uses multi-tag PNP when 2+ tags visible, falls back to lowest ambiguity single-tag.

        Args:
            estimator: The PhotonPoseEstimator for this camera.
            result: PhotonPipelineResult from the camera.
            tag_count: Number of tags visible.

        Returns:
            EstimatedRobotPose or None if estimation failed.
        """
        if tag_count >= 2:
            # Try multi-tag first (most accurate)
            estimated = estimator.estimateCoprocMultiTagPose(result)
            if estimated is not None:
                return estimated

        # Fall back to single-tag lowest ambiguity
        return estimator.estimateLowestAmbiguityPose(result)

    def execute(self) -> None:
        """Called by MagicBot every loop iteration.

        Updates all camera pose estimates and stores valid measurements.
        """
        # DEBUG: Confirm execute() is running and cameras are initialized
        self._nt.putBoolean("ExecuteRunning", True)
        self._nt.putNumber("CameraCount", len(self._cameras))

        # Process each camera
        for i, (camera, estimator, robot_to_camera, name) in enumerate(self._cameras):
            self._latest_measurements[i] = None

            # Get the latest result from the camera
            results = camera.getAllUnreadResults()

            # DEBUG: Log result count per camera
            self._nt.getSubTable(name).putNumber("ResultCount", len(results))

            for result in results:
                # Get tag count and distance info
                tag_count, avg_distance = self._get_tag_distances(result, robot_to_camera)

                self._nt.getSubTable(name).putNumber("TagsSeen", tag_count)

                if tag_count == 0:
                    continue

                # Estimate pose using appropriate strategy
                is_multi_tag = tag_count >= 2
                estimated_pose = self._estimate_pose(estimator, result, tag_count)

                cam_nt = self._nt.getSubTable(name)
                cam_nt.putBoolean("EstimateSuccess", estimated_pose is not None)
                cam_nt.putBoolean("IsMultiTag", is_multi_tag)

                if estimated_pose is None:
                    continue

                # Get the best target's ambiguity for filtering
                ambiguity = 0.0
                best_target = result.getBestTarget()
                if best_target is not None:
                    ambiguity = best_target.poseAmbiguity

                # DEBUG: Log pre-validation values
                pose = estimated_pose.estimatedPose
                cam_nt.putNumber("RawPoseX", pose.toPose2d().X())
                cam_nt.putNumber("RawPoseY", pose.toPose2d().Y())
                cam_nt.putNumber("Ambiguity", ambiguity)
                cam_nt.putNumber("AvgDist", avg_distance)
                cam_nt.putNumber("Timestamp", estimated_pose.timestampSeconds)

                # DEBUG: Dig into timestamp components
                cam_nt.putNumber("NtReceiveMicros", result.ntReceiveTimestampMicros)
                cam_nt.putNumber("PublishMicros", result.metadata.publishTimestampMicros)
                cam_nt.putNumber("CaptureMicros", result.metadata.captureTimestampMicros)
                cam_nt.putNumber("PipelineLatencyMs", result.getLatencyMillis())

                # Validate the measurement
                valid = self._is_valid_measurement(pose, ambiguity, tag_count, avg_distance)
                cam_nt.putBoolean("ValidationPass", valid)
                if not valid:
                    continue

                # Calculate distance-based standard deviations
                std_devs = self._calculate_std_devs(tag_count, avg_distance)

                # Use FPGA time minus pipeline latency since NT4 time sync is broken
                corrected_timestamp = Timer.getFPGATimestamp() - (result.getLatencyMillis() / 1000.0)

                measurement = VisionMeasurement(
                    pose=pose,
                    timestamp=corrected_timestamp,
                    ambiguity=ambiguity,
                    camera_name=name,
                    tag_count=tag_count,
                    avg_tag_distance=avg_distance,
                    std_devs=std_devs,
                )

                self._latest_measurements[i] = measurement
                self._accepted_measurement_count += 1

        # Update dashboard with camera status
        valid_count = sum(1 for m in self._latest_measurements if m is not None)
        self._nt.putNumber("ValidCameras", valid_count)
        self._nt.putNumber("TotalCameras", len(self._cameras))

        # Log detailed info for debugging
        for m in self._latest_measurements:
            if m is not None:
                sub = self._nt.getSubTable(m.camera_name)
                sub.putNumber("TagCount", m.tag_count)
                sub.putNumber("AvgDistance", m.avg_tag_distance)
                sub.putNumber("StdDevXY", m.std_devs[0])
