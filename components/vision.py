"""Vision subsystem using PhotonVision for AprilTag pose estimation.

This module handles multiple PhotonVision cameras and provides pose estimates
to the drivetrain for sensor fusion with distance-based confidence scaling.
"""

import math
from dataclasses import dataclass

from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.targeting import PhotonPipelineResult
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import RobotBase, SmartDashboard
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
            const.VisionConfig.FRONT_CAMERA_TRANSFORM,
        )
        self._setup_camera(
            const.VisionConfig.BACK_LEFT_CAMERA_NAME,
            const.VisionConfig.BACK_LEFT_CAMERA_TRANSFORM,
        )
        self._setup_camera(
            const.VisionConfig.BACK_RIGHT_CAMERA_NAME,
            const.VisionConfig.BACK_RIGHT_CAMERA_TRANSFORM,
        )

        # Store latest measurements for each camera
        self._latest_measurements: list[VisionMeasurement | None] = [None] * len(self._cameras)

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

    def update_sim(self, robot_pose: Pose3d) -> None:
        """Update the vision simulation with the robot's current pose.

        Call this every loop when running in simulation.

        Args:
            robot_pose: The robot's current pose (from drivetrain).
        """
        if self._vision_sim is not None:
            self._vision_sim.update(robot_pose)

    def get_measurements(self) -> list[VisionMeasurement]:
        """Get all valid vision measurements from this cycle.

        Returns:
            List of VisionMeasurement objects with pose, timestamp, and metadata.
            Only includes measurements that pass validation checks.
        """
        return [m for m in self._latest_measurements if m is not None]

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

        # Check ambiguity threshold (stricter for single-tag)
        max_ambiguity = self.MAX_POSE_AMBIGUITY_MULTI_TAG if is_multi_tag else self.MAX_POSE_AMBIGUITY_SINGLE_TAG
        if ambiguity > max_ambiguity:
            return False

        # Check distance threshold (stricter for single-tag)
        max_distance = self.MAX_MULTI_TAG_DISTANCE if is_multi_tag else self.MAX_SINGLE_TAG_DISTANCE
        if avg_distance > max_distance:
            return False

        # Check if pose is on the field (basic sanity check)
        pose_2d = pose.toPose2d()
        if pose_2d.X() < -1.0 or pose_2d.X() > 17.0:  # Field is ~16.5m long
            return False
        if pose_2d.Y() < -1.0 or pose_2d.Y() > 9.0:  # Field is ~8m wide
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
        # Process each camera
        for i, (camera, estimator, robot_to_camera, name) in enumerate(self._cameras):
            self._latest_measurements[i] = None

            # Get the latest result from the camera
            results = camera.getAllUnreadResults()

            for result in results:
                # Get tag count and distance info
                tag_count, avg_distance = self._get_tag_distances(result, robot_to_camera)

                if tag_count == 0:
                    continue

                # Estimate pose using appropriate strategy
                estimated_pose = self._estimate_pose(estimator, result, tag_count)

                if estimated_pose is None:
                    continue

                # Get the best target's ambiguity for filtering
                ambiguity = 0.0
                best_target = result.getBestTarget()
                if best_target is not None:
                    ambiguity = best_target.poseAmbiguity

                # Validate the measurement
                if not self._is_valid_measurement(estimated_pose.estimatedPose, ambiguity, tag_count, avg_distance):
                    continue

                # Calculate distance-based standard deviations
                std_devs = self._calculate_std_devs(tag_count, avg_distance)

                measurement = VisionMeasurement(
                    pose=estimated_pose.estimatedPose,
                    timestamp=estimated_pose.timestampSeconds,
                    ambiguity=ambiguity,
                    camera_name=name,
                    tag_count=tag_count,
                    avg_tag_distance=avg_distance,
                    std_devs=std_devs,
                )

                self._latest_measurements[i] = measurement

        # Update dashboard with camera status
        valid_count = sum(1 for m in self._latest_measurements if m is not None)
        SmartDashboard.putNumber("Vision/ValidCameras", valid_count)
        SmartDashboard.putNumber("Vision/TotalCameras", len(self._cameras))

        # Log detailed info for debugging
        for m in self._latest_measurements:
            if m is not None:
                SmartDashboard.putNumber(f"Vision/{m.camera_name}/TagCount", m.tag_count)
                SmartDashboard.putNumber(f"Vision/{m.camera_name}/AvgDistance", m.avg_tag_distance)
                SmartDashboard.putNumber(f"Vision/{m.camera_name}/StdDevXY", m.std_devs[0])
