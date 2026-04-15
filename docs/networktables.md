# NetworkTables reference

Published keys for debugging, tuning, and dashboards. MagicBot tunables and `@feedback` values appear under `/components/{attribute}/` (component field name on the robot class) and `/robot/` (robot class). Other keys use explicit tables below.

## `/components/pewpew/` (shooter)

| Key                         | Kind     | Description                                            |
| --------------------------- | -------- | ------------------------------------------------------ |
| `topRPS6ft`                 | tunable  | Top flywheel RPS at 6 ft reference distance.           |
| `bottomRPS6ft`              | tunable  | Bottom flywheel RPS at 6 ft.                           |
| `topRPS20ft`                | tunable  | Top flywheel RPS at 20 ft reference distance.          |
| `bottomRPS20ft`             | tunable  | Bottom flywheel RPS at 20 ft.                          |
| `fallbackTopRPS`            | tunable  | Top RPS when using fixed-distance (fallback) spin.     |
| `fallbackBottomRPS`         | tunable  | Bottom RPS for fallback spin.                          |
| `transitKickerSpeed`        | tunable  | Transit motor output when feeding (0–1).               |
| `manualUnshootTransitScale` | tunable  | Scale for reverse transit in manual unshoot.           |
| `maxHeadingError`           | tunable  | Max heading error (deg) to allow shooting in auto aim. |
| `flywheelSpeedTolerance`    | tunable  | Fractional speed tolerance for “at speed”.             |
| `TopRPSTarget`              | feedback | Commanded top wheel RPS.                               |
| `TopRPSActual`              | feedback | Measured top wheel RPS.                                |
| `TopRPSError`               | feedback | Top RPS target minus actual.                           |
| `BottomRPSTarget`           | feedback | Commanded bottom wheel RPS.                            |
| `BottomRPSActual`           | feedback | Measured bottom wheel RPS.                             |
| `BottomRPSError`            | feedback | Bottom RPS target minus actual.                        |
| `AtSpeed`                   | boolean  | True when flywheels are within tolerance.              |

## `/components/drivetrain/` (swerve)

| Key              | Kind                   | Description                                                                       |
| ---------------- | ---------------------- | --------------------------------------------------------------------------------- |
| `headingDegrees` | feedback               | Robot heading in degrees.                                                         |
| `poseX`          | feedback               | Estimated field X (m).                                                            |
| `poseY`          | feedback               | Estimated field Y (m).                                                            |
| `pose3d`         | struct (`Transform3d`) | Full 3D transform from field origin to robot (includes pitch/roll on real robot). |

### `/components/drivetrain/VisionInput/`

Telemetry for each vision measurement fused into odometry (last fused sample per call).

| Key                | Description                                    |
| ------------------ | ---------------------------------------------- |
| `FPGA_Now`         | RoboRIO time when measurement was applied (s). |
| `MeasureTimestamp` | Vision sample timestamp (s).                   |
| `Latency`          | `FPGA_Now - MeasureTimestamp` (s).             |
| `StdDevX`          | Vision std dev on X (m).                       |
| `StdDevY`          | Vision std dev on Y (m).                       |
| `StdDevRot`        | Vision std dev on rotation (rad).              |
| `InputPoseX`       | Fused pose X from vision (m).                  |
| `InputPoseY`       | Fused pose Y from vision (m).                  |

## `/components/intake/`

| Key                        | Kind     | Description                                  |
| -------------------------- | -------- | -------------------------------------------- |
| `intakeSpeed`              | tunable  | Roller speed (sign = direction).             |
| `intakeExtendSpeed`        | tunable  | Extension motor speed when extending.        |
| `intakeRetractSpeed`       | tunable  | Extension motor speed when retracting.       |
| `intakeRetractedRotations` | tunable  | Encoder position considered fully retracted. |
| `intakeExtendedRotations`  | tunable  | Encoder position considered fully extended.  |
| `intakeToleranceRotations` | tunable  | Position tolerance for “at setpoint”.        |
| `extensionPosition`        | feedback | Current extension position (rotations).      |

## `/components/vision/`

Global vision debug (PhotonVision fusion pipeline).

| Key                    | Description                                                          |
| ---------------------- | -------------------------------------------------------------------- |
| `ExecuteRunning`       | True while `Vision.execute()` runs.                                  |
| `CameraCount`          | Number of configured cameras.                                        |
| `MeasurementsReturned` | Valid measurements after filters (same cycle as `get_measurements`). |
| `MeasurementsFed`      | Count passed to drivetrain fusion this loop.                         |
| `ActiveCameras`        | Comma-separated names of cameras with valid measurements.            |
| `ValidCameras`         | Number of cameras with a non-None measurement this cycle.            |
| `TotalCameras`         | Same as camera count.                                                |
| `RejectReason`         | Last validation failure reason for a rejected pose (string).         |

### `/components/vision/<camera_name>/`

Per-camera keys (names match PhotonVision camera names, e.g. `back_right_camera`, `front_camera`, `back_left_camera`).

| Key                               | Description                                                |
| --------------------------------- | ---------------------------------------------------------- |
| `ResultCount`                     | Photon results read this loop.                             |
| `TagsSeen`                        | Tags visible in current result.                            |
| `EstimateSuccess`                 | Pose estimator returned a pose.                            |
| `IsMultiTag`                      | Estimate used 2+ tags.                                     |
| `RawPoseX` / `RawPoseY`           | Estimated pose before validation (m).                      |
| `Ambiguity`                       | Best-target ambiguity (0–1).                               |
| `AvgDist`                         | Average camera-to-tag distance (m).                        |
| `Timestamp`                       | Estimate timestamp from Photon (s).                        |
| `NtReceiveMicros`                 | NT receive timestamp on result.                            |
| `PublishMicros` / `CaptureMicros` | Metadata timestamps.                                       |
| `PipelineLatencyMs`               | Reported pipeline latency (ms).                            |
| `ValidationPass`                  | Passed distance/ambiguity/bounds checks.                   |
| `GyroReject`                      | Set when single-tag rotation disagrees with gyro (string). |
| `CrossCamStatus`                  | `OK` or outlier note when multiple cameras disagree.       |
| `TagCount`                        | Tags used in stored measurement (post-validation).         |
| `AvgDistance`                     | Average tag distance in stored measurement (m).            |
| `StdDevXY`                        | XY std dev passed to fusion (m).                           |

## `/robot/` (Theseus)

Robot-level MagicBot tunables and feedbacks.

| Key                 | Kind     | Description                                               |
| ------------------- | -------- | --------------------------------------------------------- |
| `precisionSlowdown` | tunable  | Drivetrain scaling factor in precision mode.              |
| `hubIsActive`       | feedback | Whether alliance hub scoring window logic allows scoring. |
| `currentShift`      | feedback | Current teleop match segment / shift enum.                |

MagicBot may also publish standard robot metadata keys (e.g. mode, simulation flags) under `/robot/` depending on version.

## `/AdvantageScope/`

Struct topics for 3D field visualization (AdvantageScope).

| Topic            | Type     | Description                            |
| ---------------- | -------- | -------------------------------------- |
| `Hub`            | `Pose2d` | Alliance hub pose on the field.        |
| `ShooterPose`    | `Pose2d` | Ground launch point under the shooter. |
| `SmartAimTarget` | `Pose2d` | Debug aim target pose.                 |

## `/Simulation/` (simulation only)

| Topic       | Type       | Description                      |
| ----------- | ---------- | -------------------------------- |
| `RobotPose` | `Pose2d`   | Sim drivetrain pose for logging. |
| `Fuel`      | `Pose3d[]` | Simulated game piece poses.      |

## `/SmartDashboard/`

| Key               | Description                                                  |
| ----------------- | ------------------------------------------------------------ |
| `Field`           | `Field2d` sendable (robot pose on 2D field widget).          |
| `Auto Selector`   | String synced from autonomous chooser (see `robot.py`).      |
| `Autonomous Mode` | Sendable chooser for auto selection (read/write per WPILib). |

Other WPILib widgets may add keys under `/SmartDashboard/` as needed.
