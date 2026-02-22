"""All the auto modes."""

import math

from phoenix6 import units
from wpimath.geometry import Pose2d, Rotation2d

FIELD_LENGTH: units.meter = 16.541  # 651.22 inches
FIELD_WIDTH: units.meter = 8.069  # 317.69 inches


def mirrorPose(pose: Pose2d) -> Pose2d:
    """Rotationally mirror a pose around the field center."""
    return Pose2d(
        FIELD_LENGTH - pose.X(),
        FIELD_WIDTH - pose.Y(),
        Rotation2d(math.pi - pose.rotation().radians()),
    )
