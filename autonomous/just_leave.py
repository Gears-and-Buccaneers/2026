"""A simple auto mode that drives forward to leave the start area."""

import magicbot as mb
import phoenix6.units as units
from wpilib import DriverStation  # optional if you want alliance-based starts
from wpimath.geometry import Pose2d, Rotation2d

import autonomous
import components

START_POSE_BLUE = Pose2d(2, 2.5, Rotation2d.fromDegrees(0))
START_POSE_RED = autonomous.mirror_pose(START_POSE_BLUE)
DRIVE_SPEED: units.meters_per_second = 1.5


class JustLeavePlease(mb.AutonomousStateMachine):
    """A state machine that waits, then drives, then stops."""

    # Must set a name here for this to be recognized as an auto mode.
    MODE_NAME = "Just Leave"

    # Set this as the default auto mode. [GK: Ick; what if two files have this?]
    DEFAULT = False

    drivetrain: components.Drivetrain

    def on_enable(self):
        """Called when this auto starts."""
        super().on_enable()
        alliance = DriverStation.getAlliance()

        # FIXME: a hardcoded speed does not belong here; probably should be tunable
        self.x_speed: units.meters_per_second = DRIVE_SPEED

        if alliance == DriverStation.Alliance.kRed:
            self.drivetrain.reset_pose(START_POSE_RED)
            self.x_speed *= -1
        else:
            self.drivetrain.reset_pose(START_POSE_BLUE)

    # Start here, and do nothing for 5 seconds.
    @mb.timed_state(duration=5, next_state="gogogo", first=True)
    def wait(self):
        """Give other robots time to do their autos."""
        pass

    # …then do this for 3 seconds (and then do nothing more)
    @mb.timed_state(duration=3)
    def gogogo(self):
        """Drive towards the other alliance for a bit."""
        self.drivetrain.drive_field_centric(velocity_x=self.x_speed)
