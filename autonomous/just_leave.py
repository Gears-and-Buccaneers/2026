"""A simple auto mode that drives forward to leave the start area."""

import magicbot as mb

import components


class JustLeavePlease(mb.AutonomousStateMachine):
    """A state machine that waits, then drives, then stops."""

    # Must set a name here for this to be recognized as an auto mode.
    MODE_NAME = "Just Leave"

    # Set this as the default auto mode. [GK: Ick; what if two files have this?]
    DEFAULT = False

    drivetrain: components.Drivetrain

    # Start here, and do nothing for 5 seconds.
    @mb.timed_state(duration=5, next_state="gogogo", first=True)
    def wait(self):
        """Give other robots time to do their autos."""
        pass

    # …then do this for 3 seconds (and then do nothing more)
    @mb.timed_state(duration=3)
    def gogogo(self):
        """Drive forward for a bit."""
        # FIXME: a hardcoded speed does not belong here; probably should be tunable
        self.drivetrain.drive(forward_speed=1)
