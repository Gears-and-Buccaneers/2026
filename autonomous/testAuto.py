"""Example Choreo autonomous modes.

These are example implementations showing how to use the ChoreoAuto base class.
Copy and modify these for your own autonomous routines!

To create a new Choreo auto:
1. Design your path in the Choreo GUI (https://choreo.autos)
2. Save the trajectory to deploy/choreo/your_trajectory.traj
3. Create a new class that extends ChoreoAuto
4. Set MODE_NAME, TRAJECTORY_NAME, and DISABLED = False
5. Optionally override on_trajectory_start() and on_trajectory_end()
"""

import time

from autonomous.choreo_auto import ChoreoAuto, ChoreoMultiTrajectoryAuto


class SimpleChoreoAuto(ChoreoAuto):
    """A simple auto that follows a single Choreo trajectory.

    This auto mode loads a trajectory called "simple_path" from
    deploy/choreo/simple_path.traj and follows it.

    To use this:
    1. Open Choreo and create a path called "simple_path"
    2. Generate the trajectory and save to deploy/choreo/simple_path.traj
    3. Select "Simple Choreo Path" on the driver station
    """

    # You can add other components here that will be injected by MagicBot
    # shooter: components.Shooter

    def on_trajectory_start(self) -> None:
        """Called when the trajectory starts.

        Use this to start mechanisms at the beginning of auto.
        Examples: start spinning up shooter, deploy intake, etc.
        """
        # Example: self.shooter.spin_up()
        pass

    def on_trajectory_end(self) -> None:
        """Called when the trajectory ends.

        Use this to perform actions after the robot stops moving.
        Examples: shoot a game piece, retract intake, etc.
        """
        # Example: self.shooter.shoot()
        pass


# Uncomment and modify this example once you have multiple trajectories


class TwoPieceChoreoAuto(ChoreoMultiTrajectoryAuto):
    """Example of chaining multiple trajectories together.

    This auto mode:
    1. Drives from start to first game piece
    2. Intakes the piece
    3. Drives to scoring position
    4. Scores the piece
    5. Drives to second game piece
    6. Intakes and scores again
    """

    MODE_NAME = "PID and Tuning Test"
    DISABLED = False  # Enable this auto mode (base class is disabled by default)
    DEFAULT = True

    # Add components that will be injected

    def setup_trajectories(self):
        """Define the sequence of trajectories and actions."""
        return [
            ("forthOneMeter", self.wait),
            ("backOneMeter", self.null),
        ]

    def wait(self):
        """Called after reaching each game piece position."""
        time.sleep(5)

    def null(self):
        """Called after reaching the scoring position."""
        pass
