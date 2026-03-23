"""Fire from the left trench, pick up pieces from nuetral zone, and score again from the trench."""

import time

import components
from autonomous.choreo_auto import ChoreoMultiTrajectoryAuto, EventCallbacks, TrajectoriesAndCallbacks
from robot import Scurvy


class OutAndBack(ChoreoMultiTrajectoryAuto):
    """Example of chaining multiple trajectories together.

    This auto mode:
    1. Runs the first trajectory—which just places the bot.
    2. Shoots into the hub from the trench.
    3. Drives out to the neutral zone to pick up pieces.
    4. Drives back to the left trench and fires again.
    """

    MODE_NAME = "Left Trench Out and Back"
    DISABLED = True  # We're not using this, but it's a reasonable example of ChoreoMultiTrajectoryAuto

    scurvy: Scurvy
    drivetrain: components.Drivetrain
    pewpew: components.Shooter
    intake: components.Intake

    def setup_trajectories(self) -> TrajectoriesAndCallbacks:
        """Define the sequence of trajectories and actions."""
        return [
            ("OutAndBack[0]", self.shoot),  # Place the bot in the left trench, then shoot
            ("OutAndBack[1]", self.shoot),  # Go pick up pieces from the neutral zone, come back, shoot again
        ]

    def setup_event_callbacks(self) -> EventCallbacks:
        """Define callbacks for events in the trajectories."""
        return {
            "RunIntake": self.run_intake,
            "StopIntake": self.stop_intake,
            "ExtendIntake": self.extend_intake,
        }

    def shoot(self):
        """Shoot into the hub from the current position."""
        print("SHOOOOOOOTING!!!!!!!!!!!!")
        # Get shooting solution accounting for robot velocity (presumably stopped)
        self.scurvy.dynamicallyTargetHub()

        time.sleep(0.4)  # Wait for the shooter to get up to speed, and the robot to turn to face the hub
        self.pewpew.shoot()  # Fire the kicker to shoot the piece

    def run_intake(self):
        """Run the intake rollers to pick up pieces."""
        print("RUNNING INTAKE!!!!!!!!!!!! RUN!!!!")
        self.intake.ingest()  # Start the intake rollers to pick up fuel

    def stop_intake(self):
        """Stop the intake rollers."""
        print("STOPPING INTAKE!!!!!!!!!!!! STOP!!!!")
        self.intake.stop()  # Stop the intake rollers

    def extend_intake(self):
        """Extend the intake mechanism."""
        print("EXTENDING INTAKE!!!!!!!!!!!! EXTEND!!!!")
        self.intake.extend()  # Lower the intake mechanism to the field
