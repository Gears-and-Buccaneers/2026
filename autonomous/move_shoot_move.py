"""Autos that all follow the pattern of start at a location, shoot, run a choreo trajectory, shoot."""
# Don't require docstrings for every state name.
# ruff: noqa: D102

import magicbot as mb
import wpimath.geometry as geom

import components
from autonomous.choreo_auto import ChoreoStateMachine


class MoveShootMove(ChoreoStateMachine):
    """Base class that handles shooting, running a trajectory, and then shooting again."""

    MODE_NAME = "(Placeholder)"
    DISABLED = True  # Override this in subclasses to enable them
    TRAJECTORY1 = "some_traj_name"
    TRAJECTORY2 = "some_traj_name"

    drivetrain: components.Drivetrain
    pewpew: components.Shooter
    intake: components.Intake

    def setup(self):
        self._target_heading: None | geom.Rotation2d = None

    def _prepare_to_fire(self):
        """Turn to face the hub and spin up the shooter."""
        self._target_heading = self.pewpew.spinUpAndTargetHub()
        self.drivetrain.driveFacingAngle(targetAngle=self._target_heading)

    def on_enable(self) -> None:
        """Called when the auto mode is enabled."""
        super().on_enable()
        self.robot_is_placed_at_start_of(self.TRAJECTORY1)

    # This is only a timed state as a fallback, in case the robot never thinks it's ready to fire.
    # When all is well, the robot should quickly transition to shoot_1 as soon as it's ready to fire.
    @mb.timed_state(duration=0.5, next_state="shoot_1")
    def prep1(self):
        self._prepare_to_fire()
        # if self.pewpew.isReadyToFire():
        #    self.next_state("shoot_1")

    # Shoot for…long enough to probably shoot all pieces.
    @mb.timed_state(duration=2, next_state="move_trajectory2")
    def shoot_1(self, initial_call: bool):
        self.pewpew.activelyShoot = True
        # TODO: if we can use shooter speed sag to detect a shot, leave when we've shot all 8
        # if initial_call:
        #     self.pewpew.piecesShot = 0
        # if self.pewpew.piecesShot >= 8:
        #     self.next_state("move_trajectory")

    @mb.timed_state(duration=0.75, next_state="move_trajectory")
    def extend_intake(self, initial_call: bool):
        if initial_call:
            self.intake.extend()

    @mb.state(first=True)
    def move_trajectory1(self, initial_call: bool):
        if initial_call:
            # Stop actively shooting while we drive
            self._target_heading = None  # Clear the target heading so we can re-calculate it for the next shot
            self.run_trajectory(
                self.TRAJECTORY1,
                event_callbacks={
                    "StartIntake": self.intake.ingest,
                    "StopIntake": self.intake.stop,
                    "ExtendIntake": self.intake.extend,
                },
            )
        elif self.is_trajectory_done():
            self.next_state("prep1")

    @mb.state
    def move_trajectory2(self, initial_call: bool):
        if initial_call:
            # Stop actively shooting while we drive
            self._target_heading = None  # Clear the target heading so we can re-calculate it for the next shot
            self.run_trajectory(
                self.TRAJECTORY2,
                event_callbacks={
                    "StartIntake": self.intake.ingest,
                    "StopIntake": self.intake.stop,
                    "ExtendIntake": self.intake.extend,
                },
            )
        elif self.is_trajectory_done():
            self.next_state("finished")

    @mb.timed_state(duration=2, next_state="shoot_2")
    def prep2(self):
        self._prepare_to_fire()
        if self.pewpew.isReadyToFire():
            self.next_state("shoot_2")

    @mb.timed_state(duration=4, next_state="finished")
    def shoot_2(self):
        self.pewpew.activelyShoot = True
        # TODO: if we can count

    @mb.state
    def finished(self):
        self.pewpew.spinDown()
        if not self.intake.isFullyRetracted():
            self.intake.retract()


class CenterBackupShoot(MoveShootMove):
    """Starts in left trench, shoots, gathers from neutral zone, returns to left trench to shoot."""

    MODE_NAME = "Center Backup Shoot"
    TRAJECTORY1 = "CenterBackupShoot"
    TRAJECTORY2 = "CenterForward"
    DISABLED = False
