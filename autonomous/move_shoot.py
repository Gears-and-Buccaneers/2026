"""Autos that all follow the pattern of: run a choreo trajectory, then shoot."""
# Don't require docstrings for every state name.
# ruff: noqa: D102

import magicbot as mb
import wpimath.geometry as geom

import components
from autonomous.choreo_auto import ChoreoStateMachine


class MoveShoot(ChoreoStateMachine):
    """Base class that handles starting by running a trajectory, and then shooting."""

    MODE_NAME = "(Placeholder)"
    DISABLED = True  # Override this in subclasses to enable them
    TRAJECTORY = "some_traj_name"

    drivetrain: components.Drivetrain
    pewpew: components.Shooter
    intake: components.Intake

    def setup(self):
        self._target_heading: None | geom.Rotation2d = None

    def _prepare_to_fire(self):
        """Turn to face the hub and spin up the shooter."""
        solution = self.pewpew.calculateShootingSolution(self.drivetrain.getVelocity())
        self.pewpew.shooterMode = "auto"
        self.pewpew.setTargetMuzzleSpeed(solution.muzzleSpeed)
        self._target_heading = solution.targetHeading
        self.intake.activelyTransit = True
        self.drivetrain.driveFacingAngle(targetAngle=self._target_heading)

    def on_enable(self) -> None:
        """Called when the auto mode is enabled."""
        super().on_enable()
        self.robot_is_placed_at_start_of(self.TRAJECTORY)

    @mb.state(first=True)
    def move_trajectory(self, initial_call: bool):
        if initial_call:
            # Stop actively shooting while we drive
            self.pewpew.spinDown()
            self._target_heading = None  # Clear the target heading so we can re-calculate it for the next shot
            self.run_trajectory(
                self.TRAJECTORY,
                event_callbacks={
                    "StartIntake": self.intake.ingest,
                    "StopIntake": self.intake.stop,
                    "ExtendIntake": self.intake.extend,
                    "IntakeExtend": self.intake.extend,
                },
            )
        elif self.is_trajectory_done():
            self.next_state("prep2")

    @mb.timed_state(duration=2, next_state="shoot_2")
    def prep2(self):
        self._prepare_to_fire()
        if self.pewpew.isReadyToFire():
            self.next_state("shoot_2")

    @mb.timed_state(duration=10, next_state="finished")
    def shoot_2(self):
        self.pewpew.activelyShoot = True
        # TODO: if we can count

    @mb.state
    def finished(self):
        self.pewpew.spinDown()
        if not self.intake.isFullyRetracted():
            self.intake.retract()


class LeftTrenchToGatherNeutralShootFromLeft(MoveShoot):
    """Starts in left trench, shoots, gathers from neutral zone, returns to left trench to shoot."""

    MODE_NAME = "1x LeftNeutralLeft"
    TRAJECTORY = "LeftTrench_Twice"
    DISABLED = False


class LeftTrenchToGatherNeutralShootFromRight(MoveShoot):
    """Starts in left trench, gathers from neutral zone, moves to right trench to shoot."""

    MODE_NAME = "1x LeftNeutralRight"
    TRAJECTORY = "LeftTrench_Gather_RightTrench"
    DISABLED = False


class RightTrenchToGatherNeutralShootFromLeft(MoveShoot):
    """Starts in right trench, gathers from neutral zone, returns to left trench to shoot."""

    MODE_NAME = "1x RightNeutralLeft"
    TRAJECTORY = "RightToLeft"
    DISABLED = False


class RightTrenchToGatherNeutralShootFromRight(MoveShoot):
    """Starts in right trench, gathers from neutral zone, moves to right trench to shoot."""

    MODE_NAME = "1x RightNeutralRight"
    TRAJECTORY = "RightNeutralRight"
    DISABLED = False


class MiddleToGatherDepotShootFromCorner(MoveShoot):
    """Starts in left trench, shoots, gathers from neutral zone, enters right trench and shoots."""

    MODE_NAME = "1x MiddleToDepotCorner"
    TRAJECTORY = "Middle_To_Depot"
    DISABLED = False


class RightBumpCollect(MoveShoot):
    """Start at right bump, collect balls from neutral zone, return over bump and shoot."""

    MODE_NAME = "1x RightBumpNeutralRight"
    TRAJECTORY = "RightBump_Collect"
    DISABLED = False


class CenterBackupShoot(MoveShoot):
    """Start centered, backup to left of ladder, shoot preloaded."""

    MODE_NAME = "1x CenterBackupShoot"
    TRAJECTORY = "CenterBackupShoot"
    DISABLED = False


class LeftBumpCollect(MoveShoot):
    """Start at left bump, collect balls from neutral zone, return over bump and shoot."""

    MODE_NAME = "1x LeftBumpNeutralLeft"
    TRAJECTORY = "LeftBump_Collect"
    DISABLED = False
