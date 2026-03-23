"""Auto: start in the left trench, shoot, gather from neutral zone, return to left trench to shoot again."""
# Don't require docstrings for every state name.
# ruff: noqa: D102

import magicbot as mb
import wpimath.geometry as geom

import components
from autonomous.choreo_auto import ChoreoStateMachine


class LeftTrenchAndBack(ChoreoStateMachine):
    """Starts in left trench, shoots, gathers from neutral zone, returns to left trench to unload."""

    MODE_NAME = "Left Trench Twice"
    DISABLED = False  # Enable this auto mode

    drivetrain: components.Drivetrain
    pewpew: components.Shooter
    intake: components.Intake

    def setup(self):
        self._target_heading: None | geom.Rotation2d = None

    def _prepare_to_fire(self):
        """Turn to face the hub and spin up the shooter."""
        if self._target_heading is None:
            self.drivetrain.stop()
            solution = self.pewpew.calculateShootingSolution(self.drivetrain.getVelocity())
            self.pewpew.shooterMode = "auto"
            self.pewpew.setTargetMuzzleSpeed(solution.muzzleSpeed)
            self._target_heading = solution.targetHeading
        self.intake.activelyTransit = True
        self.drivetrain.driveFacingAngle(targetAngle=self._target_heading)

    @mb.state(first=True)
    def place_bot(self, initial_call: bool):
        if initial_call:
            self.run_trajectory("LeftTrenchTwice[0]")
        elif self.is_trajectory_done():
            self.next_state("prep1")

    # This is only a timed state as a fallback, in case the robot never thinks it's ready to fire.
    # When all is well, the robot should quickly transition to shoot_1 as soon as it's ready to fire.
    @mb.timed_state(duration=2, next_state="shoot_1")
    def prep1(self):
        self._prepare_to_fire()
        if self.pewpew.isReadyToFire():
            self.next_state("shoot_1")

    # Shoot for…long enough to probably shoot all pieces.
    @mb.timed_state(duration=8, next_state="collect_from_neutral")
    def shoot_1(self):
        self.pewpew.activelyShoot = True

    @mb.state
    def collect_from_neutral(self, initial_call: bool):
        if initial_call:
            # Stop actively shooting while we drive
            self.pewpew.spinDown()
            self._target_heading = None  # Clear the target heading so we can re-calculate it for the next shot
            self.run_trajectory(
                "LeftTrenchTwice[1]",
                event_callbacks={
                    "RunIntake": self.intake.ingest,
                    "StopIntake": self.intake.stop,
                    "ExtendIntake": self.intake.extend,
                },
            )
        elif self.is_trajectory_done():
            self.next_state("prep2")

    @mb.timed_state(duration=2, next_state="shoot_2")
    def prep2(self):
        self._prepare_to_fire()
        if self.pewpew.isReadyToFire():
            self.next_state("shoot_2")

    # Shoot for…long enough to probably shoot all pieces.
    @mb.timed_state(duration=8, next_state="finished")
    def shoot_2(self):
        self.pewpew.activelyShoot = True

    @mb.state
    def finished(self):
        self.pewpew.spinDown()
        self.intake.retract()
