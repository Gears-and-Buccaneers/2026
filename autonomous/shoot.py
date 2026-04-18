"""Auto for only shooting."""

# ruff: noqa: N801
# ruff: noqa: D102
# ruff: noqa: D101

import magicbot

import components


class justShoot(magicbot.AutonomousStateMachine):
    pewpew: components.Shooter
    drivetrain: components.Drivetrain

    MODE_NAME = "Basic_Shoot"
    DISABLED = False

    @magicbot.timed_state(duration=1, next_state="shoot", first=True)
    def aim(self, initial_call):
        self._target_heading = self.pewpew.spinUpAndTargetHub()
        self.drivetrain.driveFacingAngle(targetAngle=self._target_heading)

    @magicbot.state
    def shoot(self, initial_call, duration=4):
        self.pewpew.activelyShoot = True
