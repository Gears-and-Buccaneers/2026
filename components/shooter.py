"""A hypothetical shooter component on the robot."""

import magicbot
import wpilib


class Shooter:
    """The robot's shooter.

    Pew! Pew!
    """

    shooterMotor: wpilib.Talon

    # Shooter motor speed is tunable via NetworkTables
    shootSpeed = magicbot.tunable(1.0)

    def __init__(self):
        """Code to run when initially creating the shooter."""
        self.enabled = False

    def enable(self):
        """Causes the shooter motor to spin."""
        self.enabled = True

    def isReady(self):
        """Determine if the shooter is ready to fire."""
        # in a real robot, you'd be using an encoder to determine if the
        # shooter were at the right speed..
        return True

    def execute(self):
        """This gets called at the end of the control loop."""
        if self.enabled:
            self.shooterMotor.set(self.shootSpeed)
        else:
            self.shooterMotor.set(0)

        self.enabled = False
