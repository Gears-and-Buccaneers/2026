"""A hypothetical shooter component on the robot."""

import magicbot
import phoenix6
import wpilib


class Shooter:
    """The robot's shooter.

    Pew! Pew!
    """

    test_motor1: phoenix6.hardware.TalonFX
    test_motor2: phoenix6.hardware.TalonFX

    desired_speed: float

    # Shooter motor speed is tunable via NetworkTables
    shoot_speed = magicbot.tunable(1.0)

    def __init__(self):
        """Code to run when initially creating the shooter."""
        self.enabled = True
        self.control = phoenix6.controls.DutyCycleOut(0)


    def enable(self):
        """Causes the shooter motor to spin."""
        self.enabled = True

    def is_ready(self):
        """Determine if the shooter is ready to fire."""
        # in a real robot, you'd be using an encoder to determine if the
        # shooter were at the right speed..
        return True

    def execute(self):
        """This gets called at the end of the control loop."""
        self.test_motor1.set_control(self.control.with_output(self.desired_speed))
        self.test_motor2.set_control(self.control.with_output(-self.desired_speed))

    def go(self, speed):
        self.desired_speed = speed
