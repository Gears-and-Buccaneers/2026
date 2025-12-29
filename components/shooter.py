"""A hypothetical shooter component on the robot."""

import phoenix6.hardware as p6hw


class Shooter:
    """The robot's shooter.

    Pew! Pew!
    """

    shooter_motor: p6hw.TalonFX
    should_intake: bool = False
    should_output: bool = False

    # Shooter motor speed is tunable via NetworkTables
    shoot_speed = 0.2

    def __init__(self):
        """Code to run when initially creating the shooter."""
        self.enabled = False

    def enable(self):
        """Causes the shooter motor to spin."""
        self.enabled = True

    def is_ready(self):
        """Determine if the shooter is ready to fire."""
        # in a real robot, you'd be using an encoder to determine if the
        # shooter were at the right angle and speed..
        return True

    def execute(self):
        """This gets called at the end of the control loop."""
        if self.should_intake:
            self.shooter_motor.set(self.shoot_speed)  # Intake is positive
            print("intaking!!")
        elif self.should_output:
            self.shooter_motor.set(-self.shoot_speed)  # Output is negative
            print("outputing!!")
        else:
            self.shooter_motor.set(0)  # Motor is zero
