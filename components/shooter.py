"""A hypothetical shooter component on the robot."""

import magicbot
import wpilib


class Shooter:
    """The robot's shooter.

    Pew! Pew!
    """

    shooter_motor: wpilib.Talon

    # Shooter motor speed is tunable via NetworkTables
    shoot_speed = magicbot.tunable(1.0)

    def __init__(self):
        """Code to run when initially creating the shooter."""
        self.enabled = False

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
        if self.enabled:
            self.shooter_motor.set(self.shoot_speed)
        else:
            self.shooter_motor.set(0)

        self.enabled = False

    def fallbackSpin(self):
        """Gets the speed to a known location for the robot to shoot."""
        print("Wow! good job, you called fallbackSpin!")

    def shoot(self):
        """Shoots the fuel from the robot."""
        print("Wow! good job, you called shoot!")

    def autoShooterMotorPower(self):
        """Automatically spins the shooter motor to the correct positiono to shoot into the hub."""
        print("Wow! good job, you called autoShooterMotorPower!")
