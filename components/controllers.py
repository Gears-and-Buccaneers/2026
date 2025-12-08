"""Controllers for both the driver and operator."""

import wpilib
from wpilib import RobotBase
from wpilib.simulation import PS4ControllerSim


class OurController(wpilib.XboxController):
    """A wrapper around wpilib.PS4Controller to provide easier access to common controls."""

    def getLeftStick(self) -> tuple[float, float]:
        """Get the position of the left stick as two values, both in the range [-1.0, 1.0].

        Note:
            Moving the stick to the right is +x, and down is +y.
        """
        return (self.getLeftX(), self.getLeftY())

    def getRightStick(self) -> tuple[float, float]:
        """Get the position of the right stick as two values, both in the range [-1.0, 1.0].

        Note:
            Moving the stick to the right is +x, and down is +y.
        """
        return (self.getRightX(), self.getRightY())


class DriverController(OurPS4Controller):
    """Controller with information focused on the driver controls.

    In simulation, keyboard input is supported via the simulation GUI.
    Drag "Keyboard 0" from System Joysticks to Joystick[0] in the sim GUI.

    Default keyboard mappings in sim GUI:
    - WASD: Left stick (strafe/forward-back)
    - Arrow keys or IJKL: Right stick (rotation)
    """

    def should_brake(self) -> bool:
        """Determine if the brake button is actively being pressed."""
        return self.getXButton()
