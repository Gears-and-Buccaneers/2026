"""Controllers for both the driver and operator."""

import wpilib
from magicbot import feedback


class DriverController(wpilib.XboxController):
    """Controller with information focused on the driver controls.

    In simulation, keyboard input is supported via the simulation GUI.
    Drag "Keyboard 0" from System Joysticks to Joystick[0] in the sim GUI.

    Default keyboard mappings in sim GUI:
    - WASD: Left stick (rotation)
    - Arrow keys: Right stick (strafe/forward-back)
    """

    def get_move_forward_percent(self) -> float:
        """Get the desired forward/reverse percent from the "move" stick.

        Returns:
            A value in the range [-1.0, 1.0], where positive is forward.
        """
        # Invert so that pushing up on the stick is positive
        return -self.getRightY()

    def get_move_left_percent(self) -> float:
        """Get the desired left/right percent from the "move" stick.

        Returns:
            A value in the range [-1.0, 1.0], where positive is left.
        """
        # Invert so that pushing left on the stick is positive
        return -self.getRightX()

    def get_rotate_counter_clockwise_percent(self) -> float:
        """Get the desired rotation percent from the "rotate" stick.

        Returns:
            A value in the range [-1.0, 1.0], where positive is counterclockwise.
        """
        # Invert so that pushing left on the stick is positive
        return -self.getLeftX()

    def should_brake(self) -> bool:
        """Determine if the brake button is actively being pressed."""
        return self.getXButton()

    def should_zero_gyro(self) -> bool:
        """Determine if the zero gyro button is actively being pressed."""
        # TODO: should this use "getRightBumperButtonPressed" or "getRightBumperButton" (which can be held)?
        return self.getRightBumperButtonPressed()


class DriverUSBGamepad(DriverController):
    """Driver controller using a generic USB gamepad."""

    def get_rotate_counter_clockwise_percent(self) -> float:
        """The rotation axis."""
        return -self.getRawAxis(0)

    def get_move_left_percent(self) -> float:
        """The left/right axis."""
        return -self.getRawAxis(3)

    def get_move_forward_percent(self) -> float:
        """The forward/reverse axis."""
        return self.getRawAxis(4)

    def should_brake(self) -> bool:
        """Get the state of the X button."""
        return self.getRawButton(3)

    def should_zero_gyro(self) -> bool:
        """Get whether the right bumper button was pressed since the last check."""
        return self.getRawButtonPressed(6)


class OperatorController(wpilib.XboxController):
    """Controller with information focused on the operator controls."""

    @feedback
    def shouldVomit(self) -> bool:
        """Determine if the outtake button is actively being pressed."""
        return self.getOperatorLeftTriggerButton()

    def shouldSmartAim(self) -> bool:
        """Determine if the auto aim button is actively being pressed."""
        return self.getOperatorRightTriggerButton()

    def shouldShoot(self) -> bool:
        """Determine if the shoot button is actively being pressed."""
        return self.getOperatorLeftTriggerButton()

    def shouldSetFallbackShooterSpinSpeed(self) -> bool:
        """Determine if the set-shooter-speed-to-predifened-speed button is actively being pressed."""
        return self.getOperatorYButton()

    def set_HGV_RGB_LEDs(self) -> float:
        """Determine if the left joystick has been moved and if so, by how much."""
        return -self.getOperatorLeftX(), -self.getOperatorLeftY()
