"""Controllers for both the driver and operator."""

import math

import wpilib
from magicbot import feedback


def joystickSquareToCircle(x: float, y: float) -> tuple[float, float]:
    """Remap square joystick coordinates to be within a circle.

    For example, moving the joystick 45° to the corner yields values (1, 1);
    this function remaps those to (√2/2, √2/2)…while leaving movement on one
    axis only unchanged.

    Uses the Shirley-Chiu concentric mapping approximation.
    """
    return (
        x * math.sqrt(1 - y * y / 2),
        y * math.sqrt(1 - x * x / 2),
    )


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
        # Check if the left trigger is more than halfway down
        return self.getLeftTriggerAxis() > 0

    def shouldSmartAim(self) -> bool:
        """Determine if the auto aim button is actively being pressed."""
        # Check if the right trigger is more than halfway down
        return self.getRightTriggerAxis() > 0

    def shouldShoot(self) -> bool:
        """Determine if the shoot button is actively being pressed."""
        return self.getRightBumperButton()

    def shouldSetFallbackShooterSpinSpeed(self) -> bool:
        """Determine if the set-shooter-speed-to-predifened-speed button is actively being pressed."""
        return self.getYButton()

<<<<<<< HEAD
    def set_HGV_RGB_LEDs(self) -> float:
        """Determine if the left joystick has been moved and if so, by how much."""
        return -self.getOperatorLeftX(), -self.getOperatorLeftY()
=======
    def shouldToggleLEDMode(self) -> bool:
        """Determine if the LED mode toggle button was pressed since the last check."""
        return self.getLeftStickButtonPressed()

    def customLEDColor(self) -> wpilib.Color:
        """Use the left stick to pick a color based on its position.

        The rotational angle of the stick controls the hue, with red at the top,
        and the distance from center controls the brightness.
        """
        return self._colorFromStickValues(self.getLeftX(), self.getLeftY())

    def _colorFromStickValues(self, x: float, y: float) -> wpilib.Color:
        """Convert joystick x/y values to a color."""
        # The joystick values "move in a square", even when the stick moves in a circle.
        # For example, down and to the left is (-1, 1), which is a Euclidean distance of √2, but we want that
        # to be just as bright as straight down (-1, 0) or straight left (0, 1).
        # So, we remap the square joystick values to circular values.
        circularX, circularY = joystickSquareToCircle(x, y)

        # The wpilib.Color.fromHSV function expects hue as an integer from [0, 180),
        # saturation and value as an integer from [0, 255].
        return wpilib.Color.fromHSV(
            h=int(math.degrees(math.atan2(-circularX, -circularY) / 2)) % 180,
            s=255,
            v=round((circularX**2 + circularY**2) ** 0.5 * 255),
        )
>>>>>>> 8f7f72a92bf424e5fadebbaa40faf2f061d69c7e


class OperatorUSBGamepad(OperatorController):
    """Operator controller using a generic USB gamepad."""

    def shouldVomit(self) -> bool:
<<<<<<< HEAD
        """The yes/no function for outtaking(Left Trigger)."""
        return -self.getRawAxis(0) > 0.5

    def shouldSmartAim(self) -> bool:
        """The yes/no function for auto-aiming(Right Trigger)."""
        return -self.getRawButton(0)

    def shouldShoot(self) -> bool:
        """The yes/no function for shooting(Right Bumper)."""
        return self.getRawAxis(0) > 0.5

    def shouldSetFallbackShooterSpinSpeed(self) -> bool:
        """The yes/no function for shooting(Y button)."""
        return self.getRawButton(0)

    def set_HGV_RGB_LEDs(self) -> float:
        """Determine if the left joystick has been moved and if so, by how much."""
        return self.getRawAxis(4), self.getRawAxis(3)
=======
        """Determine if the outtake button is actively being pressed."""
        return self.getRawAxis(2) > 0

    def shouldSmartAim(self) -> bool:
        """Determine if the auto aim button is actively being pressed."""
        return self.getRawAxis(5) > 0

    def shouldShoot(self) -> bool:
        """Determine if the shoot button is actively being pressed."""
        return self.getRawButton(5)

    def shouldSetFallbackShooterSpinSpeed(self) -> bool:
        """Determine if the set-shooter-speed-to-predifened-speed button is actively being pressed."""
        return self.getRawButton(3)

    def shouldToggleLEDMode(self) -> bool:
        """Determine if the LED mode toggle button was pressed since the last check."""
        return self.getRawButtonPressed(6)

    def customLEDColor(self) -> wpilib.Color:
        """Use the left stick to pick a color based on its position.

        The rotational angle of the stick controls the hue, with red at the top,
        and the distance from center controls the brightness.
        """
        return self._colorFromStickValues(self.getRawAxis(0), self.getRawAxis(1))
>>>>>>> 8f7f72a92bf424e5fadebbaa40faf2f061d69c7e
