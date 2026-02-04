"""Controllers for both the driver and operator."""

import math
from typing import NamedTuple

import wpilib

import utils


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


class AxisMapping(NamedTuple):
    """Mapping for a single axis."""

    axis: int  # The axis index on the controller.
    invert: bool = False  # Whether to invert the axis value.
    fullRange: bool = False  # True if trigger axis goes -1 to 1 (needs remapping to 0 to 1)


class ControllerProfile(NamedTuple):
    """Minimal mapping profile for a controller type."""

    name: str = "Standard Xbox Controller"
    # Joystick axes
    leftX: AxisMapping = AxisMapping(0)
    leftY: AxisMapping = AxisMapping(1)
    rightX: AxisMapping = AxisMapping(2)
    rightY: AxisMapping = AxisMapping(3)
    # Trigger axes
    leftTrigger: AxisMapping = AxisMapping(4)
    rightTrigger: AxisMapping = AxisMapping(5)
    # Button mappings (1-indexed as per WPILib)
    aButton: int = 1
    bButton: int = 2
    xButton: int = 3
    yButton: int = 4
    leftBumper: int = 5
    rightBumper: int = 6
    leftStickButton: int = 9
    rightStickButton: int = 10
    backButton: int = 7
    startButton: int = 8


class MappedController(wpilib.XboxController):
    """Xbox controller with remappable inputs based on a named profile.

    If a profile name is provided, all axis and button methods use that profile's mappings.
    If no profile name is provided (or is empty/"xbox"), behaves like a standard wired XboxController.

    Available profiles:
    * "wired" - [default] an Xbox controller on Windows connected with USB; running in "Mapped" mode in sim
    * "wireless" - a wireless Xbox controller on Windows connected via bluetooth; running in "Mapped" mode in sim
    * "wirednomap" - an Xbox controller on Windows connected with USB, running in "Unmapped" mode in sim
    * "wirelessnomap" - a wireless Xbox controller on Windows connected via bluetooth, running in "Unmapped" mode in sim
    * "macwireless" - an Xbox controller on macOS connected via bluetooth
    * "macwired" - an Xbox controller on macOS connected with USB
    """

    # Controller profiles based on mapping spreadsheet.
    # Negative axis numbers in the spreadsheet indicate inversion is needed.
    # Orange cells indicate full-range triggers (-1 to 1 instead of 0 to 1).
    PROFILES: dict[str, ControllerProfile] = {
        # Windows Wired Controller (mapped)
        "wired": ControllerProfile(
            name="Windows Wired Controller (mapped)",
            leftX=AxisMapping(0),
            leftY=AxisMapping(1),
            rightX=AxisMapping(4),
            rightY=AxisMapping(5),
            leftTrigger=AxisMapping(2),
            rightTrigger=AxisMapping(3),
            aButton=1,
            bButton=2,
            xButton=3,
            yButton=4,
            leftBumper=5,
            rightBumper=6,
            leftStickButton=9,
            rightStickButton=10,
        ),
        # Windows Xbox Wireless (mapped)
        "wireless": ControllerProfile(
            name="Windows Xbox Wireless (mapped mode)",
            leftX=AxisMapping(0),
            leftY=AxisMapping(1),
            rightX=AxisMapping(2),
            rightY=AxisMapping(3),
            leftTrigger=AxisMapping(4),
            rightTrigger=AxisMapping(5),
            aButton=1,
            bButton=2,
            xButton=3,
            yButton=4,
            leftBumper=5,
            rightBumper=6,
            leftStickButton=9,
            rightStickButton=10,
        ),
        # Windows Xbox Wireless (unmapped)
        "wirelessnomap": ControllerProfile(
            name="Windows Xbox Wireless (unmapped)",
            leftX=AxisMapping(0),
            leftY=AxisMapping(1),
            rightX=AxisMapping(4),
            rightY=AxisMapping(5),
            leftTrigger=AxisMapping(2, fullRange=True),
            rightTrigger=AxisMapping(3, fullRange=True),
            aButton=1,
            bButton=2,
            xButton=3,
            yButton=4,
            leftBumper=5,
            rightBumper=6,
            leftStickButton=9,
            rightStickButton=10,
        ),
        # Windows Wired Controller (unmapped)
        "wirednomap": ControllerProfile(
            name="Windows Wired Controller (unmapped)",
            leftX=AxisMapping(0),
            leftY=AxisMapping(1),
            rightX=AxisMapping(2),
            rightY=AxisMapping(3),
            leftTrigger=AxisMapping(4, fullRange=True),
            rightTrigger=AxisMapping(5, fullRange=True),
            aButton=1,
            bButton=2,
            xButton=3,
            yButton=4,
            leftBumper=5,
            rightBumper=6,
            leftStickButton=9,
            rightStickButton=10,
        ),
        # macOS Xbox Wireless
        "macwireless": ControllerProfile(
            name="macOS Xbox Wireless",
            leftX=AxisMapping(0),
            leftY=AxisMapping(1),
            rightX=AxisMapping(2),
            rightY=AxisMapping(3),
            leftTrigger=AxisMapping(5, fullRange=True),
            rightTrigger=AxisMapping(4, fullRange=True),
            aButton=1,
            bButton=2,
            xButton=4,
            yButton=5,
            leftBumper=7,
            rightBumper=8,
            leftStickButton=14,
            rightStickButton=15,
        ),
        # macOS Wired Controller
        "macwired": ControllerProfile(
            name="macOS Wired Controller",
            leftX=AxisMapping(0),
            leftY=AxisMapping(1, invert=True),
            rightX=AxisMapping(3),
            rightY=AxisMapping(4, invert=True),
            leftTrigger=AxisMapping(2, fullRange=True),
            rightTrigger=AxisMapping(5, fullRange=True),
            aButton=1,
            bButton=2,
            xButton=3,
            yButton=4,
            leftBumper=5,
            rightBumper=6,
            leftStickButton=7,
            rightStickButton=8,
        ),
    }

    # Aliases for convenience
    PROFILES["gamepad"] = PROFILES["macwired"]
    PROFILES["xbox"] = PROFILES["wired"]

    def __init__(self, port: int, profileName: str = "wired") -> None:
        """Initialize the mapped controller.

        Args:
            port: The port the controller is connected to.
            profileName: Name of the controller profile to use.
        """
        super().__init__(port)

        # Resolve profile name to profile, with fallback to "xbox"
        name = profileName.lower().strip()
        self.profile = self.PROFILES.get(name, self.PROFILES["xbox"])
        print(f"Controller on port {port} using profile: {self.profile.name}")

    def _axis(self, mapping: AxisMapping) -> float:
        """Get an axis value with mapping applied."""
        value = self.getRawAxis(mapping.axis)
        return -value if mapping.invert else value

    def _trigger(self, mapping: AxisMapping) -> float:
        """Get a trigger axis value, converting full-range (-1..1) to half-range (0..1) if needed."""
        value = self.getRawAxis(mapping.axis)
        if mapping.invert:
            value = -value
        if mapping.fullRange:
            value = (value + 1.0) / 2.0
        return value

    # --- Axis methods ---

    def getLeftX(self) -> float:  # noqa: D102
        return self._axis(self.profile.leftX)

    def getLeftY(self) -> float:  # noqa: D102
        return self._axis(self.profile.leftY)

    def getRightX(self) -> float:  # noqa: D102
        return self._axis(self.profile.rightX)

    def getRightY(self) -> float:  # noqa: D102
        return self._axis(self.profile.rightY)

    def getLeftTriggerAxis(self) -> float:  # noqa: D102
        return self._trigger(self.profile.leftTrigger)

    def getRightTriggerAxis(self) -> float:  # noqa: D102
        return self._trigger(self.profile.rightTrigger)

    # --- Button methods ---

    def getAButton(self) -> bool:  # noqa: D102
        return self.getRawButton(self.profile.aButton)

    def getAButtonPressed(self) -> bool:  # noqa: D102
        return self.getRawButtonPressed(self.profile.aButton)

    def getAButtonReleased(self) -> bool:  # noqa: D102
        return self.getRawButtonReleased(self.profile.aButton)

    def getBButton(self) -> bool:  # noqa: D102
        return self.getRawButton(self.profile.bButton)

    def getBButtonPressed(self) -> bool:  # noqa: D102
        return self.getRawButtonPressed(self.profile.bButton)

    def getBButtonReleased(self) -> bool:  # noqa: D102
        return self.getRawButtonReleased(self.profile.bButton)

    def getXButton(self) -> bool:  # noqa: D102
        return self.getRawButton(self.profile.xButton)

    def getXButtonPressed(self) -> bool:  # noqa: D102
        return self.getRawButtonPressed(self.profile.xButton)

    def getXButtonReleased(self) -> bool:  # noqa: D102
        return self.getRawButtonReleased(self.profile.xButton)

    def getYButton(self) -> bool:  # noqa: D102
        return self.getRawButton(self.profile.yButton)

    def getYButtonPressed(self) -> bool:  # noqa: D102
        return self.getRawButtonPressed(self.profile.yButton)

    def getYButtonReleased(self) -> bool:  # noqa: D102
        return self.getRawButtonReleased(self.profile.yButton)

    def getLeftBumperButton(self) -> bool:  # noqa: D102
        return self.getRawButton(self.profile.leftBumper)

    def getLeftBumperButtonPressed(self) -> bool:  # noqa: D102
        return self.getRawButtonPressed(self.profile.leftBumper)

    def getLeftBumperButtonReleased(self) -> bool:  # noqa: D102
        return self.getRawButtonReleased(self.profile.leftBumper)

    def getRightBumperButton(self) -> bool:  # noqa: D102
        return self.getRawButton(self.profile.rightBumper)

    def getRightBumperButtonPressed(self) -> bool:  # noqa: D102
        return self.getRawButtonPressed(self.profile.rightBumper)

    def getRightBumperButtonReleased(self) -> bool:  # noqa: D102
        return self.getRawButtonReleased(self.profile.rightBumper)

    def getLeftStickButton(self) -> bool:  # noqa: D102
        return self.getRawButton(self.profile.leftStickButton)

    def getLeftStickButtonPressed(self) -> bool:  # noqa: D102
        return self.getRawButtonPressed(self.profile.leftStickButton)

    def getLeftStickButtonReleased(self) -> bool:  # noqa: D102
        return self.getRawButtonReleased(self.profile.leftStickButton)

    def getRightStickButton(self) -> bool:  # noqa: D102
        return self.getRawButton(self.profile.rightStickButton)

    def getRightStickButtonPressed(self) -> bool:  # noqa: D102
        return self.getRawButtonPressed(self.profile.rightStickButton)

    def getRightStickButtonReleased(self) -> bool:  # noqa: D102
        return self.getRawButtonReleased(self.profile.rightStickButton)

    def getBackButton(self) -> bool:  # noqa: D102
        return self.getRawButton(self.profile.backButton)

    def getBackButtonPressed(self) -> bool:  # noqa: D102
        return self.getRawButtonPressed(self.profile.backButton)

    def getBackButtonReleased(self) -> bool:  # noqa: D102
        return self.getRawButtonReleased(self.profile.backButton)

    def getStartButton(self) -> bool:  # noqa: D102
        return self.getRawButton(self.profile.startButton)

    def getStartButtonPressed(self) -> bool:  # noqa: D102
        return self.getRawButtonPressed(self.profile.startButton)

    def getStartButtonReleased(self) -> bool:  # noqa: D102
        return self.getRawButtonReleased(self.profile.startButton)


class DriverController(MappedController):
    """Controller with information focused on the driver controls.

    In simulation, keyboard input is supported via the simulation GUI.
    Drag "Keyboard 0" from System Joysticks to Joystick[0] in the sim GUI.

    Default keyboard mappings in sim GUI:
    - WASD: Left stick (rotation)
    - Arrow keys: Right stick (strafe/forward-back)
    """

    def __init__(self, port: int, profileName: str = "wired") -> None:
        """Initialize the driver controller.

        Args:
            port: The port the controller is connected to.
            profileName: Name of the controller profile (e.g., "xbox", "wireless", "macwired").
        """
        super().__init__(port, profileName)

    def getMoveForwardPercent(self) -> float:
        """Get the desired forward/reverse percent from the "move" stick.

        Returns:
            A value in the range [-1.0, 1.0], where positive is forward.
        """
        return -self.getRightY()

    def getMoveLeftPercent(self) -> float:
        """Get the desired left/right percent from the "move" stick.

        Returns:
            A value in the range [-1.0, 1.0], where positive is left.
        """
        return -self.getRightX()

    def getRotateCounterClockwisePercent(self) -> float:
        """Get the desired rotation percent from the "rotate" stick.

        Returns:
            A value in the range [-1.0, 1.0], where positive is counterclockwise.
        """
        return -self.getLeftX()

    def shouldBrake(self) -> bool:
        """Determine if the brake button is actively being pressed."""
        return self.getXButton()

    def shouldZeroGyro(self) -> bool:
        """Determine if the zero gyro button was pressed since the last check."""
        return self.getRightBumperButtonPressed()

    def shouldIntake(self) -> bool:
        """Determine if the intake button was pressed since the last check."""
        return self.getRightTriggerAxis() > 0.5

    def activatePrecisionMode(self) -> bool:
        """Determine if the presicion mode button was pressed since the last check."""
        return self.getLeftBumperButtonPressed()

    def moveToOutpost(self) -> bool:
        """Determine if the move to outpost button is being pressed."""
        return self.getAButton()

    def shouldClimbDown(self) -> bool:
        """Determine if the climb down button is being pressed."""
        return self.getPOV()

    def shouldClimbUp(self) -> bool:
        """Determine if the climb up button is being pressed."""
        return self.getPOV()

    def activateCycleMode(self) -> bool:
        """Determine if the cycle mode button is being pressed."""
        return self.getYButton()


class OperatorController(MappedController):
    """Controller with information focused on the operator controls."""

    def __init__(self, port: int, profileName: str = "wired") -> None:
        """Initialize the operator controller.

        Args:
            port: The port the controller is connected to.
            profileName: Name of the controller profile (e.g., "xbox", "wireless", "macwired").
        """
        super().__init__(port, profileName)

    def shouldVomit(self) -> bool:
        """Determine if the outtake button is actively being pressed."""
        return self.getLeftTriggerAxis() > 0.5

    def shouldSmartAim(self) -> bool:
        """Determine if the auto aim button is actively being pressed."""
        return self.getRightTriggerAxis() > 0.5

    def shouldShoot(self) -> bool:
        """Determine if the shoot button is actively being pressed."""
        return self.getRightBumperButton()

    def shouldSetFallbackShooterSpinSpeed(self) -> bool:
        """Determine if the set-shooter-speed-to-predefined-speed button is actively being pressed."""
        return self.getYButton()

    def shouldToggleLEDMode(self) -> bool:
        """Determine if the LED mode toggle button was pressed since the last check."""
        return self.getLeftStickButtonPressed()

    def customLEDColor(self) -> wpilib.Color8Bit:
        """Use the left stick to pick a color based on its position.

        The rotational angle of the stick controls the hue, with red at the top,
        and the distance from center controls the brightness.
        """
        return self._colorFromStickValues(self.getLeftX(), self.getLeftY())

    def _colorFromStickValues(self, x: float, y: float) -> wpilib.Color8Bit:
        """Convert joystick x/y values to a color."""
        # Remap square joystick values to circular values
        circularX, circularY = joystickSquareToCircle(x, y)

        return utils.color8FromHSV(
            h=math.degrees(math.atan2(-circularX, -circularY)),
            s=1,
            v=(circularX**2 + circularY**2) ** 0.5,
        )
