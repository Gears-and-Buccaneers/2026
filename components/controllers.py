"""Controllers for both the driver and operator."""

import math
from dataclasses import dataclass

import wpilib


def joystick_square_to_circle(x: float, y: float) -> tuple[float, float]:
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


@dataclass(frozen=True)
class AxisMapping:
    """Mapping for a single axis."""

    index: int
    invert: bool = False
    full_range: bool = False  # True if trigger axis goes -1 to 1 (needs remapping to 0 to 1)


@dataclass(frozen=True)
class ControllerProfile:
    """Minimal mapping profile for a controller type."""

    name: str
    # Joystick axes
    left_x: AxisMapping
    left_y: AxisMapping
    right_x: AxisMapping
    right_y: AxisMapping
    # Trigger axes
    left_trigger: AxisMapping
    right_trigger: AxisMapping
    # Button mappings (1-indexed as per WPILib)
    a_button: int
    b_button: int
    x_button: int
    y_button: int
    left_bumper: int
    right_bumper: int
    left_stick_button: int
    right_stick_button: int
    back_button: int = 7
    start_button: int = 8


class MappedController(wpilib.XboxController):
    """Xbox controller with remappable inputs based on a named profile.

    If a profile name is provided, all axis and button methods use that profile's mappings.
    If no profile name is provided (or is empty/"xbox"), behaves like a standard wired XboxController.

    Available profiles:
    * "wired" - [default] an XBox controller on Windows connected with USB; running in "Mapped" mode in Sim
    * "wireless" - an wireless XBox controller on Windows connected via bluetooth; running in "Mapped" mode in Sim
    * "wirednomap" - an XBox controller on Windows connected with USB; running in "Unmapped" mode in Sim
    * "wirelessnomap" - an wireless XBox controller on Windows connected via bluetooth; running in "Unmapped" mode in Sim
    * "macwireless" - an XBox controller on macOS connected via bluetooth; running in Sim
    * "macwired" - an XBox controller on macOS connected with USB
    """

    # Controller profiles based on mapping spreadsheet.
    # Negative axis numbers in the spreadsheet indicate inversion is needed.
    # Orange cells indicate full-range triggers (-1 to 1 instead of 0 to 1).
    PROFILES: dict[str, ControllerProfile] = {
        # Windows Wired Controller (mapped)
        "wired": ControllerProfile(
            name="Windows Wired Controller (mapped)",
            left_x=AxisMapping(0),
            left_y=AxisMapping(1),
            right_x=AxisMapping(4),
            right_y=AxisMapping(5),
            left_trigger=AxisMapping(2),
            right_trigger=AxisMapping(3),
            a_button=1,
            b_button=2,
            x_button=3,
            y_button=4,
            left_bumper=5,
            right_bumper=6,
            left_stick_button=9,
            right_stick_button=10,
        ),
        # Windows Xbox Wireless (mapped)
        "wireless": ControllerProfile(
            name="Windows Xbox Wireless (mapped mode)",
            left_x=AxisMapping(0),
            left_y=AxisMapping(1),
            right_x=AxisMapping(2),
            right_y=AxisMapping(3),
            left_trigger=AxisMapping(4),
            right_trigger=AxisMapping(5),
            a_button=1,
            b_button=2,
            x_button=3,
            y_button=4,
            left_bumper=5,
            right_bumper=6,
            left_stick_button=9,
            right_stick_button=10,
        ),
        # Windows Xbox Wireless (unmapped)
        "wirelessnomap": ControllerProfile(
            name="Windows Xbox Wireless (unmapped)",
            left_x=AxisMapping(0),
            left_y=AxisMapping(1),
            right_x=AxisMapping(4),
            right_y=AxisMapping(5),
            left_trigger=AxisMapping(2, full_range=True),
            right_trigger=AxisMapping(3, full_range=True),
            a_button=1,
            b_button=2,
            x_button=3,
            y_button=4,
            left_bumper=5,
            right_bumper=6,
            left_stick_button=9,
            right_stick_button=10,
        ),
        # Windows Wired Controller (unmapped)
        "wirednomap": ControllerProfile(
            name="Windows Wired Controller (unmapped)",
            left_x=AxisMapping(0),
            left_y=AxisMapping(1),
            right_x=AxisMapping(2),
            right_y=AxisMapping(3),
            left_trigger=AxisMapping(4, full_range=True),
            right_trigger=AxisMapping(5, full_range=True),
            a_button=1,
            b_button=2,
            x_button=3,
            y_button=4,
            left_bumper=5,
            right_bumper=6,
            left_stick_button=9,
            right_stick_button=10,
        ),
        # macOS Xbox Wireless
        "macwireless": ControllerProfile(
            name="macOS Xbox Wireless",
            left_x=AxisMapping(0),
            left_y=AxisMapping(1),
            right_x=AxisMapping(2),
            right_y=AxisMapping(3),
            left_trigger=AxisMapping(5, full_range=True),
            right_trigger=AxisMapping(4, full_range=True),
            a_button=1,
            b_button=2,
            x_button=4,
            y_button=5,
            left_bumper=7,
            right_bumper=8,
            left_stick_button=14,
            right_stick_button=15,
        ),
        # macOS Wired Controller
        "macwired": ControllerProfile(
            name="macOS Wired Controller",
            left_x=AxisMapping(0),
            left_y=AxisMapping(1, invert=True),
            right_x=AxisMapping(3),
            right_y=AxisMapping(4, invert=True),
            left_trigger=AxisMapping(2, full_range=True),
            right_trigger=AxisMapping(5, full_range=True),
            a_button=1,
            b_button=2,
            x_button=3,
            y_button=4,
            left_bumper=5,
            right_bumper=6,
            left_stick_button=7,
            right_stick_button=8,
        ),
    }

    # Aliases for convenience
    PROFILES["gamepad"] = PROFILES["macwired"]
    PROFILES["xbox"] = PROFILES["wired"]

    def __init__(self, port: int, profile_name: str = "") -> None:
        """Initialize the mapped controller.

        Args:
            port: The port the controller is connected to.
            profile_name: Name of the controller profile to use.
                          Empty string or "xbox" uses standard XboxController behavior.
        """
        super().__init__(port)

        # Resolve profile name to profile (empty or "xbox" means no remapping)
        name = profile_name.lower().strip()
        if name and name != "xbox":
            profile = self.PROFILES.get(name)
            if profile is None:
                print(f"Warning: Unknown controller profile '{profile_name}', using xbox")
                profile = self.PROFILES["xbox"]
            print(f"Controller on port {port} using profile: {profile.name}")
        else:
            profile = self.PROFILES["xbox"]

        # Store the resolved mappings for fast access
        self._left_x = profile.left_x
        self._left_y = profile.left_y
        self._right_x = profile.right_x
        self._right_y = profile.right_y
        self._left_trigger = profile.left_trigger
        self._right_trigger = profile.right_trigger
        self._a = profile.a_button
        self._b = profile.b_button
        self._x = profile.x_button
        self._y = profile.y_button
        self._lb = profile.left_bumper
        self._rb = profile.right_bumper
        self._ls = profile.left_stick_button
        self._rs = profile.right_stick_button
        self._back = profile.back_button
        self._start = profile.start_button

    def _axis(self, mapping: AxisMapping) -> float:
        """Get an axis value with mapping applied."""
        value = self.getRawAxis(mapping.index)
        return -value if mapping.invert else value

    def _trigger(self, mapping: AxisMapping) -> float:
        """Get a trigger axis value, converting full-range (-1..1) to half-range (0..1) if needed."""
        value = self.getRawAxis(mapping.index)
        if mapping.invert:
            value = -value
        if mapping.full_range:
            value = (value + 1.0) / 2.0
        return value

    # --- Axis methods ---

    def getLeftX(self) -> float:
        return self._axis(self._left_x)

    def getLeftY(self) -> float:
        return self._axis(self._left_y)

    def getRightX(self) -> float:
        return self._axis(self._right_x)

    def getRightY(self) -> float:
        return self._axis(self._right_y)

    def getLeftTriggerAxis(self) -> float:
        return self._trigger(self._left_trigger)

    def getRightTriggerAxis(self) -> float:
        return self._trigger(self._right_trigger)

    # --- Button methods ---

    def getAButton(self) -> bool:
        return self.getRawButton(self._a)

    def getAButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self._a)

    def getAButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self._a)

    def getBButton(self) -> bool:
        return self.getRawButton(self._b)

    def getBButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self._b)

    def getBButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self._b)

    def getXButton(self) -> bool:
        return self.getRawButton(self._x)

    def getXButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self._x)

    def getXButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self._x)

    def getYButton(self) -> bool:
        return self.getRawButton(self._y)

    def getYButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self._y)

    def getYButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self._y)

    def getLeftBumperButton(self) -> bool:
        return self.getRawButton(self._lb)

    def getLeftBumperButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self._lb)

    def getLeftBumperButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self._lb)

    def getRightBumperButton(self) -> bool:
        return self.getRawButton(self._rb)

    def getRightBumperButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self._rb)

    def getRightBumperButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self._rb)

    def getLeftStickButton(self) -> bool:
        return self.getRawButton(self._ls)

    def getLeftStickButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self._ls)

    def getLeftStickButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self._ls)

    def getRightStickButton(self) -> bool:
        return self.getRawButton(self._rs)

    def getRightStickButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self._rs)

    def getRightStickButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self._rs)

    def getBackButton(self) -> bool:
        return self.getRawButton(self._back)

    def getBackButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self._back)

    def getBackButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self._back)

    def getStartButton(self) -> bool:
        return self.getRawButton(self._start)

    def getStartButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self._start)

    def getStartButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self._start)


class DriverController(MappedController):
    """Controller with information focused on the driver controls.

    In simulation, keyboard input is supported via the simulation GUI.
    Drag "Keyboard 0" from System Joysticks to Joystick[0] in the sim GUI.

    Default keyboard mappings in sim GUI:
    - WASD: Left stick (rotation)
    - Arrow keys: Right stick (strafe/forward-back)
    """

    def __init__(self, port: int, profile_name: str = "") -> None:
        """Initialize the driver controller.

        Args:
            port: The port the controller is connected to.
            profile_name: Name of the controller profile (e.g., "xbox", "wireless", "macwired").
        """
        super().__init__(port, profile_name)

    def get_move_forward_percent(self) -> float:
        """Get the desired forward/reverse percent from the "move" stick.

        Returns:
            A value in the range [-1.0, 1.0], where positive is forward.
        """
        return -self.getRightY()

    def get_move_left_percent(self) -> float:
        """Get the desired left/right percent from the "move" stick.

        Returns:
            A value in the range [-1.0, 1.0], where positive is left.
        """
        return -self.getRightX()

    def get_rotate_counter_clockwise_percent(self) -> float:
        """Get the desired rotation percent from the "rotate" stick.

        Returns:
            A value in the range [-1.0, 1.0], where positive is counterclockwise.
        """
        return -self.getLeftX()

    def should_brake(self) -> bool:
        """Determine if the brake button is actively being pressed."""
        return self.getXButton()

    def should_zero_gyro(self) -> bool:
        """Determine if the zero gyro button was pressed since the last check."""
        return self.getRightBumperButtonPressed()


class OperatorController(MappedController):
    """Controller with information focused on the operator controls."""

    def __init__(self, port: int, profile_name: str = "") -> None:
        """Initialize the operator controller.

        Args:
            port: The port the controller is connected to.
            profile_name: Name of the controller profile (e.g., "xbox", "wireless", "macwired").
        """
        super().__init__(port, profile_name)

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

    def customLEDColor(self) -> wpilib.Color:
        """Use the left stick to pick a color based on its position.

        The rotational angle of the stick controls the hue, with red at the top,
        and the distance from center controls the brightness.
        """
        return self._colorFromStickValues(self.getLeftX(), self.getLeftY())

    def _colorFromStickValues(self, x: float, y: float) -> wpilib.Color:
        """Convert joystick x/y values to a color."""
        # Remap square joystick values to circular values
        circularX, circularY = joystick_square_to_circle(x, y)

        # wpilib.Color.fromHSV expects hue [0, 180), saturation and value [0, 255]
        return wpilib.Color.fromHSV(
            h=int(math.degrees(math.atan2(-circularX, -circularY) / 2)) % 180,
            s=255,
            v=round((circularX**2 + circularY**2) ** 0.5 * 255),
        )
