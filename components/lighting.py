"""LED control component for the robot."""

import wpilib
import wpimath.units as units
from wpilib import DriverStation

import constants as const

# Mapping of robot phase or shift name, and whether our alliance can score in the hub,
# to the color to show on the LEDs.
# fmt: off
STATE_COLORS: dict[tuple[str, bool], wpilib.Color8Bit] = {
    ("disabled",  False): wpilib.Color8Bit( 50 , 50,  50), # dark gray
    ("test",      False): wpilib.Color8Bit(255, 255,   0), # yellow
    ("autonomous", True): wpilib.Color8Bit(100, 100, 255), # light blue
    ("transition", True): wpilib.Color8Bit(180, 255,   0), # lime green
    ("shift1",     True): wpilib.Color8Bit(  0, 220,   0), # medium green
    ("shift1",    False): wpilib.Color8Bit(220,   0,   0), # medium red
    ("shift2",     True): wpilib.Color8Bit(  0, 220,   0), # medium green
    ("shift2",    False): wpilib.Color8Bit(220,   0,   0), # medium red
    ("shift3",     True): wpilib.Color8Bit(  0, 220,   0), # medium green
    ("shift3",    False): wpilib.Color8Bit(220,   0,   0), # medium red
    ("shift4",     True): wpilib.Color8Bit(  0, 220,   0), # medium green
    ("shift4",    False): wpilib.Color8Bit(220,   0,   0), # medium red
    ("endgame",    True): wpilib.Color8Bit(  0, 255, 255), # bright cyan
}
# fmt: on

LIGHTS_IN_STRIP: int = 30
OFF_COLOR: wpilib.Color8Bit = wpilib.Color8Bit(0, 0, 0)


class Lighting:
    """Controls the LED lighting system on the robot."""

    ledController: wpilib.AddressableLED

    def setup(self) -> None:
        """Initialize the LED hardware once it's been created.

        Called once by MagicBot after all the variable injection has finished.
        """
        self._desiredPercent: float = 1.0
        self._desiredColor: wpilib.Color8Bit = wpilib.Color8Bit(255, 0, 255)

        # Specify how many lights are in the LED strip
        self.ledController.setLength(LIGHTS_IN_STRIP)

        # Create one LEDData per LED in the strip
        self.ledBuffer: list[wpilib.AddressableLED.LEDData] = [
            wpilib.AddressableLED.LEDData() for _ in range(LIGHTS_IN_STRIP)
        ]

        # Start continuous output
        self.ledController.start()

    def showProgress(self, fraction: float) -> None:
        """Show progress on the LEDs.

        Args:
            fraction (float): A value between 0.0 and 1.0 representing progress.
        """
        self._desiredPercent = max(0.0, min(1.0, fraction))

    def showShift(self, shift: const.TeleopShift, canScoreInHub: bool) -> None:
        """Pick a color to show the current phase, shift, and whether the hub is active for our alliance or not."""
        if DriverStation.isDisabled():
            self.setColor(STATE_COLORS[("disabled", False)])
            self.showProgress(1.0)

        elif DriverStation.isTest():
            self.setColor(STATE_COLORS[("test", False)])
            self.showProgress(1.0)

        elif DriverStation.isAutonomous():
            self.setColor(STATE_COLORS[("autonomous", True)])
            self.showProgress(wpilib.Timer.getMatchTime() / const.PhaseDuration.AUTONOMOUS)

        elif DriverStation.isTeleop():
            # Get the color for the shift and whether we can score in the hub
            # Show magenta if we don't have a defined color for this state
            # (which should only happen if it says we cannot score in transition or endgame)
            timeLeftInShift: units.seconds = wpilib.Timer.getMatchTime() - shift.endTime
            fractionLeftInShift: float = timeLeftInShift / shift.duration if shift.duration > 0 else 0

            color: wpilib.Color8Bit = STATE_COLORS.get((shift.value, canScoreInHub), wpilib.Color8Bit(255, 0, 255))
            self.setColor(color)
            self.showProgress(fractionLeftInShift)

    def setColor(self, color: wpilib.Color8Bit) -> None:
        """Set the LED color."""
        self._desiredColor: wpilib.Color8Bit = color

    def execute(self) -> None:
        """Update the LED hardware states based on the current settings."""
        # TODO: can we improve performance by only updating when something changes?
        litCount = int(LIGHTS_IN_STRIP * self._desiredPercent)
        for i, led in enumerate(self.ledBuffer):
            if i < litCount:
                led.setLED(self._desiredColor)
            else:
                led.setLED(OFF_COLOR)

        self.ledController.setData(self.ledBuffer)
