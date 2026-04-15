"""LED control component for the robot."""

import math

import wpilib
import wpimath.units as units
from wpilib import DriverStation

import constants as const
import utils

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

LIGHTS_IN_STRIP: int = 50
LIGHTS_ON_SIDE_LEFT = range(0, 19)
LIGHTS_ON_SIDE_RIGHT = range(49, 30, -1)
LIGHTS_ON_BOTTOM = range(19, 31)

# The color to show when the LED is off.
OFF_COLOR: wpilib.Color8Bit = wpilib.Color8Bit(0, 0, 0)

# Pulse the brightness of the color.
FADE_PULSING_MIN_BRIGHTNESS: float = 0.75
FADE_PULSING_MAX_BRIGHTNESS: float = 0.9
FADE_PULSING_RATE: units.hertz = 0.5

# Bright spark that travels from middle to ends.
SPARK_LIGHTS_PER_SECOND: float = 15.0
SPARK_BLEND_PERCENT: float = 0.6


class Lighting:
    """Controls the LED lighting system on the robot."""

    ledController: wpilib.AddressableLED

    def setup(self) -> None:
        """Initialize the LED hardware once it's been created.

        Called once by MagicBot after all the variable injection has finished.
        """
        self._desiredPercent: float = 1.0
        self._desiredColor: wpilib.Color8Bit = wpilib.Color8Bit(255, 0, 255)
        self._pulsePosition: float = 0.0
        self._lastPulseTime: float = 0.0

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
        self._desiredColor = color

    def execute(self) -> None:
        """Update the LED hardware states based on the current settings."""
        # Pulse the brightness of the color.
        t = wpilib.Timer.getFPGATimestamp()
        fade_range: float = FADE_PULSING_MAX_BRIGHTNESS - FADE_PULSING_MIN_BRIGHTNESS
        pulse_scale: float = math.sin(2.0 * math.pi * FADE_PULSING_RATE * t) * fade_range + FADE_PULSING_MIN_BRIGHTNESS
        self._desiredColor = utils.scaleColor(self._desiredColor, pulse_scale)

        litCountSideLeft = int(len(LIGHTS_ON_SIDE_LEFT) * self._desiredPercent)
        litCountSideRight = int(len(LIGHTS_ON_SIDE_RIGHT) * self._desiredPercent)
        for idx, i in enumerate(LIGHTS_ON_SIDE_LEFT):
            led = self.ledBuffer[i]
            if idx >= len(LIGHTS_ON_SIDE_LEFT) - litCountSideLeft:
                led.setLED(self._desiredColor)
            else:
                led.setLED(OFF_COLOR)

        for idx, i in enumerate(LIGHTS_ON_SIDE_RIGHT):
            led = self.ledBuffer[i]
            if idx >= len(LIGHTS_ON_SIDE_RIGHT) - litCountSideRight:
                led.setLED(self._desiredColor)
            else:
                led.setLED(OFF_COLOR)

        bottom_on = litCountSideLeft > 0 or litCountSideRight > 0
        for i in LIGHTS_ON_BOTTOM:
            led = self.ledBuffer[i]
            if bottom_on:
                led.setLED(self._desiredColor)
            else:
                led.setLED(OFF_COLOR)

        # White pulse traveling from middle of bottom to the lit edge on each side
        halfBottom = len(LIGHTS_ON_BOTTOM) // 2
        if litCountSideLeft > 0:
            pathLength = halfBottom + litCountSideLeft
            if self._lastPulseTime > 0:
                dt = t - self._lastPulseTime
                self._pulsePosition += SPARK_LIGHTS_PER_SECOND * dt
                while self._pulsePosition >= pathLength:
                    self._pulsePosition -= pathLength
            self._lastPulseTime = t

            pulseBlend = wpilib.Color8Bit(
                int(self._desiredColor.red + (255 - self._desiredColor.red) * SPARK_BLEND_PERCENT),
                int(self._desiredColor.green + (255 - self._desiredColor.green) * SPARK_BLEND_PERCENT),
                int(self._desiredColor.blue + (255 - self._desiredColor.blue) * SPARK_BLEND_PERCENT),
            )
            pos = int(self._pulsePosition)
            self.ledBuffer[LIGHTS_ON_BOTTOM[halfBottom - 1] - pos].setLED(pulseBlend)
            self.ledBuffer[LIGHTS_ON_BOTTOM[halfBottom] + pos].setLED(pulseBlend)
        else:
            self._lastPulseTime = t
            self._pulsePosition = 0.0

        self.ledController.setData(self.ledBuffer)
