from commands2 import Subsystem
from wpilib import AddressableLED, DriverStation
from constants import RobotSensorMap as RSM
from subsystems.amp import Amp
from subsystems.shooter import Shooter
from subsystems.drivetrain import Drivetrain
from subsystems.note_tracker import NoteTracker
from subsystems.climber import Climber
from subsystems.photoeyes import Photoeyes
from subsystems.intake import Intake



"""
HSV values:
https://hyperskill.org/learn/step/13179

Red falls between 0 and 60 degrees.
Yellow falls between 61 and 120 degrees.
Green falls between 121 and 180 degrees.
Cyan falls between 181 and 240 degrees.
Blue falls between 241 and 300 degrees.
Magenta falls between 301 and 360 degrees.
"""

WHITE = 0
RED = 30
ORANGE = 60
YELLOW = 90
GREEN = 150
CYAN = 210
BLUE = 270
MAGENTA = 330

blink_loops = 25  # 25 loops at 20ms per loop is 0.5 seconds


class Leds(Subsystem):
    def __init__(self, amp: Amp, intake: Intake, shooter: Shooter,
                 drive: Drivetrain, note_tracker: NoteTracker,
                 climber: Climber, photoeyes: Photoeyes):
        super().__init__()
        self.leds = AddressableLED(RSM.addressable_leds)
        self.amp = amp
        self.intake = intake
        self.shooter = shooter
        self.drive = drive
        self.note_tracker = note_tracker
        self.climber = climber
        self.photoeyes = photoeyes

        self.led_length = 135
        self.blinking = False
        self.blink_counter = 0
        self.blink_on = True
        self.rainbow_mode = False
        self.leds.setLength(self.led_length)

        self.led_data = [AddressableLED.LEDData() for _ in range(self.led_length)]

        self.rainbowFirstPixelHue = 0
        self.intake_loaded()

        self.leds.setData(self.led_data)
        self.leds.start()

    def periodic(self) -> None:
        if False:
            if self.rainbow_mode:
                self.rainbow()
            elif self.blinking:
                if self.blink_counter < blink_loops:
                    self.blink_counter += 1
                else:
                    self.blink_counter = 0
                    if self.blink_on:
                        self.set_color(0)
                        self.blink_on = False
                    else:
                        self.set_color(self.curr_color)
                        self.blink_on = True
        self.leds.setData(self.led_data)

    def rainbow(self):
        # For every pixel
        for i in range(self.led_length):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = (self.rainbowFirstPixelHue + (i * 180 / self.led_length)) % 180

            # Set the value
            self.led_data[i].setHSV(int(hue), 255, 128)

        # Increase by to make the rainbow "move"
        self.rainbowFirstPixelHue += 3

        # Check bounds
        self.rainbowFirstPixelHue %= 180

    def set_color(self, color, blinking=False, brightness=255):
        self.rainbow_mode = False
        self.blinking = blinking
        self.blink_counter = 0
        self.curr_color = color
        for i in range(self.led_length):
            self.led_data[i].setHSV(color, 255, brightness)

    def set_connect_status(self):
        if DriverStation.isDSAttached():
            self.set_color(120)
        else:
            self.set_color(0)

    # Yellow, like Pac man. Gobble gobble, blink blink.
    def intake_running(self):
        self.set_color(YELLOW)

    # Pac man is sick
    def intake_ejecting(self):
        self.set_color(0)

    # Pac man full
    def intake_loaded(self):
        self.set_color(GREEN)

    def intake_front_blocked(self):
        self.set_color(RED, True)

    def intake_empty(self):
        pass

    def shooter_loaded(self):
        self.set_color(RED)

    def shooter_empty(self):
        pass

    def shooter_running(self):
        self.set_color(CYAN, True)
        pass

    def shooter_up_to_speed(self):
        self.set_color(BLUE, True)
        pass

    def amp_loaded(self):
        self.set_color(GREEN)
        pass

    def amp_empty(self):
        pass

    def amp_running(self):
        pass

    def amp_reversed(self):
        # self.set_color(RED, True)
        pass

    def tracking_note(self):
        # self.set_color(ORANGE)
        pass

    def tracking_note_not_found(self):
        # self.set_color(MAGENTA)
        pass

    def tracking_speaker(self):
        # self.set_color(YELLOW)
        pass

    def tracking_speaker_not_found(self):
        # self.set_color(MAGENTA)
        pass

    def off(self):
        self.set_color(0)

    def drivetrain_slow(self):
        # self.set_color(RED)
        pass