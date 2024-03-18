from misc import iterable
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


blink_loops = 25  # 25 loops at 20ms per loop is 0.5 seconds


RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
MAGENTA = (255, 0, 255)
CYAN = (0, 255, 255)
ORANGE = (255, 165, 0)
BROWN = (165, 42, 42)
PINK = (255, 192, 203)
PURPLE = (128, 0, 128)
TEAL = (0, 128, 128)
NAVY = (0, 0, 128)
GRAY = (128, 128, 128)
DARK_GRAY = (169, 169, 169)
LIGHT_GRAY = (211, 211, 211)
GOLD = (255, 215, 0)
SILVER = (192, 192, 192)
MAROON = (128, 0, 0)
OLIVE = (128, 128, 0)
LIME = (0, 128, 0)
AQUA = (0, 255, 255)
SKY_BLUE = (135, 206, 235)
INDIGO = (75, 0, 130)
TURQUOISE = (64, 224, 208)
VIOLET = (238, 130, 238)
BEIGE = (245, 245, 220)



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
        self.set_colorRGB((0, 255, 0))

        self.leds.setData(self.led_data)
        self.leds.start()

    def periodic(self) -> None:
        # Blinking red if the front intake photoeye is blocked
        # That means we're in some kind of error state
        # Next show various tracking modes
        tracking = (
            self.drive.is_note_tracking()
            or
            self.drive.is_speaker_tracking()
            or
            self.drive.is_amp_tracking()
        )
        visible = (
            self.drive.is_note_visible()
            or
            self.drive.is_speaker_visible()
            or
            self.drive.is_amp_visible()
        )

        if self.photoeyes.get_intake_front():
            self.set_colorRGB([ORANGE, RED, ORANGE], True)
        elif tracking:
            self.set_colorRGB([ORANGE, GREEN])
            if not visible:
                self.set_colorRGB([ORANGE, RED])
        elif self.photoeyes.get_intake_loaded():
            self.set_colorRGB(MAGENTA)

        elif self.photoeyes.get_amp_loaded():
            self.set_colorRGB(YELLOW)

        elif self.shooter.is_up_to_speed():
            self.set_colorRGB(WHITE)

        elif self.photoeyes.get_shooter_loaded():
            self.set_colorRGB(BLUE)

        else:
            self.set_colorRGB(BLACK)

        if self.rainbow_mode:
            self.rainbow()
        elif self.blinking:
            if self.blink_counter < blink_loops:
                self.blink_counter += 1
            else:
                self.blink_counter = 0
                self.blink_on = not self.blink_on
            if self.blink_on:
                self.set_colorRGB(self.curr_color)
            else:
                self.set_colorRGB((0, 0, 0))
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

    def set_colorRGB(self, color, blinking=False):
        self.rainbow_mode = False
        self.blinking = blinking

        self.curr_color = color
        if iterable(color[0]):
            n = len(color)
            chunk_size = self.led_length / n
            for i in range(self.led_length):
                idx = int(i // chunk_size)
                c = color[idx]
                self.led_data[i].setRGB(*c)
        else:
            for i in range(self.led_length):
                self.led_data[i].setRGB(*color)

    def set_colorHSV(self, color, blinking=False, brightness=255):
        self.rainbow_mode = False
        self.blinking = blinking
        self.curr_color = color
        for i in range(self.led_length):
            self.led_data[i].setHSV(color, 255, brightness)

    def set_connect_status(self):
        if DriverStation.isDSAttached():
            self.set_colorHSV(120)
        else:
            self.set_colorHSV(0)
