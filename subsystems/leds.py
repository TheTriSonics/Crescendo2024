from commands2 import Subsystem
from wpilib import AddressableLED, DriverStation
from constants import RobotSensorMap as RSM


class Leds(Subsystem):
    def __init__(self):
        super().__init__()

        self.leds = AddressableLED(RSM.addressable_leds)

        self.led_length = 30 #TODO: Set the correct length

        self.leds.setLength(self.led_length)

        self.led_data = [AddressableLED.LEDData() for _ in range(self.led_length)]

        self.rainbowFirstPixelHue = 0

        self.leds.setData(self.led_data)
        self.leds.start()

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

    def set_color(self, color, brightness=255):
        for i in range(self.led_length):
            self.led_data[i].setHSV(color, 255, brightness)
    
    def set_connect_status(self):
        if DriverStation.isDSAttached():
            self.set_color(120)
        else:
            self.set_color(0)
    
    def periodic(self) -> None:
        self.leds.setData(self.led_data)