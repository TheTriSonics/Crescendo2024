from wpilib import Joystick


# This is the controller for the person in charge of the drive train and
# any other tasks required by them.
class DriverController():

    def __init__(self, joystick: Joystick):
        self.joystick = joystick

    def get_drive_x(self) -> float:
        return self.joystick.getRawAxis(0)

    def get_drive_y(self) -> float:
        return self.joystick.getRawAxis(1)

    def get_drive_rot(self) -> float:
        return self.joystick.getRawAxis(4)

