from wpilib import Joystick
from misc import square, deadband


# This is the controller for the person in charge of the drive train and
# any other tasks required by them.
class DriverController():

    def __init__(self, joystick: Joystick):
        self.joystick = joystick

    @square
    @deadband(0.04)
    def get_drive_x(self) -> float:
        return self.joystick.getRawAxis(0)

    @square
    @deadband(0.04)
    def get_drive_y(self) -> float:
        return self.joystick.getRawAxis(1)

    @square
    @deadband(0.04)
    def get_drive_rot(self) -> float:
        return self.joystick.getRawAxis(4)

    # Controller has no equivalent for this method
    # so default to full throttle
    def get_master_throttle(self) -> float:
        return 1

    # Toggles whether or the robot is in robot relative
    # or field relative mode
    def get_field_relative_toggle(self) -> bool:
        return self.joystick.getRawButton(3)
