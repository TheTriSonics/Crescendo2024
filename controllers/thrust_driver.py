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
        return -self.joystick.getRawAxis(1)

    @square
    @deadband(0.04)
    def get_drive_y(self) -> float:
        return -self.joystick.getRawAxis(0)

    @square
    @deadband(0.04)
    def get_drive_rot(self) -> float:
        return -self.joystick.getRawAxis(5)

    @deadband(0.01)
    def get_master_throttle(self) -> float:
        rawval = self.joystick.getRawAxis(2)
        rawval = -rawval
        rawval += 1
        rawval /= 2
        return rawval

    def get_note_lockon(self) -> bool:
        return self.joystick.getRawButton(1)

    def get_field_relative_toggle(self) -> bool:
        return self.joystick.getRawButton(3)
    
    def get_climber_trigger(self) -> float:
        # TODO: Set the correct axis id
        return self.joystick.getRawAxis(0)
