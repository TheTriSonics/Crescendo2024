from wpilib import Joystick


# This is the controller for the person not operating the drivetrain
class CommanderController():

    def __init__(self, joystick: Joystick):
        self.joystick = joystick

    def get_intake_on(self) -> bool:
        return self.joystick.getRawButton(1)

    def get_diverter_shooter(self) -> bool:
        return self.joystick.getRawButton(3)

    def get_diverter_amp(self) -> bool:
        return self.joystick.getRawButton(4)

    def get_tilt_up(self) -> bool:
        return self.joystick.getRawButton(5)

    def get_tilt_down(self) -> bool:
        return self.joystick.getRawButton(6)

    def get_intake_reverse(self) -> bool:
        return self.joystick.getRawButton(2)

    def get_intake_override(self) -> bool:
        return self.joystick.getRawButton(20)

    def set_shooter_shot_protected(self) -> bool:
        return self.joystick.getRawButton(7)

    def set_shooter_shot_subwoofer(self) -> bool:
        return self.joystick.getRawButton(8)