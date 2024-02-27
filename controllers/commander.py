from wpilib import Joystick
from constants import RobotButtonMap as RM


# This is the controller for the person not operating the drivetrain
class CommanderController():

    def __init__(self, joystick1: Joystick, joystick2: Joystick):
        self.joystick1 = joystick1
        self.joystick2 = joystick2

    def get_intake_ready(self) -> bool:
        return self.joystick1.getRawButton(RM.intake_ready)

    def get_intake_eject(self) -> bool:
        return self.joystick1.getRawButton(RM.intake_eject)

    def get_load_shooter(self) -> bool:
        return self.joystick1.getRawButton(RM.load_note_shooter)

    def get_load_amp(self) -> bool:
        return self.joystick1.getRawButton(RM.load_note_amp)

    def get_load_back_to_home(self) -> bool:
        return self.joystick1.getRawButton(RM.load_note_back_to_home)

    def get_shooter_aim_otf(self) -> bool:
        return self.joystick1.getRawButton(RM.shooter_aim_otf)

    def get_shooter_aim_safe(self) -> bool:
        return self.joystick1.getRawButton(RM.shooter_aim_safe)

    def get_shooter_aim_sub(self) -> bool:
        return self.joystick1.getRawButton(RM.shooter_aim_sub)

    def get_shooter_shoot(self) -> bool:
        return self.joystick1.getRawButton(RM.shooter_shoot)
    
    def get_amp_lift_home(self) -> bool:
        return self.joystick2.getRawButton(RM.amp_lift_home)

    def get_amp_lift_amp(self) -> bool:
        return self.joystick2.getRawButton(RM.amp_lift_amp)

    def get_amp_lift_trap(self) -> bool:
        return self.joystick2.getRawButton(RM.amp_lift_trap)

    def get_amp_eject(self) -> bool:
        return self.joystick2.getRawButton(RM.amp_eject)

    def get_override_intake_tilt_up(self) -> bool:
        return self.joystick2.getRawButton(RM.intake_tilt_up)

    def get_override_intake_tilt_down(self) -> bool:
        return self.joystick2.getRawButton(RM.intake_tilt_down)

    def get_override_intake_roller_in(self) -> bool:
        return self.joystick2.getRawButton(RM.intake_roller_in)

    def get_override_intake_roller_out(self) -> bool:
        return self.joystick2.getRawButton(RM.intake_roller_out)