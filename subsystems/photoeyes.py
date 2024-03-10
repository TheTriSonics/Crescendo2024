from wpilib import SmartDashboard, DigitalInput
from commands2 import Subsystem
from constants import RobotSensorMap as RSM

class Photoeyes(Subsystem):
    def __init__(self):
        super().__init__()
        self.intake_front_photoeye = DigitalInput(RSM.intake_front_photoeye)
        self.intake_hold_photoeye = DigitalInput(RSM.intake_hold_photoeye)
        self.amp_hold_photoeye = DigitalInput(RSM.amp_hold_photoeye)
        self.shooter_hold_photoeye = DigitalInput(RSM.shooter_hold_photoeye)

    def get_intake_front(self) -> bool:
        return not self.intake_front_photoeye.get()

    def get_intake_loaded(self) -> bool:
        return not self.intake_hold_photoeye.get()

    def get_shooter_loaded(self) -> bool:
        return not self.shooter_hold_photoeye.get()

    def get_amp_loaded(self) -> bool:
        return not self.amp_hold_photoeye.get()

    def periodic(self) -> None:
        pb = SmartDashboard.putBoolean
        intake_front = self.get_intake_front()
        pb("photoeyes/intake_front", intake_front)
        pb("photoeyes/intake_full", self.get_intake_loaded())
        pb("photoeyes/shooter_loaded", self.get_shooter_loaded())
        pb("photoeyes/amp_loaded", self.get_amp_loaded())
