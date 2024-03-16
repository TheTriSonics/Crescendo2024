from wpilib import SmartDashboard, DigitalInput
from commands2 import Subsystem
from constants import RobotSensorMap as RSM

pn = SmartDashboard.putNumber
gn = SmartDashboard.getNumber
pb = SmartDashboard.putBoolean
gb = SmartDashboard.getBoolean

sdbase = 'fakesensors/photoeye'

class Photoeyes(Subsystem):
    def __init__(self):
        super().__init__()
        self.intake_front_photoeye = DigitalInput(RSM.intake_front_photoeye)
        self.intake_hold_photoeye = DigitalInput(RSM.intake_hold_photoeye)
        self.amp_hold_photoeye = DigitalInput(RSM.amp_hold_photoeye)
        self.shooter_hold_photoeye = DigitalInput(RSM.shooter_hold_photoeye)
        pb(f'{sdbase}/intake_front', False)
        pb(f'{sdbase}/intake_loaded', False)
        pb(f'{sdbase}/shooter_loaded', False)
        pb(f'{sdbase}/amp_loaded', False)

    def get_intake_front(self) -> bool:
        fake = gb(f'{sdbase}/intake_front', False)
        return not self.intake_front_photoeye.get() or fake

    def get_intake_loaded(self) -> bool:
        fake = gb(f'{sdbase}/intake_loaded', False)
        return not self.intake_hold_photoeye.get() or fake

    def get_shooter_loaded(self) -> bool:
        fake = gb(f'{sdbase}/shooter_loaded', False)
        return not self.shooter_hold_photoeye.get() or fake

    def get_amp_loaded(self) -> bool:
        fake = gb(f'{sdbase}/amp_loaded', False)
        return not self.amp_hold_photoeye.get() or fake

    def periodic(self) -> None:
        pb = SmartDashboard.putBoolean
        pb("photoeyes/intake_front", self.get_intake_front())
        pb("photoeyes/intake_full", self.get_intake_loaded())
        pb("photoeyes/shooter_loaded", self.get_shooter_loaded())
        pb("photoeyes/amp_loaded", self.get_amp_loaded())
