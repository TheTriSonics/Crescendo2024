from wpilib import Timer
from commands2 import Command
from subsystems.intake import Intake
from subsystems.shooter import Shooter
from subsystems.amp import Amp
from subsystems.photoeyes import Photoeyes
from subsystems.leds import Leds

class IntakeNote(Command):
    def __init__(self, intake:Intake, shooter:Shooter, amp:Amp, photoeyes:Photoeyes, leds:Leds):
        super().__init__()
        self.intake = intake
        self.shooter = shooter
        self.amp = amp
        self.photoeyes = photoeyes
        self.leds = leds
        self.addRequirements(intake)

    def initialize(self):
        # shooter_loaded = False
        # amp_loaded = False
        # intake_loaded = False
        # for c in [shooter_loaded, amp_loaded, intake_loaded]:
        #    if c is True:
        #         self.forceQuit = True
        #         return
        self.intake.tilt_down()
        self.timer = Timer()
        self.forceQuit = False
        if self.photoeyes.get_shooter_loaded() or self.photoeyes.get_amp_loaded():
            self.forceQuit = True
        self.timer.start()

    def execute(self):
        self.leds.intake_running()
        if self.forceQuit:
            return
        self.intake.feed()
        if self.photoeyes.get_intake_loaded():
            self.intake.halt()
            self.intake.tilt_up()
            self.leds.intake_loaded()
            self.forceQuit = True
        pass

    def end(self, interrupted: bool):
        self.intake.halt()
        self.intake.tilt_up()
        pass

    def isFinished(self):
        if self.forceQuit:
            return True
        # if self.timer.get() > 3.0:
        #     return True
        return False
