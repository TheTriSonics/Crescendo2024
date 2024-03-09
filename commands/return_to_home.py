from wpilib import Timer
from commands2 import Command
from subsystems.intake import Intake
from subsystems.shooter import Shooter
from subsystems.amp import Amp
from subsystems.photoeyes import Photoeyes
from subsystems.leds import Leds

class ReturnToHome(Command):
    def __init__(self, intake:Intake, shooter:Shooter, amp:Amp, photoeyes:Photoeyes, leds:Leds):
        super().__init__()
        self.intake = intake
        self.shooter = shooter
        self.amp = amp
        self.photoeyes = photoeyes
        self.leds = leds
        self.addRequirements(intake)

    def initialize(self):
        self.timer = Timer()
        self.forceQuit = False
        self.tripped = False
        if self.photoeyes.get_intake_loaded():
            self.forceQuit = True

    def execute(self):
        if self.forceQuit:
            return
        if self.photoeyes.get_amp_loaded():
            self.amp.reverse()
        elif self.photoeyes.get_shooter_loaded():
            self.shooter.reverse()
        self.intake.reverse()
        if self.photoeyes.get_intake_loaded() and not self.tripped:
            self.tripped = True
            self.timer.start()
        if self.timer.hasElapsed(0.1):
            self.intake.halt()
            self.amp.halt()
            self.forceQuit = True
        pass

    def end(self, interrupted: bool):
        self.intake.halt()
        self.amp.halt()
        pass

    def isFinished(self):
        if self.forceQuit:
            return True
        if self.timer.get() > 3.0:
            return True
        return False