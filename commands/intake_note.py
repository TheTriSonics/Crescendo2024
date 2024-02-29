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
        self.timer = Timer()
        self.tiltTimer = Timer()
        self.forceQuit = False
        self.addRequirements(intake)

    def initialize(self):
        shooter_loaded = False
        amp_loaded = False
        intake_loaded = False
        for c in [shooter_loaded, amp_loaded, intake_loaded]:
           if c is True:
                self.forceQuit = True
                return
        self.timer.start()

    def execute(self):
        if self.forceQuit:
            return
        self.intake.feed()
        if not self.intake.wanted_down():
            self.intake.tilt_down()
        if self.photoeyes.get_intake_front():
            self.tiltTimer.start()
        if self.tiltTimer.get() > 1.0 and not self.intake.wanted_up():
            self.intake.tilt_up() 
        if self.photoeyes.get_intake_loaded():
            self.intake.halt()
            if not self.intake.wanted_up():
                self.intake.tilt_up()
            self.forceQuit = True
        pass

    def end(self):
        self.intake.halt()
        pass

    def isFinished(self):
        if self.forceQuit:
            return True
        if self.timer.get() > 3.0:
            return True
        return False