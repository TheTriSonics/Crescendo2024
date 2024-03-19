from wpilib import Timer
from commands2 import Command
from subsystems.intake import Intake
from subsystems.shooter import Shooter
from subsystems.amp import Amp
from subsystems.photoeyes import Photoeyes


running = False


class IntakeNote(Command):
    def __init__(self, intake: Intake, shooter: Shooter, amp: Amp,
                 photoeyes: Photoeyes):
        super().__init__()
        self.intake = intake
        self.shooter = shooter
        self.amp = amp
        self.photoeyes = photoeyes
        self.addRequirements(intake)
        self.timer = Timer()
        self.timer.start()

    def initialize(self):
        running = True
        self.forceQuit = False
        shooter_loaded = self.photoeyes.get_shooter_loaded()
        amp_loaded = self.photoeyes.get_amp_loaded()
        if shooter_loaded or amp_loaded:
            self.forceQuit = True
        self.timer.restart()

    def execute(self):
        if self.forceQuit:
            return
        self.intake.feed()
        if self.photoeyes.get_intake_loaded():
            self.intake.halt()
            self.forceQuit = True
        pass

    def end(self, interrupted: bool):
        running = False
        self.intake.halt()
        pass

    # added a timer to the isFinished method @ Ethan #1
    def isFinished(self):
        if self.forceQuit:
            return True
        # if self.timer.get() > 3.0:
        #     return True
        return False
