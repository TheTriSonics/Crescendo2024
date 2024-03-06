from commands2 import Command
from wpilib import Timer

from subsystems.intake import Intake
from subsystems.photoeyes import Photoeyes
from subsystems.amp import Amp
from subsystems.shooter import Shooter


class ShooterLoad(Command):
    def __init__(self, amp: Amp, intake: Intake, shooter: Shooter, photoeyes: Photoeyes) -> None:
        super().__init__()
        self.amp = amp
        self.photoeyes = photoeyes
        self.intake = intake
        self.shooter = shooter
        self.addRequirements(amp)
        self.addRequirements(shooter)
        self.addRequirements(intake)

    def initialize(self) -> None:
        self.timer = Timer()
        self.forceQuit = False
        if not self.intake.is_loaded():
            self.forceQuit = True
        self.timer.start()

    def execute(self) -> None:
        if self.forceQuit is True:
            return
        self.intake.feed()
        self.amp.reverse()
        self.shooter.feed_note()
        pass

    def end(self, isInterrupted) -> None:
        self.intake.halt()
        self.amp.halt()
        self.shooter.feed_off()
        pass

    def isFinished(self) -> bool:
        if self.forceQuit is True:
            return True
        if self.timer.get() > 2.0:
            return True
        if self.photoeyes.get_shooter_loaded() is True:
            return True
        return False

