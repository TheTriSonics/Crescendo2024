from commands2 import Command
from wpilib import Timer

from subsystems.intake import Intake
from subsystems.photoeyes import Photoeyes
from subsystems.amp import Amp


class AmpLoad(Command):
    def __init__(self, amp: Amp, intake: Intake, photoeyes: Photoeyes) -> None:
        super().__init__()
        self.amp = amp
        self.photoeyes = photoeyes
        self.timer = Timer()
        self.forceQuit = False
        self.addRequirements(amp)

    def initialize(self) -> None:
        if not self.intake.is_loaded():
            self.forceQuit = True
        self.timer.start()

    def execute(self) -> None:
        if self.forceQuit is True:
            return

        self.intake.feed()
        self.amp.load()
        pass

    def end(self, isInterrupted) -> None:
        self.intake.halt()
        self.amp.halt()
        pass

    def isFinished(self) -> bool:
        if self.forceQuit is True:
            return True
        if self.timer.get() > 3.0:
            return True
        if self.photoeyes.get_amp_loaded() is True:
            return True
        return False

