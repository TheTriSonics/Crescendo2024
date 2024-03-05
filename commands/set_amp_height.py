from commands2 import Command
from wpilib import Timer

from subsystems.intake import Intake
from subsystems.amp import Amp


class AmpLoad(Command):
    def __init__(self, amp: Amp, intake: Intake, height: Amp.Height) -> None:
        super().__init__()
        self.amp = amp
        self.intake = intake
        self.height = height
        self.addRequirements(amp)

    def initialize(self) -> None:
        self.amp.set_height(self.height)

    def execute(self) -> None:
        pass

    def end(self, isInterrupted) -> None:
        pass

    def isFinished(self) -> bool:
        return True

