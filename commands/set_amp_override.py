from commands2 import Command

from subsystems.amp import Amp


class SetAmpOverride(Command):
    def __init__(self, amp: Amp, dir) -> None:
        super().__init__()
        self.amp = amp
        self.dir = dir
        self.addRequirements(amp)

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        if self.dir == 1:
            self.amp.height += 0.05
        elif self.dir == -1:
            self.amp.height -= 0.05
        pass

    def end(self, isInterrupted) -> None:
        pass

    def isFinished(self) -> bool:
        return False

