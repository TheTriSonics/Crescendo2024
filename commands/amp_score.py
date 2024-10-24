from commands2 import Command
from wpilib import Timer

from subsystems.photoeyes import Photoeyes
from subsystems.amp import Amp


class AmpScore(Command):
    def __init__(self, amp: Amp, photoeyes: Photoeyes) -> None:
        super().__init__()
        self.amp = amp
        self.photoeyes = photoeyes
        self.addRequirements(amp)

    def initialize(self) -> None:
        self.timer = Timer()
        self.dump_timer = Timer()
        self.forceQuit = False
        # if not self.photoeyes.get_amp_loaded():
        #     self.forceQuit = True
        #     pass
        self.timer.start()

    def execute(self) -> None:
        if self.forceQuit is True:
            return
        if not self.photoeyes.get_amp_loaded():
            self.dump_timer.start()
        self.amp.feed()
        pass

    def end(self, isInterrupted) -> None:
        self.amp.halt()
        self.amp.go_home()
        self.timer.stop()
        self.timer.reset()
        self.dump_timer.stop()
        self.dump_timer.reset()
        pass

    def isFinished(self) -> bool:
        if self.forceQuit is True:
            return True
        if self.timer.get() > 3.0:
            return True
        loaded = self.photoeyes.get_amp_loaded()
        timeout = self.dump_timer.get() > 0.5
        if loaded is False and timeout:
            return True
        return False

