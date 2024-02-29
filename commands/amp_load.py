from commands2 import Command
from wpilib import Timer

class AmpLoad(Command):
    def __init__(self, amp, photoeyes):
        super().__init__()
        self.amp = amp
        self.photoeyes = photoeyes
        self.timer = Timer()
        self.addRequirements(amp)

    def initialize(self):
        self.timer.start()

    def execute(self):
        pass

    def end(self):
        pass

    def isFinished(self):
        if self.timer.get() > 3.0:
            return True
        return False