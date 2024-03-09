from wpilib import Timer
from commands2 import Command
from subsystems.intake import Intake
from subsystems.photoeyes import Photoeyes
from subsystems.leds import Leds


class EjectNote(Command):
    def __init__(self, intake: Intake, photoeyes: Photoeyes, leds: Leds):
        super().__init__()
        self.intake = intake
        self.photoeyes = photoeyes
        self.leds = leds
        self.addRequirements(intake)

    def initialize(self):
        self.intake.tilt_down()
        self.timer = Timer()
        self.forceQuit = False
        shooter_loaded = self.photoeyes.get_shooter_loaded()
        amp_loaded = self.photoeyes.get_amp_loaded()
        # We don't want two notes at once!
        if shooter_loaded or amp_loaded:
            self.forceQuit = True
        self.timer.start()

    def execute(self):
        print("Ejecting Note")
        if self.forceQuit:
            return
        if self.intake.is_down():
            self.intake.reverse()

    def end(self, interrupted: bool):
        print("Eject done")
        self.intake.halt()
        self.intake.tilt_up()

    def isFinished(self):
        if self.forceQuit:
            return True
        if self.timer.get() > 2.0:
            return True
        return False
