from commands2 import Command
from wpilib import Timer

from subsystems.intake import Intake
from subsystems.photoeyes import Photoeyes
from subsystems.amp import Amp
from subsystems.shooter import Shooter


class ShooterLoad(Command):
    def __init__(self, amp: Amp, intake: Intake, shooter: Shooter,
                 photoeyes: Photoeyes) -> None:
        super().__init__()
        self.amp = amp
        self.photoeyes = photoeyes
        self.intake = intake
        self.shooter = shooter
        self.addRequirements(amp)
        self.addRequirements(shooter)
        self.addRequirements(intake)

    def initialize(self) -> None:
        print("shooter loading")
        self.timer = Timer()
        self.forceQuit = False
        self.shooter.prepare_to_load()
        self.timer.start()

    def execute(self) -> None:
        if self.forceQuit is True:
            return
        if not self.shooter.is_tilt_aimed():
            return
        self.intake.feed()
        self.amp.reverse()
        self.shooter.feed_note()
        pass

    def end(self, isInterrupted) -> None:
        print("shooter load done")
        self.intake.halt()
        self.amp.halt()
        self.shooter.feed_off()
        self.timer.stop()
        self.timer.reset()
        pass

    def isFinished(self) -> bool:
        if self.forceQuit is True:
            return True
        if self.timer.get() > 4.0:
            return True
        if self.photoeyes.get_shooter_loaded() is True:
            print("shooter load photoeye triggered")
            return True
        return False

