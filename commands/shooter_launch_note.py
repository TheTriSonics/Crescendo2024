from wpilib import Timer
from commands2 import Command

from subsystems.shooter import Shooter


class ShooterLaunchNote(Command):
    def __init__(self, shooter: Shooter, target_rpm) -> None:
        super().__init__()
        self.shooter = shooter
        self.target_rpm = target_rpm
        self.timer = Timer()
        self.addRequirements(shooter)

    def initialize(self) -> None:
        self.shooter_set_speed(self.target_rpm)
        self.shot_fired = False

    def execute(self) -> None:
        if self.shooter.is_up_to_speed():
            self.shooter.feed_note()
            self.shot_fired = True
            self.timer.start()

    def end(self, interrupted: bool) -> None:
        self.shooter.feed_off()

    def isFinished(self) -> bool:
        if self.timer.hasElapsed(3.0):
            return True
        return False
