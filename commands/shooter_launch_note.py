from wpilib import Timer
from commands2 import Command

from subsystems.shooter import Shooter
from phoenix6.controls import DutyCycleOut


class ShooterLaunchNote(Command):
    def __init__(self, shooter: Shooter) -> None:
        super().__init__()
        self.shooter = shooter
        self.timer = Timer()
        self.shot_timer = Timer()
        self.addRequirements(shooter)

    def initialize(self) -> None:
        self.shot_fired = False
        self.timer.restart()

    def execute(self) -> None:
        if not self.shot_fired:
            self.shooter.feed_note()
            self.shot_fired = True
            self.shot_timer.restart()

    def end(self, interrupted: bool) -> None:
        self.shooter.feed_off()
        self.shooter.prepare_to_load()
        self.shooter.shooter_motor_left.set_control(DutyCycleOut(0.0), override_brake_dur_neutral=True)
        self.shooter.shooter_motor_right.set_control(DutyCycleOut(0.0), override_brake_dur_neutral=True)

    def isFinished(self) -> bool:
        return self.shot_fired and self.shot_timer.hasElapsed(1.0)
