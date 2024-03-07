from wpilib import Timer
from commands2 import Command

from subsystems.shooter import Shooter
from phoenix6.controls import DutyCycleOut


class ShooterLaunchNoteTest(Command):
    def __init__(self, shooter: Shooter) -> None:
        super().__init__()
        self.shooter = shooter
        self.target_rpm = 0
        self.timer = Timer()
        self.shot_timer = Timer()
        self.addRequirements(shooter)

    def initialize(self) -> None:
        # self.shooter_set_speed(self.target_rpm)
        self.shot_fired = False
        self.shooter.set_velocity(78)
        self.timer.restart()

    def execute(self) -> None:
        # if self.shooter.is_up_to_speed():
        if not self.shot_fired and self.timer.hasElapsed(5.0):
            self.shooter.feed_note()
            self.shot_fired = True
            self.shot_timer.restart()
            # self.timer.start()

    def end(self, interrupted: bool) -> None:
        self.shooter.feed_off()
        self.shooter.shooter_motor_left.set_control(DutyCycleOut(0.0))
        self.shooter.shooter_motor_right.set_control(DutyCycleOut(0.0))

    def isFinished(self) -> bool:
        return self.shot_fired and self.shot_timer.hasElapsed(1.0)
