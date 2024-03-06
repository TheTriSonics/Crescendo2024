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
        self.addRequirements(shooter)

    def initialize(self) -> None:
        # self.shooter_set_speed(self.target_rpm)
        self.shot_fired = False

    def execute(self) -> None:
        # if self.shooter.is_up_to_speed():
        self.shooter.shooter_motor_left.set_control(DutyCycleOut(1.0))
        self.shooter.shooter_motor_right.set_control(DutyCycleOut(-1.0))
        self.timer.start()
        if not self.shot_fired and self.timer.hasElapsed(2.0):
            self.shooter.feed_note()
            self.shot_fired = True
            # self.timer.start()

    def end(self, interrupted: bool) -> None:
        self.shooter.feed_off()
        self.shooter.shooter_motor_left.set_control(DutyCycleOut(0.0))
        self.shooter.shooter_motor_right.set_control(DutyCycleOut(0.0))
        self.timer.reset()
        self.shot_fired = False


    def isFinished(self) -> bool:
        if self.timer.hasElapsed(3.0):
            return True
        return False
