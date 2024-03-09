from wpilib import Timer
from commands2 import Command

from subsystems.shooter import Shooter
from phoenix6.controls import DutyCycleOut

import subsystems.shooter as sm


class AutoShooterLaunchNote(Command):
    def __init__(self, shooter: Shooter, tilt=sm.tilt_sub, rpm=75) -> None:
        super().__init__()
        self.shooter = shooter
        self.timer = Timer()
        self.shot_timer = Timer()
        self.tilt = tilt
        self.rpm = rpm
        self.addRequirements(shooter)

    def initialize(self) -> None:
        self.shooter.set_tilt(self.tilt)
        self.shooter.set_velocity(self.rpm)
        self.shooter.spin_up()
        self.shot_fired = False
        self.timer.restart()

    def execute(self) -> None:
        # TODO:
        # Add in check for vision targets here?
        up_to_speed = self.shooter.is_up_to_speed()
        aimed = self.shooter.is_tilt_aimed()
        if not self.shot_fired and up_to_speed and aimed:
            self.shooter.feed_note()
            self.shot_fired = True
            self.shot_timer.restart()

    def end(self, interrupted: bool) -> None:
        self.shooter.feed_off()
        self.shooter.prepare_to_load()
        self.shooter.spin_down()
        self.shooter.shooter_motor_left.set_control(DutyCycleOut(0.0))
        self.shooter.shooter_motor_right.set_control(DutyCycleOut(0.0))

    def isFinished(self) -> bool:
        return self.shot_fired and self.shot_timer.hasElapsed(0.5)
