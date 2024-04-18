from wpilib import Timer
from commands2 import Command

from controllers.commander import CommanderController
from subsystems.shooter import Shooter
from subsystems.amp import Amp 
from phoenix6.controls import DutyCycleOut


class ShooterLaunchNote(Command):
    def __init__(self, shooter: Shooter, amp: Amp,
                 controller: CommanderController) -> None:
        super().__init__()
        self.shooter = shooter
        self.timer = Timer()
        self.amp = amp
        self.controller = controller
        self.shot_timer = Timer()
        self.addRequirements(shooter)

    def initialize(self) -> None:
        self.shot_fired = False
        self.timer.restart()

    def execute(self) -> None:
        ovr = self.controller.get_override_shooter_speed()
        if self.shooter.is_up_to_speed() or ovr:
            if not self.shot_fired:
                self.amp.reverse()
                self.shooter.feed_note()
                self.shot_fired = True
                self.shot_timer.restart()

    def end(self, interrupted: bool) -> None:
        self.amp.halt()
        self.shooter.feed_off()
        self.shooter.prepare_to_load()
        self.shooter.shooter_motor_left.set_control(DutyCycleOut(0.0))
        self.shooter.shooter_motor_right.set_control(DutyCycleOut(0.0))

    def isFinished(self) -> bool:
        return self.shot_fired and self.shot_timer.hasElapsed(1.0)
