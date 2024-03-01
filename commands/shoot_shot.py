from wpilib import Timer
from commands2 import Command

from subsystems.shooter import Shooter

class ShootShot(Command):
    def __init__(self, shooter: Shooter, on_the_fly: bool = True, protected_shot=False, speaker_shot=False):
        self.shooter = shooter
        self.on_the_fly = on_the_fly
        self.protected_shot = protected_shot
        self.speaker_shot = speaker_shot
        self.timer = Timer()
        self.addRequirements(shooter)

    def initialize(self):
        if self.protected_shot:
            self.shooter.set_protected_shot()
        elif self.speaker_shot:
            self.shooter.set_speaker_shot()
        else:
            self.shooter.set_auto_target_shot()
            
    def execute(self):
        if self.shooter.is_up_to_speed():
            self.shooter.feed_note()
            self.timer.start()

    def end(self, interrupted: bool):
        self.shooter.feed_off()
        self.shooter.halt()

    def isFinished(self) -> bool:
        return self.timer.get() > 0.5