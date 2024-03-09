from commands2 import Command

from subsystems.shooter import Shooter


class ShooterMove(Command):
    def __init__(self, shooter: Shooter, dir) -> None:
        super().__init__()
        self.shooter = shooter
        self.dir = dir
        self.addRequirements(shooter)

    def initialize(self) -> None:
        pass
            
    def execute(self) -> None:
        if self.dir == 1:
            self.shooter.tilt_target = self.shooter.tilt_encoder.getAbsolutePosition() + 0.005
        elif self.dir == -1:
            self.shooter.tilt_target = self.shooter.tilt_encoder.getAbsolutePosition() - 0.005
        pass

    def end(self, isInterrupted) -> None:
        pass

    def isFinished(self) -> bool:
        return False

