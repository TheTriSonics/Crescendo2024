from commands2 import Command
from subsystems.gyro import Gyro


class GyroReset(Command):

    def __init__(self, gyro: Gyro) -> None:
        self.gyro = gyro

    def initialize(self):
        forward = 180 if self.drivetrain.shouldFlipPath() else 0
        self.gyro.set_yaw(forward)

    def execute(self):
        # We do nothing here; the init can handle
        # the gyro reset
        pass

    def isFinished(self):
        return True
