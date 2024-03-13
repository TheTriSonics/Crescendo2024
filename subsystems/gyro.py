from phoenix6.hardware import Pigeon2
from commands2 import Subsystem

from constants import RobotSensorMap as RSM


class Gyro(Subsystem):
    def __init__(self):
        super().__init__()
        self.gyro = Pigeon2(RSM.pigeon2_id, "canivore")
        # self.gyro = Pigeon2(RSM.pigeon2_id)

    def get_yaw(self) -> float:
        return self.gyro.get_yaw().value

    def set_yaw(self, x):
        self.gyro.set_yaw(x)
