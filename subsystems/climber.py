from commands2 import Subsystem
from rev import CANSparkMax

from constants import RobotMotorMap as RMM


class ClimberSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        # Initialize the motor controller
        self.climber_motor_l = CANSparkMax(RMM.climber_motor_left)
        self.climber_motor_r = CANSparkMax(RMM.climber_motor_right)
        
        self.climber_motor_l.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.climber_motor_r.setIdleMode(CANSparkMax.IdleMode.kBrake)

        self.climber_motor_r.follow(self.climber_motor_l, invertOutput=False)
        
    def go_up(self):
        # Set the motor to go up
        self.climber_motor_l.set(1.0)

    def go_down(self):
        # Set the motor to go down
        self.climber_motor_l.set(-1.0)