from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut

from constants import RobotMap


class ClimberSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        # Initialize the motor controller
        self.climber_motor_l = TalonFX(RobotMap.climber_left)
        self.climber_motor_r = TalonFX(RobotMap.climber_right)
        
    def go_up(self):
        # Set the motor to go up
        self.climber_motor.set_control(
            DutyCycleOut(1.0, override_brake_dur_neutral=True)
        )

    def go_down(self):
        # Set the motor to go down
        self.climber_motor.set_control(
            DutyCycleOut(-1.0, override_brake_dur_neutral=True)
        )

    def stop(self):
        # Stop the motor
        self.climber_motor.set_control(
            DutyCycleOut(0, override_brake_dur_neutral=True)
        )
