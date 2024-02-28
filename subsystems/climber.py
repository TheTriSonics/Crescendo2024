from commands2 import Command, Subsystem
from rev import CANSparkMax

from constants import RobotMotorMap as RMM
from controllers.commander import CommanderController
from controllers.thrust_driver import DriverController


class ClimberSubsystem(Subsystem):
    def __init__(self, controller: DriverController):
        super().__init__()

        self.controller = controller

        # Initialize the motor controller
        self.climber_motor_l = CANSparkMax(RMM.climber_motor_left)
        self.climber_motor_r = CANSparkMax(RMM.climber_motor_right)
        
        self.climber_motor_l.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.climber_motor_r.setIdleMode(CANSparkMax.IdleMode.kBrake)

        self.climber_motor_r.follow(self.climber_motor_l, invertOutput=False)

    def set_speed(self, speed):
        # Set the motor to a specific speed
        self.climber_motor_l.set(speed)
        
    def go_up(self):
        # Set the motor to go up
        self.climber_motor_l.set(1.0)

    def go_down(self):
        # Set the motor to go down
        self.climber_motor_l.set(-1.0)

class ClimberDefaultCommand(Command):
    def __init__(self, climber: ClimberSubsystem, controller: DriverController):
        super().__init__(climber)
        self.climber = climber
        self.controller = controller
        self.addRequirements(climber)

    def execute(self):
        self.climber.set_speed(self.controller.get_climber_trigger())