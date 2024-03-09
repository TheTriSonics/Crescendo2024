from wpilib import SmartDashboard, DutyCycleEncoder
from commands2 import Command, Subsystem
from rev import CANSparkLowLevel, CANSparkMax
from commands.climb import Climb

from constants import RobotMotorMap as RMM
from constants import RobotSensorMap as RSM
from controllers.driver import DriverController


class Climber(Subsystem):
    def __init__(self, controller: DriverController):
        super().__init__()

        self.controller = controller

        defcmd = ClimberDefaultCommand(self, controller)
        self.setDefaultCommand(defcmd)

        # Initialize the motor controller
        self.climber_motor_l = CANSparkMax(RMM.climber_motor_left, CANSparkLowLevel.MotorType.kBrushed)
        self.climber_motor_r = CANSparkMax(RMM.climber_motor_right, CANSparkLowLevel.MotorType.kBrushed)
        self.encoder_l = DutyCycleEncoder(RSM.climber_left_encoder) 
        self.encoder_r = DutyCycleEncoder(RSM.climber_right_encoder) 
        self.encoder_l.setDistancePerRotation(1)
        self.encoder_r.setDistancePerRotation(1)
        self.encoder_l.reset()
        self.encoder_r.reset()
        # self.encoder_l.setPositionOffset(.548)
        # self.encoder_r.setPositionOffset(.227)
        # self.loffset = self.encoder_l.getDistance()
        # self.roffset = self.encoder_r.getDistance()
        
        self.climber_motor_l.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.climber_motor_r.setIdleMode(CANSparkMax.IdleMode.kBrake)

        # self.climber_motor_r.follow(self.climber_motor_l, invertOutput=False)

    def set_speed(self, speed):
        # Set the motor to a specific speed
        self.climber_motor_l.set(speed)
        self.climber_motor_r.set(speed)
        
    def go_up(self):
        # Set the motor to go up
        self.climber_motor_l.set(1.0)

    def go_down(self):
        # Set the motor to go down
        self.climber_motor_l.set(-1.0)

    def periodic(self):
        pn = SmartDashboard.putNumber
        pn('climber/left/encoder', self.encoder_l.get())
        pn('climber/right/encoder', self.encoder_r.get())

class ClimberDefaultCommand(Command):
    def __init__(self, climber: Climber, controller: DriverController):
        self.climber = climber
        self.controller = controller
        self.addRequirements(climber)

    def execute(self):
        self.climber.set_speed(self.controller.get_climber_trigger())