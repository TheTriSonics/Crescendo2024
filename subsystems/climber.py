from wpilib import SmartDashboard, DutyCycleEncoder
from wpimath.controller import PIDController
from commands2 import Command, Subsystem
from rev import CANSparkLowLevel, CANSparkMax

from constants import RobotMotorMap as RMM
from constants import RobotSensorMap as RSM
from controllers.driver import DriverController


# Totally made up numbers right now
left_down = 0.548
right_down = 0.227
left_up = 1.5
right_up = 1.3

deadband_limit = 0.05


class Climber(Subsystem):
    def __init__(self, controller: DriverController):
        super().__init__()

        self.controller = controller

        # defcmd = ClimberDefaultCommand(self, controller)
        # self.setDefaultCommand(defcmd)

        # Initialize the motor controller
        self.climber_motor_l = CANSparkMax(RMM.climber_motor_left,
                                           CANSparkLowLevel.MotorType.kBrushed)
        self.climber_motor_r = CANSparkMax(RMM.climber_motor_right,
                                           CANSparkLowLevel.MotorType.kBrushed)
        self.encoder_l = DutyCycleEncoder(RSM.climber_left_encoder)
        self.encoder_r = DutyCycleEncoder(RSM.climber_right_encoder)
        self.left_setpoint = left_down
        self.right_setpoint = right_down

        p, i, d = 5, 0, 0
        self.left_pid = PIDController(p, i, d)
        self.right_pid = PIDController(p, i, d)

        self.climber_motor_l.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.climber_motor_r.setIdleMode(CANSparkMax.IdleMode.kBrake)

        # self.climber_motor_r.follow(self.climber_motor_l, invertOutput=False)

    def set_speed(self, speed):
        self.climber_motor_l.set(speed)
        self.climber_motor_r.set(speed)

    def go_up(self):
        self.left_setpoint = left_up
        self.right_setpoint = right_up

    def go_down(self):
        self.left_setpoint = left_down
        self.right_setpoint = right_down

    def periodic(self):
        pn = SmartDashboard.putNumber
        lpos = self.encoder_l.getAbsolutePosition()
        rpos = self.encoder_r.getAbsolutePosition()
        lpower, rpower = 0, 0
        if abs(lpos - self.left_setpoint) > deadband_limit:
            lpower = self.left_pid.calculate(lpos, self.left_setpoint)

        if abs(rpos - self.right_setpoint) > deadband_limit:
            rpower = self.right_pid.calculate(rpos, self.right_setpoint)
        self.climber_motor_l.set(lpower)
        self.climber_motor_r.set(rpower)
        pn('climber/left/encoder', lpos)
        pn('climber/right/encoder', rpos)


class ClimberDefaultCommand(Command):
    def __init__(self, climber: Climber, controller: DriverController):
        self.climber = climber
        self.controller = controller
        self.addRequirements(climber)

    def execute(self):
        # self.climber.set_speed(self.controller.get_climber_trigger())
        pass
