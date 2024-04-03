from wpilib import SmartDashboard, DutyCycleEncoder
from commands2 import Command, Subsystem
from rev import CANSparkLowLevel
from revshim import CANSparkMax

from constants import RobotMotorMap as RMM
from constants import RobotSensorMap as RSM
from controllers.commander import CommanderController
from controllers.driver import DriverController


class Climber(Subsystem):
    def __init__(self, controller: DriverController, op_controller: CommanderController):
        super().__init__()

        self.controller = controller
        self.op_controller = op_controller

        defcmd = ClimberDefaultCommand(self, controller, op_controller)
        self.setDefaultCommand(defcmd)

        # Initialize the motor controller
        self.climber_motor_l = CANSparkMax(RMM.climber_motor_left,
                                           CANSparkLowLevel.MotorType.kBrushed)
        self.climber_motor_r = CANSparkMax(RMM.climber_motor_right,
                                           CANSparkLowLevel.MotorType.kBrushed)
        self.encoder_l = DutyCycleEncoder(RSM.climber_left_encoder)
        self.encoder_r = DutyCycleEncoder(RSM.climber_right_encoder)
        self.encoder_l.setDistancePerRotation(1)
        self.encoder_r.setDistancePerRotation(1)
        self.encoder_l_zero = 2.3
        self.encoder_r_zero = -0.53
        # self.encoder_l.reset()
        # self.encoder_r.reset()
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

    def encoder_reset(self):
        self.encoder_l.reset()
        self.encoder_r.reset()


class ClimberDefaultCommand(Command):
    def __init__(self, climber: Climber, controller: DriverController, op_controller: CommanderController):
        self.climber = climber
        self.controller = controller
        self.op_controller = op_controller
        self.addRequirements(climber)

    def execute(self):
        power = self.controller.get_climber_trigger()
        if power < 0:
            power *= .50
        if self.op_controller.get_climber_up():
            power = 0.4
        elif self.op_controller.get_climber_down():
            power = -1
        # if self.climber.encoder_l.get() > self.climber.encoder_l_zero or self.climber.encoder_r.get() < self.climber.encoder_r_zero:
        #     self.climber.set_speed(0)
        # else:
        self.climber.set_speed(power)

