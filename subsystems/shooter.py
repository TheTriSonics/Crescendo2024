from rev import CANSensor, CANSparkMax, CANSparkLowLevel, MotorFeedbackSensor, SparkAbsoluteEncoder
from wpilib import DutyCycleEncoder, Joystick, SmartDashboard
from commands2 import Subsystem, Command
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import Follower, DutyCycleOut, VelocityDutyCycle
from wpimath.controller import PIDController

from constants import RobotMotorMap as RMM, RobotSensorMap as RSM


class Shooter(Subsystem):
    """
    """
    # May or may not be needed; if used for the shooter it could later be
    # replaced by the TalonFX's internal controller once sane values are known
    shooter_pid: PIDController

    # The SparkMax doesn't do any internal PID so for that it's software PID
    # or nothing.
    rotate_pid: PIDController

    class Tilt():
        HOME = 0.3
        SUB = 0.5
        SAFE = 0.5
        MAX = 0.986

    def __init__(self):
        super().__init__()
        defcmd = ShooterDefaultCommand(self)
        self.setDefaultCommand(defcmd)

        ### Shooter Launch Motors ###
        # Initialize the target speed
        self.target_speed = 0
        self.target_tilt = 0

        # Initialize the motor controllers
        self.shooter_motor_left = TalonFX(RMM.shooter_motor_left, "canivore")
        self.shooter_motor_right = TalonFX(RMM.shooter_motor_right, "canivore")
        
        self.shooter_motor_left_configurator = self.shooter_motor_left.configurator
        self.shooter_motor_left_config = TalonFXConfiguration()

        self.shooter_motor_right_configurator = self.shooter_motor_right.configurator
        self.shooter_motor_right_config = TalonFXConfiguration()

        # Shooter PID gains
        left_slot0_configs = self.shooter_motor_left_config.slot0
        
        self.shooter_motor_left_configurator.apply(self.shooter_motor_left_config)
        self.shooter_motor_right_configurator.apply(self.shooter_motor_right_config)

        # self.shooter_motor_right.set_control(Follower(RMM.shooter_motor_left, True))

        ### Shooter Feed Motor ###
        self.feed_motor = TalonFX(RMM.shooter_motor_feed, "canivore")

        ### Shooter Tilt Motors ###
        self.tilt_motor_left = CANSparkMax(RMM.shooter_motor_tilt_left,
                                       CANSparkLowLevel.MotorType.kBrushless)
        self.tilt_motor_right = CANSparkMax(RMM.shooter_motor_tilt_right,
                                        CANSparkLowLevel.MotorType.kBrushless)
        
        self.tilt_position = 0 # TODO: Get the actual position of the tilt
        
        self.tilt_motor_right.follow(self.tilt_motor_left)

        self.tilt_pid_controller = self.tilt_motor_left.getPIDController()
        self.tilt_encoder = self.tilt_motor_left.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
        self.tilt_encoder.setAverageDepth(32)

        self.tilt_pid_controller.setP(0.1)
        self.tilt_pid_controller.setI(0.0)
        self.tilt_pid_controller.setD(0.0)
        self.tilt_pid_controller.setIZone(0)
        self.tilt_pid_controller.setFF(0.0)
        self.tilt_pid_controller.setOutputRange(-1, 1)
        self.tilt_pid_controller.setFeedbackDevice(self.tilt_encoder)
        
    def set_auto_target_shot(self):
        pass

    def set_protected_shot(self):
        pass

    def set_speaker_shot(self):
        pass

    def set_tilt(self, tilt):
        self.target_tilt = tilt

    def set_velocity(self, velocity):
        self.target_speed = velocity
        self.shooter_motor_left.set_control(VelocityDutyCycle(velocity))

    def is_up_to_speed(self):
        return self.shooter_motor_left.get_velocity() >= self.target_speed

    def feed_note(self):
        self.feed_motor.set_control(DutyCycleOut(0.25))

    def feed_reverse(self):
        self.feed_motor.set_control(DutyCycleOut(-0.5))

    def feed_off(self):
        self.feed_motor.set_control(DutyCycleOut(0.0))

    def halt(self):
        # Stop the shooter motor
        self.shooter_motor_left.set_control(DutyCycleOut(0), override_brake_dur_neutral=False)

    def periodic(self) -> None:
        pn = SmartDashboard.putNumber
        pn("shooter/target_speed", self.target_speed)
        pn("shooter/target_tilt", self.target_tilt)
        pn("Shooter Feed Speed", self.feed_motor.get_duty_cycle().value_as_double)
        pn("Shooter Tilt Encoder", self.tilt_encoder.getPosition())
        # self.tilt_pid_controller.setReference(self.target_tilt, CANSparkMax.ControlType.kPosition)

class ShooterDefaultCommand(Command):

    def __init__(self, shooter: Shooter):
        self.addRequirements(shooter)
        self.shooter = shooter
        self.controller = Joystick(5)

    def execute(self):
        power = 0

        if abs(self.controller.getRawAxis(1)) >= 0.04:
            power = self.controller.getRawAxis(1) * 0.1
        else:
            power = 0

        self.shooter.tilt_motor_left.set(power)
        self.shooter.tilt_motor_right.set(power)

