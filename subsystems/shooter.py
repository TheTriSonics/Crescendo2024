from rev import CANSparkMax, CANSparkLowLevel
from wpilib import DutyCycleEncoder, SmartDashboard
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

    def __init__(self):
        super().__init__()
        ### Shooter Launch Motors ###
        # Initialize the target speed
        self.target_speed = 0
        self.target_tilt = 0
        # Initialize the motor controllers
        self.shooter_motor_left = TalonFX(RMM.shooter_motor_left)
        self.shooter_motor_right = TalonFX(RMM.shooter_motor_right)
        
        self.shooter_motor_left_configurator = self.shooter_motor_left.configurator
        self.shooter_motor_left_config = TalonFXConfiguration()

        self.shooter_motor_right_configurator = self.shooter_motor_right.configurator
        self.shooter_motor_right_config = TalonFXConfiguration()

        # Shooter PID gains
        left_slot0_configs = self.shooter_motor_left_config.slot0
        
        self.shooter_motor_left_configurator.apply(self.shooter_motor_left_config)
        self.shooter_motor_right_configurator.apply(self.shooter_motor_right_config)

        self.shooter_motor_right.set_control(Follower(RMM.shooter_motor_left, True))

        ### Shooter Feed Motor ###
        self.feed_motor = TalonFX(RMM.shooter_motor_feed)

        ### Shooter Tilt Motors ###
        self.tilt_motor_left = CANSparkMax(RMM.shooter_motor_tilt_left,
                                       CANSparkLowLevel.MotorType.kBrushless)
        self.tilt_motor_right = CANSparkMax(RMM.shooter_motor_tilt_right,
                                        CANSparkLowLevel.MotorType.kBrushless)
        
        self.tilt_position = 0 # TODO: Get the actual position of the tilt
        
        self.tilt_motor_right.follow(self.tilt_motor_left.get, invertOutput=False)

        self.tilt_pid_controller = self.tilt_motor_left.getPIDController()
        self.tilt_encoder = DutyCycleEncoder(RSM.shooter_tilt_encoder)

        self.tilt_pid_controller.setP(0.1)
        self.tilt_pid_controller.setI(0.0)
        self.tilt_pid_controller.setD(0.0)
        self.tilt_pid_controller.setIZone(0)
        self.tilt_pid_controller.setFF(0.0)
        self.tilt_pid_controller.setOutputRange(-1, 1)
        self.tilt_pid_controller.setFeedbackDevice(self.tilt_encoder)


    def set_tilt(self, tilt):
        self.tilt_pid_controller.setReference(tilt, CANSparkMax.ControlType.kPosition)        

    def set_velocity(self, velocity):
        self.target_speed = velocity
        self.shooter_motor_left.set_control(VelocityDutyCycle(velocity))

    def is_up_to_speed(self):
        return self.shooter_motor_left.get_velocity() >= self.target_speed

    def feed_note(self):
        self.feed_motor.set_control(DutyCycleOut(0.5))

    def feed_off(self):
        self.feed_motor.set_control(DutyCycleOut(0.0))

    def halt(self):
        # Stop the shooter motor
        self.shooter_motor_left.set_control(DutyCycleOut(0), override_brake_dur_neutral=True)

    def periodic(self) -> None:
        pn = SmartDashboard.putNumber
        pn("shooter/target_speed", self.target_speed)
        pn("shooter/target_tilt", self.target_tilt)
        # TODO: Add the feed motor's output to the dashboard
        pass
