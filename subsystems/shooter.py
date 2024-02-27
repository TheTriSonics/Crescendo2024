from rev import CANSparkMax, CANSparkLowLevel
from wpilib import DutyCycle, SmartDashboard
from commands2 import Subsystem, Command
from phoenix6.hardware import TalonFX
from phoenix6.signals import ControlModeValue
from wpimath.controller import PIDController

from constants import RobotMotorMap as RMM


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
        # Initialize the target speed
        self.target_speed = 0
        self.target_elevation = 0
        # Initialize the motor controllers
        self.shooter_motor_left = TalonFX(RMM.shooter_motor_left)
        self.shooter_motor_right = TalonFX(RMM.shooter_motor_right)
        # TODO: Set right motor as follower of left

        self.feed_motor = TalonFX(RMM.shooter_motor_feed)

        self.tilt_motor_left = CANSparkMax(RMM.shooter_motor_tilt_left,
                                       CANSparkLowLevel.MotorType.kBrushless)
        self.tilt_motor_right = CANSparkMax(RMM.shooter_motor_tilt_right,
                                        CANSparkLowLevel.MotorType.kBrushless)

    def set_elevation(self, value):
        # Set the speed of the elevation motor
        # self.elevation_motor.set(value)
        pass

    def set_speed(self, value):
        # Set the target speed of the shooter motor
        # self.target_speed = value
        pass

    def is_up_to_speed(self):
        # Check if the shooter motor is up to speed
        # return self.encoder.getRate() >= self.target_speed
        pass

    def feed_note(self):
        self.feed_motor.set_control(DutyCycle(0.5))

    def feed_off(self):
        self.feed_motor.set_control(DutyCycle(0.0))

    def halt(self):
        # Stop the shooter motor
        self.shooter_motor_left.set_control(DutyCycle(0), override_brake_dur_neutral=True)
        pass

    def periodic(self) -> None:
        pn = SmartDashboard.putNumber
        pn("shooter/target_speed", self.target_speed)
        pn("shooter/target_elevation", self.target_elevation)
        # TODO: Add the feed motor's output to the dashboard
        pass
