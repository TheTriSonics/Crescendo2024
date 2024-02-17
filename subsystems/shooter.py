from rev import CANSparkMax, CANSparkLowLevel
from commands2 import Subsystem, Command
from phoenix6.hardware import TalonFX
from wpilib import DutyCycle
from wpimath.controller import PIDController

from constants import RobotMap


class Shooter(Subsystem):
    shooter_left: TalonFX
    shooter_right: TalonFX
    feed: TalonFX
    rotate_left: CANSparkMax
    rotate_right: CANSparkMax

    # May or may not be needed; if used for the shooter it could later be
    # replaced by the TalonFX's internall controller once sane values are known
    shooter_pid: PIDController

    # The SparkMax doesn't do any internal PID so for that it's software PID
    # or nothing.
    rotate_pid: PIDController

    def __init__(self):
        super().__init__()

        # Initialize the motor controllers
        # Replace with the type of motor controller you're using
        self.shooter_left = TalonFX(RobotMap.shooter_left)
        self.shooter_right = TalonFX(RobotMap.shooter_right)

        self.feed = TalonFX(RobotMap.shooter_feed)

        self.rotate_left = CANSparkMax(RobotMap.shooter_rotate_left,
                                       CANSparkLowLevel.MotorType.kBrushless)
        self.rotate_right = CANSparkMax(RobotMap.shooter_rotate_right,
                                        CANSparkLowLevel.MotorType.kBrushless)

        # Initialize the target speed
        self.target_speed = 0

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
        self.feed.set_control(DutyCycle(0.5))

    def feed_off(self):
        self.feed.set_control(DutyCycle(0.0))

    def reverse(self):
        # Reverse the direction of the shooter motor
        # self.shooter_motor.set(-self.target_speed)
        pass

    def halt(self):
        # Stop the shooter motor
        # self.shooter_motor.set(0)
        pass
