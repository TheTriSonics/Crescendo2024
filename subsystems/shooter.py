from rev import CANSensor, CANSparkMax, CANSparkLowLevel, MotorFeedbackSensor, SparkAbsoluteEncoder
from wpilib import DutyCycleEncoder, Joystick, SmartDashboard, Timer
from commands2 import Subsystem, Command
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import Follower, DutyCycleOut, VelocityDutyCycle
from wpimath.controller import PIDController

from constants import RobotMotorMap as RMM, RobotSensorMap as RSM

tilt_bottom_limit = 0.76
tilt_load_limit = 0.77
tilt_upper_limit = 0.856
max_tilt_diff = (tilt_upper_limit - tilt_bottom_limit) / 50

tilt_sub = 0.84
tilt_safe = 0.77


class Shooter(Subsystem):
    # The SparkMax doesn't do any internal PID so for that it's software PID
    # or nothing.
    rotate_pid: PIDController
    dir_up = 1
    dir_down = -1

    def __init__(self):
        super().__init__()
        self.alive_timer = Timer()
        self.alive_timer.start()
        # defcmd = ShooterDefaultCommand(self)
        # self.setDefaultCommand(defcmd)

        ### Shooter Launch Motors ###
        # Initialize the target speed
        # Valid speeds seem to be 0-83 RPM.
        # We need to find a stable speed to shoot at at various distances
        self.speed_target = 0
        self.waiting_speed_target = 0
        self.tilt_target = tilt_load_limit

        # Initialize the motor controllers
        self.shooter_motor_left = TalonFX(RMM.shooter_motor_left, "canivore")
        self.shooter_motor_right = TalonFX(RMM.shooter_motor_right, "canivore")

        # p, i, d = 0.1, 0.0, 0.0
        # self.left_shooter_pid = PIDController(p, i, d)
        # self.right_shooter_pid = PIDController(p, i, d)

        self.shooter_motor_left_configurator = self.shooter_motor_left.configurator
        self.shooter_motor_left_config = TalonFXConfiguration()

        self.shooter_motor_right_configurator = self.shooter_motor_right.configurator
        self.shooter_motor_right_config = TalonFXConfiguration()

        # Shooter PID gains
        left_slot0_configs = self.shooter_motor_left_config.slot0
        right_slot0_configs = self.shooter_motor_right_config.slot0

        left_slot0_configs.k_p = 0.025
        left_slot0_configs.k_s = 0.0025
        left_slot0_configs.k_v = 0.005

        right_slot0_configs.k_p = 0.025
        right_slot0_configs.k_s = 0.0025
        right_slot0_configs.k_v = 0.005

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

        self.tilt_motor_right.follow(self.tilt_motor_left)

        ### Shooter Tilt Encoder ###
        self.tilt_encoder = DutyCycleEncoder(RSM.shooter_tilt_encoder)
        self.tilt_encoder.setDistancePerRotation(360.0)

        ### Shooter Tilt Motor Encoders ###
        self.tilt_motor_left_encoder = self.tilt_motor_left.getEncoder()
        self.tilt_motor_right_encoder = self.tilt_motor_right.getEncoder()

        ### Override the motor's encoder position to the tilt absolute encoder's position so we can use the built in PID controller
        self.tilt_motor_left_encoder.setPosition(self.tilt_encoder.getAbsolutePosition())
        self.tilt_motor_right_encoder.setPosition(self.tilt_encoder.getAbsolutePosition())

        ### Shooter Tilt Left Motor PID Controller ###

        self.tilt_pid = PIDController(30.0, 0, 0)

        """
        For now we will use the sofwree PID controller, but we will
        switch to the SparkMAX PID controller later
        self.tilt_left_pid_controller = self.tilt_motor_left.getPIDController()

        self.tilt_left_pid_controller.setP(0.1)
        self.tilt_left_pid_controller.setI(0.0)
        self.tilt_left_pid_controller.setD(0.0)
        self.tilt_left_pid_controller.setIZone(0)
        self.tilt_left_pid_controller.setFF(0.0)
        self.tilt_left_pid_controller.setOutputRange(-1, 1)

        ### Shooter Tilt Right Motor PID Controller ###
        self.tilt_right_pid_controller = self.tilt_motor_right.getPIDController()

        self.tilt_right_pid_controller.setP(0.1)
        self.tilt_right_pid_controller.setI(0.0)
        self.tilt_right_pid_controller.setD(0.0)
        self.tilt_right_pid_controller.setIZone(0)
        self.tilt_right_pid_controller.setFF(0.0)
        self.tilt_right_pid_controller.setOutputRange(-1, 1)
        """
        self.tilt_motor_left.burnFlash()
        self.tilt_motor_right.burnFlash()

    def set_auto_target_shot(self):
        pass

    def set_protected_shot(self):
        pass

    def set_speaker_shot(self):
        pass

    def set_velocity(self, velocity):
        self.waiting_speed_target = velocity

    def is_up_to_speed(self):
        return abs(self.shooter_motor_left.get_velocity().value - self.speed_target) < 4

    def is_tilt_aimed(self):
        return abs(self.tilt_encoder.getAbsolutePosition()-self.tilt_target) < max_tilt_diff

    def feed_note(self):
        self.feed_motor.set_control(DutyCycleOut(0.25))

    def feed_reverse(self):
        self.feed_motor.set_control(DutyCycleOut(-0.25))

    def feed_off(self):
        self.feed_motor.set_control(DutyCycleOut(0.0, override_brake_dur_neutral=True))

    def tilt_up(self):
        self.tilt_motor_left.set(0.2)
        self.tilt_motor_right.set(0.2)

    def tilt_down(self):
        self.tilt_motor_left.set(-0.2)
        self.tilt_motor_right.set(-0.2)

    def halt(self):
        # Stop the shooter motor
        self.shooter_motor_left.set_control(DutyCycleOut(0, override_brake_dur_neutral=False))

    def _deg_to_shooter_tilt(self, deg):
        deg_min = 30
        deg_max = 80
        run = sm.tilt_upper_limit - sm.tilt_bottom_limit
        rise = deg_max - deg_min
        m = rise/run
        b = sm.tilt_bottom_limit
        if deg < deg_min:
            retval = sm.tilt_bottom_limit
        elif deg >= deg_min and deg <= deg_max:
            retval = m*deg + b
        elif deg > deg_max:
            retval = sm.tilt_top_limit
        return retval

    # TODO: Actual numbers need to be sorted out for this
    def _fidarea_to_shooter_rpm(self, fidarea):
        if fidarea < 100:
            self.speed_target = 40
        elif fidarea < 200:
            self.speed_target = 30
        elif fidarea < 300:
            self.speed_target = 20
        else:
            self.speed_target = 10

    def set_otf_elevation(self, deg):
        self.tilt_target = self._deg_to_shooter_tilt(deg)

    def set_otf_rpm(self, fidarea):
        self.speed_target = self._fidarea_to_shooter_rpm(fidarea)


    def safe_shot(self):
        self.tilt_target = tilt_safe
        self.waiting_speed_target = 75

    def sub_shot(self):
        self.tilt_target = tilt_sub
        self.waiting_speed_target = 68

    def spin_up(self):
        self.speed_target = self.waiting_speed_target

    def spin_down(self):
        self.speed_target = 0

    def prepare_to_load(self):
        self.tilt_target = tilt_load_limit


    def periodic(self) -> None:
        pn = SmartDashboard.putNumber
        left_velocity = self.shooter_motor_left.get_velocity().value
        right_velocity = self.shooter_motor_right.get_velocity().value
        pn("shooter/target_speed", self.speed_target)
        pn("shooter/left_actual_speed", left_velocity)
        pn("shooter/right_actual_speed", right_velocity)
        pn("Shooter Feed Speed", self.tilt_motor_left.get())
        pn("Shooter Tilt Encoder", self.tilt_encoder.getAbsolutePosition())
        pn("Shooter tilt motor Encoder", self.tilt_motor_left_encoder.getPosition())

        # Control the shooter flywheels with a software PID Controller
        # self.left_shooter_pid.setSetpoint(self.speed_target)
        # self.right_shooter_pid.setSetpoint(self.speed_target)
        # left_power = self.left_shooter_pid.calculate(left_velocity)
        # right_power = self.right_shooter_pid.calculate(right_velocity)
        self.shooter_motor_left.set_control(VelocityDutyCycle(self.speed_target))
        self.shooter_motor_right.set_control(VelocityDutyCycle(-self.speed_target))
        # Do NOTHING for 3 seconds after the robot starts up
        if not self.alive_timer.hasElapsed(2.0):
            return

        # If some other part of the software sets the tilt target to a value
        # that would break something we rip it back into the limit here.
        if self.tilt_target < tilt_bottom_limit:
            self.tilt_target = tilt_bottom_limit
        if self.tilt_target > tilt_upper_limit:
            self.tilt_target = tilt_upper_limit
        curr_pos = self.tilt_encoder.getAbsolutePosition()
        # Sanity check on the encoder values too!
        if curr_pos < tilt_bottom_limit:
            curr_pos = tilt_bottom_limit
        if curr_pos > tilt_upper_limit:
            curr_pos = tilt_upper_limit
        # Keep, want to use SparkMAX PID controller later
        # self.tilt_pid_controller.setReference(self.target_tilt, CANSparkMax.ControlType.kPosition)
        tilt_power = self.tilt_pid.calculate(curr_pos, self.tilt_target)
        self.tilt_motor_left.set(tilt_power)
        self.tilt_motor_right.set(tilt_power)
        pn("shooter/tilt_power", tilt_power)

