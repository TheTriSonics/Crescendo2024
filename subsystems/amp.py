from wpilib import DigitalInput
from commands2 import Subsystem, Command
from rev import CANSparkMax
from phoenix6.hardware import TalonFX
from phoenix6.controls import DynamicMotionMagicVoltage
from phoenix6.configs import TalonFXConfiguration

from constants import RobotMotorMap as RMM, RobotSensorMap as RSM
from controllers.commander import CommanderController
from subsystems.photoeyes import Photoeyes


class Amp(Subsystem):
    class Direction():
        SHOOTER = 1
        AMP = -1

    class Height():
        HOME = 0.075
        AMP = 17.327
        TRAP = 20.327
        LIMIT = 30.35

    dir_up = 1
    dir_down = -1

    def __init__(self, controller: CommanderController, photoeyes: Photoeyes):
        super().__init__()

        self.controller = controller
        self.photoeyes = photoeyes

        self.limit_switch = DigitalInput(RSM.amp_lift_bottom_limit_switch)

        self.feed_motor = CANSparkMax(RMM.amp_feed_motor,
                                      CANSparkMax.MotorType.kBrushed)
        self.lift_motor = TalonFX(RMM.amp_lift_motor, "canivore")
        # self.lift_motor = TalonFX(RMM.amp_lift_motor)

        self.feed_motor.setInverted(True)

        self.lift_configurator = self.lift_motor.configurator
        self.lift_configs = TalonFXConfiguration()

        # Lift PID gains TODO: tunes these values, they were copied from the example
        slot0_configs = self.lift_configs.slot0
        slot0_configs.k_s = 0.25  # Add 0.25 V output to overcome static friction
        slot0_configs.k_v = 0.12  # A velocity target of 1 rps results in 0.12 V output
        slot0_configs.k_a = 0.01  # An acceleration of 1 rps/s requires 0.01 V output
        slot0_configs.k_p = 4.8  # A position error of 2.5 rotations results in 12 V output
        slot0_configs.k_i = 0  # no output for integrated error
        slot0_configs.k_d = 0.1  # A velocity error of 1 rps results in 0.1 V output

        motion_magic_configs = self.lift_configs.motion_magic
        motion_magic_configs.motion_magic_acceleration = 400
        motion_magic_configs.motion_magic_jerk = 4000

        self.lift_configurator.apply(self.lift_configs)

        self.height = self.Height.HOME

    def set_height(self, height):
        self.height = height
        pass

    def go_home(self):
        self.height = self.Height.HOME

    def get_height(self):
        return self.height

    def feed(self):
        self.feed_motor.set(1.0)

    def reverse(self):
        self.feed_motor.set(-1.0)

    def halt(self):
        self.feed_motor.set(0.0)

    def periodic(self) -> None:
        if self.height >= self.Height.LIMIT:
            self.height = self.Height.LIMIT
        self.request = DynamicMotionMagicVoltage(
            0, 80, 400, 4000, override_brake_dur_neutral=True,
            limit_reverse_motion=True
        ).with_limit_reverse_motion(not self.limit_switch.get())
        self.lift_motor.set_control(self.request.with_position(self.height))
        
