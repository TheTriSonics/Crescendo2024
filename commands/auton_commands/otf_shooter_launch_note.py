from wpilib import SmartDashboard, Timer
from commands2 import Command
from controllers.driver import DriverController
from subsystems.amp import Amp
from subsystems.drivetrain import Drivetrain
from subsystems.intake import Intake

from subsystems.shooter import Shooter
from phoenix6.controls import DutyCycleOut

class OTFShooterLaunchNote(Command):
    def __init__(self, shooter: Shooter, drivetrain: Drivetrain, driver: DriverController,intake: Intake, amp: Amp,
                  driverneeded = False, rpm=80, do_rotation = False) -> None:
        super().__init__()
        self.shooter = shooter
        self.drivetrain = drivetrain
        self.driver = driver
        self.amp = amp
        self.intake = intake
        self.driverneeded = driverneeded
        self.timer = Timer()
        self.shot_timer = Timer()
        self.rpm = rpm
        self.do_rotation = do_rotation
        self.addRequirements(shooter)
        # self.addRequirements(drivetrain)

    def setTilt(self, distance: float):
        self.tilt = 0.7622085 + 0.1940302/(2 ** (distance / 0.9421715))

    def initialize(self) -> None:
        print("OTF Shooter launch note started")
        self.shooter.set_velocity(self.rpm)
        self.shooter.spin_up()
        self.shot_fired = False
        self.timer.restart()

    def execute(self) -> None:
        # TODO:
        # Add in check for vision targets here?
        up_to_speed = self.shooter.is_up_to_speed()
        aimed = self.shooter.is_tilt_aimed()
        driverRB = self.driver.get_speaker_lockon()
        self.shooter.go_otf_tilt()
        if self.driverneeded and not driverRB:
            self.timer.restart()
        if self.do_rotation:
            drive_aimed = self.drivetrain.is_speaker_aimed()
        else:
            drive_aimed = True
        # print(f"OTF uptospeed = {up_to_speed}, aimed = {aimed}, timerelapsed = {self.timer.hasElapsed(0.5)}")
        if not self.shot_fired and up_to_speed and aimed and drive_aimed and self.timer.hasElapsed(0.5):
            self.intake.feed()
            self.amp.reverse()
            self.shooter.feed_note()
            self.shot_fired = True
            self.shot_timer.restart()

    def end(self, interrupted: bool) -> None:
        self.shooter.feed_off()
        self.shooter.prepare_to_load()
        self.shooter.halt()
        self.intake.halt()
        self.amp.halt()
        self.shooter.shooter_motor_left.set_control(DutyCycleOut(0.0))
        self.shooter.shooter_motor_right.set_control(DutyCycleOut(0.0))
        self.drivetrain.defcmd.speaker_tracking_off()

    def isFinished(self) -> bool:
        return self.shot_fired and self.shot_timer.hasElapsed(0.5)
