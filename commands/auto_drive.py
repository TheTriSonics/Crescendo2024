import math
from commands2 import Command
from wpilib import SmartDashboard, Timer
from controllers.driver import DriverController

from subsystems.drivetrain import Drivetrain
from subsystems.gyro import Gyro
from subsystems.intake import Intake
from subsystems.note_tracker import NoteTracker
from wpimath.controller import PIDController
from constants import RobotPIDConstants as PIDC
from wpimath.filter import SlewRateLimiter, LinearFilter

import subsystems.shooter as shooter
from subsystems.shooter import Shooter

kMaxSpeed = 4.5  # m/s
kMaxAngularSpeed = math.pi * 5

swerve_offset = 27 / 100  # cm converted to meters


pn = SmartDashboard.putNumber
gn = SmartDashboard.getNumber
pb = SmartDashboard.putBoolean
gb = SmartDashboard.getBoolean

curr_note_intake_speed = 2
default_note_intake_speed = 2


class AutoDriveCommand(Command):
    """
    Command for the drivetrain to be used in Autonomous.
    """
    def __init__(self, drivetrain: Drivetrain, photon: NoteTracker, gyro: Gyro, intake: Intake, shooter: Shooter,
                 note_lockon = False, speaker_lockon = False, timeout = 10) -> None:
        super().__init__()
        self.timer = Timer()
        self.drivetrain = drivetrain
        self.photon = photon
        self.gyro = gyro
        self.intake = intake
        self.shooter = shooter
        self.note_lockon = note_lockon
        self.speaker_lockon = speaker_lockon
        self.timeout = timeout
        # self.note_pid = PIDController(*PIDC.note_tracking_pid)
        self.note_translate_pid = PIDController(*PIDC.note_translate_pid)
        self.speaker_pid = PIDController(*PIDC.speaker_tracking_pid)
        self.straight_drive_pid = PIDController(*PIDC.straight_drive_pid)
        self.straight_drive_pid.setTolerance(2.0)
        self.note_yaw_filter = LinearFilter.highPass(0.1, 0.02)
        self.idle_counter = 0
        self.desired_heading = 0
        self.addRequirements(drivetrain)
        self.stop = False
        self.drivetrain.set_speaker_tracking(False)
        self.drivetrain.set_speaker_visible(False)
        self.drivetrain.set_speaker_aimed(False)
        self.xSpeed = 0
        self.ySpeed = 0
        self.rot = 0


    def initialize(self):
        self.swapped = False
        self.timer.restart()

    def note_tracking_on(self):
        self.note_lockon = True

    def note_tracking_off(self):
        self.note_lockon = False

    def speaker_tracking_on(self):
        print("drivetrain.auto_drive.speaker_tracking_on function called")
        self.speaker_lockon = True

    def speaker_tracking_off(self):
        self.speaker_lockon = False

    def _curr_heading(self) -> float:
        return self.drivetrain.get_heading_rotation_2d().degrees()

    def flipHeading(self):
        self.desired_heading += 180
        if self.desired_heading > 360:
            self.desired_heading -= 360

    def swapDirection(self):
        self.swapped = not self.swapped

    def lock_heading(self):
        self.desired_heading = self._curr_heading()

    def is_note_tracking(self):
        return self.drivetrain.note_tracking

    def is_note_visible(self):
        return self.drivetrain.note_visible

    def is_speaker_tracking(self):
        return self.drivetrain.speaker_tracking

    def is_speaker_visible(self):
        return self.drivetrain.speaker_visible

    def is_speaker_aimed(self):
        return self.drivetrain.speaker_aimed


    def is_amp_tracking(self):
        return self.drivetrain.amp_tracking

    def is_amp_visible(self):
        return self.drivetrain.amp_visible
    
    def stop_auto_drive(self):
        self.stop = True

    def execute(self) -> None:
        # Heading is in degrees here
        if self.drivetrain.get_lock_heading() is None:
            self.drivetrain.set_lock_heading(self._curr_heading())
        curr = self.drivetrain.get_heading_rotation_2d().degrees()
        self.xSpeed *= kMaxSpeed
        self.ySpeed *= kMaxSpeed
        self.rot *= kMaxAngularSpeed
        if self.swapped:
            self.xSpeed = -self.xSpeed
            self.ySpeed = -self.ySpeed

        # When in lockon mode, the robot will rotate to face the node
        # that PhotonVision is detecting
        robot_centric_force = False

        self.drivetrain.note_tracking = False
        self.drivetrain.note_visible = False
        if self.note_lockon:
            self.xSpeed = 0.15
            self.rot = 0
            self.note_tracking = True
            robot_centric_force = True
            pn = SmartDashboard.putNumber
            note_yaw = self.photon.getYawOffset()
            if note_yaw is not None:
                self.note_visible = True
                self.note_yaw_filtered = self.note_yaw_filter.calculate(note_yaw)
                pn('drivetrain/note_tracker/note_yaw', note_yaw)
                pn('drivetrain/note_tracker/note_yaw_filtered', self.note_yaw_filtered)
            # pitch = self.photon.getPitchOffset()
            if note_yaw is not None:
                # Setpoint was 0 but moved to 2 to try and get the
                # robot from going left of the note
                self.P, self.I, self.D = 0.05, 0, 0
                # self.P, self.I, self.D = self.drivetrain.getPID()
                self.note_pid = PIDController(self.P, self.I, self.D)
                self.rot = self.note_pid.calculate(note_yaw, 0)
                # xSpeed = -0.003 * note_yaw ** 2 + curr_note_intake_speed
                self.xSpeed = 1.3

        self.slow_mode = False
        if self.slow_mode:
            self.slow_mode = True
            slow_mode_factor = 1/2
            self.xSpeed *= slow_mode_factor
            self.ySpeed *= slow_mode_factor
            self.rot *= slow_mode_factor

        if self.speaker_lockon:
            # TODO: Possible! Check with Nathan -- slow down the drivetrain by setting
            # master_throttle to something like 0.8 or 0.6...
            self.drivetrain.speaker_tracking = True
            fid = 4 if self.drivetrain.is_red_alliance() else 7
            speaker_heading = self.drivetrain.get_fid_heading(fid)
            speaker_distance = self.drivetrain.get_fid_distance(fid)
            # print(f"speaker distance is {speaker_distance}")
            if speaker_heading is not None:
                self.shooter.calc_otf_tilt(speaker_distance)
                # print(f"OTF sees speaker and distance is {speaker_distance}")
                self.drivetrain.speaker_visible = True
                if abs(speaker_heading) < 3.0:
                    self.rot = 0
                    self.drivetrain.speaker_aimed = True
                else:
                    self.rot = self.speaker_pid.calculate(speaker_heading, 0)
        
        pb("Speaker tracking", self.drivetrain.speaker_tracking)
        pb("Speaker visible", self.drivetrain.speaker_visible)
        pb("Speaker aimed", self.drivetrain.speaker_aimed)

        self.drivetrain.drive(self.xSpeed, self.ySpeed, self.rot,
                              robot_centric_force=robot_centric_force)

    def end(self, interrupted: bool):
        if interrupted:
            print('auto_drive interrupted')
        else:
            print('auto_drive done')
        self.drivetrain.drive(0, 0, 0)
        self.drivetrain.set_speaker_tracking(False)
        self.drivetrain.set_speaker_visible(False)
        self.drivetrain.set_speaker_aimed(False)
        pass

    def isFinished(self):
        if self.timer.hasElapsed(self.timeout):
            return True
        if self.stop:
            return True
