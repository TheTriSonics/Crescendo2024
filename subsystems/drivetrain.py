
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from subsystems.speaker_tracker import SpeakerTracker
import subsystems.swervemodule as swervemodule
import subsystems.gyro as gyro
from commands2 import CommandScheduler, Subsystem, Command
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import (
    SwerveModuleState, SwerveDrive4Kinematics,
    SwerveModulePosition, ChassisSpeeds
)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import (
    HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
)

from constants import RobotMotorMap as RMM
from subsystems.note_tracker import NoteTracker
from subsystems.intake import Intake


kMaxSpeed = 4.5  # m/s
kMaxAngularSpeed = math.pi * 5

swerve_offset = 27 / 100  # cm converted to meters


pn = SmartDashboard.putNumber
gn = SmartDashboard.getNumber
pb = SmartDashboard.putBoolean
gb = SmartDashboard.getBoolean

sdbase = 'fakesensors/drivetrain'

curr_note_intake_speed = 2
default_note_intake_speed = 2

import math
from commands2 import Command
from wpilib import PS4Controller, SmartDashboard
from controllers.driver import DriverController

from subsystems.gyro import Gyro
from subsystems.intake import Intake
from subsystems.note_tracker import NoteTracker

from wpimath.filter import SlewRateLimiter, LinearFilter
from wpimath.controller import PIDController
from constants import RobotPIDConstants as PIDC

sdbase = 'fakesensors/drivetrain'

pn = SmartDashboard.putNumber
gn = SmartDashboard.getNumber
pb = SmartDashboard.putBoolean
gb = SmartDashboard.getBoolean

kMaxSpeed = 4.5  # m/s
kMaxAngularSpeed = math.pi * 5

curr_note_intake_speed = 2
default_note_intake_speed = 2


class Drivetrain(Subsystem):
    """
    Represents a swerve drive style drivetrain.
    """
    def __init__(self, gyro: gyro.Gyro, driver_controller,
                 photon: NoteTracker, intake: Intake,
                 speaker_tracker: SpeakerTracker) -> None:
        super().__init__()
        # TODO: Set these to the right numbers in centimeters
        self.frontLeftLocation = Translation2d(swerve_offset, swerve_offset)
        self.frontRightLocation = Translation2d(swerve_offset, -swerve_offset)
        self.backLeftLocation = Translation2d(-swerve_offset, swerve_offset)
        self.backRightLocation = Translation2d(-swerve_offset, -swerve_offset)
        self.photon = photon
        self.intake = intake

        pb(f'{sdbase}/note_tracking', False)
        pb(f'{sdbase}/note_visible', False)

        pb(f'{sdbase}/speaker_tracking', False)
        pb(f'{sdbase}/speaker_visible', False)
        pb(f'{sdbase}/speaker_aimed', False)

        pb(f'{sdbase}/amp_tracking', False)
        pb(f'{sdbase}/amp_visible', False)

        self.note_tracking = False
        self.note_visible = False
        self.speaker_tracking = False
        self.speaker_visible = False
        self.speaker_aimed = False
        self.amp_tracking = False
        self.amp_visible = False
        self.lockable = False
        self.locked = False
        self.vision_stable = True


        # Half the motors need to be inverted to run the right direction and
        # half are in brake mode to slow the robot down faster but also not
        # make it come to a complete stop too quickly.
        self.frontLeft = swervemodule.SwerveModule(
            RMM.front_left_drive,
            RMM.front_left_turn,
            RMM.front_left_turn_encoder,
            inverted=True,
            brake=True,
            name='Front left')
        self.frontRight = swervemodule.SwerveModule(
            RMM.front_right_drive,
            RMM.front_right_turn,
            RMM.front_right_turn_encoder,
            inverted=False,
            brake=True,  # Set to false becaue this module isn't working at GVSU
            name='Front right')
        self.backLeft = swervemodule.SwerveModule(
            RMM.back_left_drive,
            RMM.back_left_turn,
            RMM.back_left_turn_encoder,
            inverted=True,
            brake=True,
            name='Back left')
        self.backRight = swervemodule.SwerveModule(
            RMM.back_right_drive,
            RMM.back_right_turn,
            RMM.back_right_turn_encoder,
            inverted=False,
            brake=True,
            name='Back right')

        self.modules = [
            self.frontLeft,
            self.frontRight,
            self.backLeft,
            self.backRight,
        ]

        self.ntinst = NetworkTableInstance.getDefault().getTable('limelight')
        self.ll_json = self.ntinst.getStringTopic("json")
        self.ll_json_entry = self.ll_json.getEntry('[]')

        self.fieldRelative = False

        self.gyro = gyro
        self.controller = driver_controller

        self.kinematics = SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        woc = 0.05
        self.odometry = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_heading_rotation_2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            Pose2d(),
            (woc, woc, woc),  # wheel std devs for Kalman filters
            (0.9, 0.9, 0.9),  # vision std devs for Kalman filters
        )

        self.resetOdometry()

        # Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            # Robot pose supplier
            self.getPose,
            # Method to reset odometry (will be called if your auto has a
            # starting pose)
            self.resetOdometry,
            # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.getSpeeds,
            # Method that will drive the robot given ROBOT RELATIVE
            # ChassisSpeeds
            self.driveRobotRelative,
            HolonomicPathFollowerConfig(
                PIDConstants(6.0, 0.0, 0.0),  # Translation PID constants
                PIDConstants(5, 0.0, 0.16),  # Rotation PID constants
                kMaxSpeed,  # Max module speed, in m/s.
                # Drive base radius in meters. Distance from robot center to
                # furthest module.
                0.350,
                # Default path replanning config. See the API for options
                ReplanningConfig()
            ),
            # Supplier to control path flipping based on alliance color
            self.shouldFlipPath,
            self  # Reference to this subsystem to set requirements
        )

        self.defcmd = DrivetrainDefaultCommand(self, self.controller, photon, gyro, intake)
        self.setDefaultCommand(self.defcmd)

    def set_note_intake_speed(self, x):
        curr_note_intake_speed = x

    def is_note_tracking(self):
        fake = gb(f'{sdbase}/note_tracking', False)
        return self.defcmd.is_note_tracking() or fake

    def is_note_visible(self):
        fake = gb(f'{sdbase}/note_visible', False)
        return self.defcmd.is_note_visible() or fake

    def is_speaker_tracking(self):
        fake = gb(f'{sdbase}/speaker_tracking', False)
        return self.defcmd.is_speaker_tracking() or fake

    def is_speaker_visible(self):
        fake = gb(f'{sdbase}/speaker_visible', False)
        return self.defcmd.is_speaker_visible() or fake

    def is_speaker_aimed(self):
        fake = gb(f'{sdbase}/speaker_aimed', False)
        return self.defcmd.is_speaker_aimed() or fake

    def is_amp_tracking(self):
        fake = gb(f'{sdbase}/amp_tracking', False)
        return self.defcmd.is_amp_tracking() or fake

    def is_amp_visible(self):
        fake = gb(f'{sdbase}/amp_visible', False)
        return self.defcmd.is_amp_visible() or fake

    def lock_heading(self):
        self.desired_heading = self.get_heading_rotation_2d().degrees()

    def llJson(self) -> str:
        return self.ll_json.getEntry("[]")

    def getSpeeds(self):
        cs = self.kinematics.toChassisSpeeds(
            [m.getState() for m in self.modules]
        )
        return cs

    def driveRobotRelative(self, speeds):
        self.fieldRelative = False
        states = self.kinematics.toSwerveModuleStates(speeds)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, kMaxSpeed)
        for m, s in zip(self.modules, states):
            m.setDesiredState(s)

    def shouldFlipPath(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def is_red_alliance(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def resetOdometry(self, pose: Pose2d = None):
        if pose is None:
            self.gyro.set_yaw(0)
            defaultPos = (
                SwerveModulePosition(0, Rotation2d(0)),
                SwerveModulePosition(0, Rotation2d(0)),
                SwerveModulePosition(0, Rotation2d(0)),
                SwerveModulePosition(0, Rotation2d(0)),
            )
            self.odometry.resetPosition(
                Rotation2d(), defaultPos, Pose2d()
            )
        else:
            self.gyro.set_yaw(pose.rotation().degrees())
            self.odometry.resetPosition(
                pose.rotation(),
                modulePositions=[m.getPosition() for m in self.modules],
                pose=pose,
            )

    def get_heading_rotation_2d(self) -> Rotation2d:
        from misc import is_sim
        if not is_sim():
            yaw = self.gyro.get_yaw()
            return Rotation2d(math.radians(yaw))
        else:
            return Rotation2d(0)

    def toggleFieldRelative(self):
        self.fieldRelative = not self.fieldRelative

    def flipHeading(self):
        self.defcmd.flipHeading()

    def swapDirection(self):
        self.defcmd.swapDirection()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        robot_centric_force: bool = False,
        periodSeconds: float = 0.02,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param periodSeconds: Time
        """

        # Force the robot to be field relative if we're tracking a note
        if self.fieldRelative and not robot_centric_force:
            flip = self.shouldFlipPath()
            if flip:
                xSpeed = -xSpeed
                ySpeed = -ySpeed
            cs = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.get_heading_rotation_2d(),
                )
        else:
            cs = ChassisSpeeds(xSpeed, ySpeed, rot)

        states = self.kinematics.toSwerveModuleStates(cs)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, kMaxSpeed)
        for m, s in zip(self.modules, states):
            m.setDesiredState(s)

    def lockWheels(self):
        for m in self.modules:
            m.lock()

    def setStates(self, fl: SwerveModuleState, fr: SwerveModuleState,
                  bl: SwerveModuleState, br: SwerveModuleState):
        for m, s in zip(self.modules, (fl, fr, bl, br)):
            m.setDesiredState(s)

    def getAngles(self) -> tuple[float, float, float, float]:
        return (m.getState().angle.radians() for m in self.modules)

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.get_heading_rotation_2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

    def getPose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()

    def periodic(self) -> None:
        self.updateOdometry()
        pb = SmartDashboard.putBoolean
        pn = SmartDashboard.putNumber
        pb("drivetrain/field_relative", self.fieldRelative)
        pn('drivetrain/odometry/pose_x', self.getPose().X())
        pn('drivetrain/odometry/pose_y', self.getPose().Y())
        pn('drivetrain/odometry/pose_rotation', self.getPose().rotation().degrees())




class DrivetrainDefaultCommand(Command):
    """
    Default command for the drivetrain.
    """
    def __init__(self, drivetrain: Drivetrain, controller: DriverController,
                 photon: NoteTracker, gyro: Gyro, intake: Intake) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.controller = controller
        self.photon = photon
        self.gyro = gyro
        self.intake = intake
        self.note_pid = PS4Controller(*PIDC.note_tracking_pid)
        self.note_translate_pid = PIDController(*PIDC.note_translate_pid)
        self.speaker_pid = PIDController(*PIDC.speaker_tracking_pid)
        self.straight_drive_pid = PIDController(*PIDC.straight_drive_pid)
        self.straight_drive_pid.setTolerance(2.0)
        # Slew rate limiters to make joystick inputs more gentle
        self.xslew = SlewRateLimiter(5.0)
        self.yslew = SlewRateLimiter(5.0)
        self.rotslew = SlewRateLimiter(2)
        self.note_yaw_filtered = LinearFilter.highPass(0.1, 0.02)
        self.idle_counter = 0
        self.desired_heading = None
        self.addRequirements(drivetrain)

    def initialize(self):
        # Heading is in degrees here
        if self.desired_heading is None:
            self.desired_heading = self._curr_heading()
        self.swapped = False
        self.slow_mode = False

    def note_tracking_on(self):
        self.note_lockon = True

    def note_tracking_off(self):
        self.note_lockon = False

    def speaker_tracking_on(self):
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
        fake = gb(f'{sdbase}/note_tracking', False)
        return self.drivetrain.note_tracking or fake

    def is_note_visible(self):
        fake = gb(f'{sdbase}/note_visible', False)
        return self.drivetrain.note_visible or fake

    def is_speaker_tracking(self):
        fake = gb(f'{sdbase}/speaker_tracking', False)
        return self.drivetrain.speaker_tracking or fake

    def is_speaker_visible(self):
        fake = gb(f'{sdbase}/speaker_visible', False)
        return self.drivetrain.speaker_visible or fake

    def is_speaker_aimed(self):
        fake = gb(f'{sdbase}/speaker_aimed', False)
        return self.drivetrain.speaker_aimed or fake

    def is_amp_tracking(self):
        fake = gb(f'{sdbase}/amp_tracking', False)
        return self.drivetrain.amp_tracking or fake

    def is_amp_visible(self):
        fake = gb(f'{sdbase}/amp_visible', False)
        return self.drivetrain.amp_visible or fake

    def _get_x_speed(self) -> float:
        master_throttle = self.controller.get_master_throttle()
        xraw = self.controller.get_drive_x()
        xSpeed = self.xslew.calculate(xraw)
        if xraw == 0:
            xSpeed /= 2
        xSpeed *= kMaxSpeed
        if self.swapped:
            xSpeed = -xSpeed
        return xSpeed*master_throttle

    def _get_y_speed(self) -> float:
        master_throttle = self.controller.get_master_throttle()
        yraw = self.controller.get_drive_y()
        ySpeed = self.yslew.calculate(yraw)
        if yraw == 0:
            ySpeed /= 2
        ySpeed *= kMaxSpeed
        if self.swapped:
            ySpeed = -ySpeed
        return ySpeed*master_throttle

    def _get_rotation(self) -> float:
        master_throttle = self.controller.get_master_throttle()
        rotraw = self.controller.get_drive_rot()
        rot = self.rotslew.calculate(rotraw)
        if rotraw == 0:
            rot /= 2
        rot *= kMaxAngularSpeed
        return rot*master_throttle

    def lock_heading_helper(self, rot):
        curr = self.drivetrain.get_heading_rotation_2d().degrees()
        # If the user is commanding rotation set the desired heading to the
        # current heading so if they let off we can use PID to keep the robot
        # driving straight
        if rot != 0:
            self.lock_heading()
        else:
            error = curr - self.desired_heading
            if abs(error) > 1.0:
                rot = self.straight_drive_pid.calculate(curr,
                                                        self.desired_heading)
            else:
                rot = 0
        return rot

    def get_stick_data(self):
        xSpeed = self._get_x_speed()
        ySpeed = self._get_y_speed()
        rot = self._get_rotation()
        # TODO: Move to an InstantCommand
        if self.controller.get_slow_mode():
            self.slow_mode = True
            slow_mode_factor = 1/2
            xSpeed *= slow_mode_factor
            ySpeed *= slow_mode_factor
            rot *= slow_mode_factor
        return xSpeed, ySpeed, rot

    def execute(self) -> None:
        xSpeed, ySpeed, rot = self.get_stick_data()
        self.drivetrain.drive(xSpeed, ySpeed, rot, robot_centric_force=False)
