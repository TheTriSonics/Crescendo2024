#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import json
import math
from subsystems.leds import Leds
import subsystems.swervemodule as swervemodule
import subsystems.gyro as gyro
from commands2 import Subsystem, Command
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.controller import PIDController
from wpimath.kinematics import (
    SwerveModuleState, SwerveDrive4Kinematics,
    SwerveModulePosition, ChassisSpeeds
)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import (
    HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
)

from controllers.driver import DriverController
from constants import RobotMotorMap as RMM
from subsystems.note_tracker import NoteTracker
from subsystems.gyro import Gyro
from constants import RobotPIDConstants as PIDC

kMaxSpeed = 4.5  # m/s
kMaxAngularSpeed = math.pi * 5

swerve_offset = 55 / 100  # cm converted to meters

slow_mode_factor = 1/2


class Drivetrain(Subsystem):
    """
    Represents a swerve drive style drivetrain.
    """
    def __init__(self, gyro: gyro.Gyro, driver_controller, photon: NoteTracker,
                 leds: Leds) -> None:
        super().__init__()
        # TODO: Set these to the right numbers in centimeters
        self.frontLeftLocation = Translation2d(swerve_offset, swerve_offset)
        self.frontRightLocation = Translation2d(swerve_offset, -swerve_offset)
        self.backLeftLocation = Translation2d(-swerve_offset, swerve_offset)
        self.backRightLocation = Translation2d(-swerve_offset, -swerve_offset)
        self.photon = photon
        self.leds = leds

        self.lockable = False
        self.locked = False
        self.vision_stable = False

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
            brake=True,
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
            (0, 0, 0),  # wheel std devs for Kalman filters
            (0, 0, 0),  # vision std devs for Kalman filters
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

        defcmd = DrivetrainDefaultCommand(self, self.controller, photon, leds, gyro)
        self.setDefaultCommand(defcmd)

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

    def get_fid_heading(self, id) -> tuple[list[Pose2d], float]:
        tag_heading = None
        data = self.ll_json_entry.get()
        obj = json.loads(data)
        # tl = None
        # Short circuit any JSON processing if we got back an empty list, which
        # is the default value for the limelight network table entry
        if len(obj) == 0:
            return None
        results = obj['Results']
        if 'Fiducial' not in results:
            return None
        fids = results['Fiducial']
        for f in fids:
            if f['fID'] != id:
                continue
            tag_heading = f['tx']
        return tag_heading

    def periodic(self) -> None:
        self.updateOdometry()
        SmartDashboard.putBoolean("drivetrain/field_relative", self.fieldRelative)


class DrivetrainDefaultCommand(Command):
    """
    Default command for the drivetrain.
    """
    def __init__(self, drivetrain: Drivetrain, controller: DriverController,
                 photon: NoteTracker, leds: Leds, gyro: Gyro) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.controller = controller
        self.photon = photon
        self.leds = leds
        self.gyro = gyro
        self.note_pid = PIDController(*PIDC.note_tracking_pid)
        self.note_translate_pid = PIDController(*PIDC.note_translate_pid)
        self.speaker_pid = PIDController(*PIDC.speaker_tracking_pid)
        self.straight_drive_pid = PIDController(*PIDC.straight_drive_pid)
        self.straight_drive_pid.setTolerance(2.0)
        self.zero_rotation_counter = 0
        # Slew rate limiters to make joystick inputs more gentle
        self.xslew = SlewRateLimiter(5.0)
        self.yslew = SlewRateLimiter(5.0)
        self.rotslew = SlewRateLimiter(2)
        self.idle_counter = 0
        self.desired_heading = None
        self.addRequirements(drivetrain)

    def _curr_heading(self) -> float:
        return self.drivetrain.get_heading_rotation_2d().degrees()

    def lock_heading(self):
        self.desired_heading = self._curr_heading()

    def execute(self) -> None:
        if self.desired_heading is None:
            self.desired_heading = self._curr_heading()
        curr = self.drivetrain.get_heading_rotation_2d().degrees()
        xraw = self.controller.get_drive_x()
        xSpeed = self.xslew.calculate(xraw)
        if xraw == 0:
            xSpeed /= 2
        xSpeed *= kMaxSpeed

        yraw = self.controller.get_drive_y()
        ySpeed = self.yslew.calculate(yraw)
        if yraw == 0:
            ySpeed /= 2
        ySpeed *= kMaxSpeed

        rotraw = self.controller.get_drive_rot()
        rot = self.rotslew.calculate(rotraw)
        if rotraw == 0:
            rot /= 2
        rot *= kMaxAngularSpeed

        master_throttle = self.controller.get_master_throttle()
        xSpeed *= master_throttle
        ySpeed *= master_throttle
        rot *= master_throttle

        # If the user is commanding rotation set the desired heading to the
        # current heading so if they let off we can use PID to keep the robot
        # driving straight
        if rot == 0:
            self.zero_rotation_counter += 1
        if rot != 0:
            self.lock_heading()
        else:
            if abs(curr - self.desired_heading) > 1.0:
                rot = self.straight_drive_pid.calculate(curr, self.desired_heading)
            else:
                rot = 0

        if self.controller.get_yaw_reset():
            forward = 180 if self.drivetrain.shouldFlipPath() else 0
            self.gyro.set_yaw(forward)

        # When in lockon mode, the robot will rotate to face the node
        # that PhotonVision is detecting
        robot_centric_force = False
        if self.controller.get_note_lockon():
            robot_centric_force = True
            yaw = self.photon.getYawOffset()
            pitch = self.photon.getPitchOffset()
            if yaw is not None:
                if abs(yaw) < 1.7:
                    rot = 0
                    self.leds.tracking_note()
                else:
                    if pitch is not None and pitch > 0:
                        rot = self.note_pid.calculate(yaw, 0)
                    else:
                        rot = 0
                        # Force a translation to center on the note when close
                        xSpeed = self.note_translate_pid.calculate(yaw, 0)
                    self.leds.tracking_note()
            else:
                self.leds.tracking_note_not_found()

        if self.controller.get_slow_mode():
            self.leds.drivetrain_slow()
            xSpeed *= slow_mode_factor
            ySpeed *= slow_mode_factor
            rot *= slow_mode_factor

        if self.controller.get_speaker_lockon():
            fid = 4 if self.drivetrain.is_red_alliance() else 7
            heading = self.drivetrain.get_fid_heading(fid)
            if heading is not None:
                self.leds.tracking_speaker()
                if abs(heading) < 3.0:
                    rot = 0
                else:
                    rot = self.speaker_pid.calculate(heading, 0)
            else:  # Not found
                self.leds.tracking_speaker_not_found()

        """
        SmartDashboard.putNumber('xspeed', xSpeed)
        SmartDashboard.putNumber('yspeed', ySpeed)
        SmartDashboard.putNumber('rot', rot)
        # Code to lock the wheels if the robot is idle
        if xSpeed > 0 or ySpeed > 0 or rot > 0:
            self.idle_counter = 0
            self.drivetrain.locked = False
            self.drivetrain.lockable = False
        else:
            self.idle_counter += 1
        if self.idle_counter > 50:
            self.drivetrain.lockable = True
        """
        self.drivetrain.drive(xSpeed, ySpeed, rot,
                              robot_centric_force=robot_centric_force)
