#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import subsystems.swervemodule as swervemodule
from commands2 import Subsystem, Command
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.controller import PIDController
from wpimath.kinematics import (
    SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry,
    SwerveModulePosition, ChassisSpeeds
)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import (
    HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
)

from controllers.thrust_driver import DriverController
from constants import RobotMotorMap as RMM
from subsystems.note_tracker import NoteTracker
from constants import RobotPIDConstants as PIDC

kMaxSpeed = 4.8  # m/s
kMaxAngularSpeed = math.pi * 5

swerve_offset = 30 / 100  # cm converted to meters


class DrivetrainDefaultCommand(Command):
    """
    Default command for the drivetrain.
    """
    def __init__(self, drivetrain, controller: DriverController, photon: NoteTracker) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.controller = controller
        self.photon = photon
        self.pid = PIDController(*PIDC.note_tracking_pid)
        # Slew rate limiters to make joystick inputs more gentle
        self.xslew = SlewRateLimiter(2)
        self.yslew = SlewRateLimiter(2)
        self.rotslew = SlewRateLimiter(1)
        self.idle_counter = 0
        self.addRequirements(drivetrain)

    def execute(self) -> None:
        xSpeed = self.xslew.calculate(self.controller.get_drive_x())
        xSpeed *= kMaxSpeed

        ySpeed = self.yslew.calculate(self.controller.get_drive_y())
        ySpeed *= kMaxSpeed

        rot = self.rotslew.calculate(self.controller.get_drive_rot())
        rot *= kMaxAngularSpeed

        master_throttle = self.controller.get_master_throttle()
        xSpeed *= master_throttle
        ySpeed *= master_throttle
        rot *= master_throttle
        
        # When in lockon mode, the robot will rotate to face the node
        # that PhtonVision is detecting
        if self.controller.get_note_lockon():
            yaw = self.photon.getYawOffset()
            if yaw is None:
                pass
            elif abs(yaw) < 1.7:
                rot = 0
            else:
                rot = self.pid.calculate(yaw, 0)

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
        self.drivetrain.drive(xSpeed, ySpeed, rot)


class Drivetrain(Subsystem):
    """
    Represents a swerve drive style drivetrain.
    """
    def __init__(self, gyro, driver_controller, photon: NoteTracker) -> None:
        super().__init__()
        # TODO: Set these to the right numbers in centimeters
        self.frontLeftLocation = Translation2d(swerve_offset, swerve_offset)
        self.frontRightLocation = Translation2d(swerve_offset, -swerve_offset)
        self.backLeftLocation = Translation2d(-swerve_offset, swerve_offset)
        self.backRightLocation = Translation2d(-swerve_offset, -swerve_offset)
        self.photon = photon

        self.lockable = False
        self.locked = False
        self.vision_stable = False

        self.frontLeft = swervemodule.SwerveModule(
            RMM.front_left_drive,
            RMM.front_left_turn,
            RMM.front_left_turn_encoder,
            False,
            'Front left')
        self.frontRight = swervemodule.SwerveModule(
            RMM.front_right_drive,
            RMM.front_right_turn,
            RMM.front_right_turn_encoder,
            True,
            'Front right')
        self.backLeft = swervemodule.SwerveModule(
            RMM.back_left_drive,
            RMM.back_left_turn,
            RMM.back_left_turn_encoder,
            False,
            'Back left')
        self.backRight = swervemodule.SwerveModule(
            RMM.back_right_drive,
            RMM.back_right_turn,
            RMM.back_right_turn_encoder,
            True,
            'Back right')

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
                PIDConstants(1.9, 0.0, 0.0),  # Translation PID constants
                PIDConstants(1.6, 0.0, 0.0),  # Rotation PID constants
                kMaxSpeed,  # Max module speed, in m/s.
                # Drive base radius in meters. Distance from robot center to
                # furthest module.
                0.431,
                # Default path replanning config. See the API for options
                ReplanningConfig(enableDynamicReplanning=True)
            ),
            # Supplier to control path flipping based on alliance color
            self.shouldFlipPath,
            self  # Reference to this subsystem to set requirements
        )

        defcmd = DrivetrainDefaultCommand(self, self.controller, photon)
        self.setDefaultCommand(defcmd)

    def llJson(self) -> str:
        return self.ll_json.getEntry("[]")

    def getSpeeds(self):
        cs = self.kinematics.toChassisSpeeds(
            [m.getState() for m in self.modules]
        )
        return cs

    def driveRobotRelative(self, speeds):
        self.fieldRelative = False
        # self.drive(speeds.vx, speeds.vy, speeds.omega)
        # SmartDashboard.putNumber("vx", speeds.vx)
        # SmartDashboard.putNumber("vy", speeds.vy)
        # SmartDashboard.putNumber("omega", speeds.omega)
        # self.cs = speeds
        states = self.kinematics.toSwerveModuleStates(speeds)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, kMaxSpeed)
        for m, s in zip(self.modules, states):
            m.setDesiredState(s)

    def shouldFlipPath(self):
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
        return Rotation2d(math.radians(self.gyro.get_yaw()))

    def toggleFieldRelative(self):
        self.fieldRelative = not self.fieldRelative

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        periodSeconds: float = 0.02,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative
        :      to the field.
        :param periodSeconds: Time
        """
        if self.fieldRelative:
            self.cs = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.get_heading_rotation_2d(),
                )
        else:
            self.cs = ChassisSpeeds(xSpeed, ySpeed, rot)

        states = self.kinematics.toSwerveModuleStates(self.cs)
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
        pose = self.getPose()
        speeds = self.getSpeeds()
        pn = SmartDashboard.putNumber
        pb = SmartDashboard.putBoolean
        pn("drivetrain/odometry/x_pos", pose.X())
        pn("drivetrain/odometry/y_pos", pose.Y())
        pn("drivetrain/odometry/heading_pos", pose.rotation().degrees())
        pn("drivetrain/odometry/x_speed", speeds.vx)
        pn("drivetrain/odometry/y_speed", speeds.vy)
        pb("drivetrain/field_relative", self.fieldRelative)
