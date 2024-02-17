#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import ntcore
import subsystems.swervemodule as swervemodule
from commands2 import Subsystem, Command
from wpilib import SmartDashboard, DriverStation
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import (
    SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry,
    SwerveModulePosition, ChassisSpeeds
)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import (
    HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
)

from controllers.driver import DriverController
from constants import RobotMap

kMaxSpeed = 4.8  # m/s
kMaxAngularSpeed = math.pi * 5

swerve_offset = 30 / 100  # cm converted to meters


class DrivetrainDefaultCommand(Command):
    """
    Default command for the drivetrain.
    """
    def __init__(self, drivetrain, controller: DriverController) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.controller = controller
        # Slew rate limiters to make joystick inputs more gentle
        self.xslew = SlewRateLimiter(2)
        self.yslew = SlewRateLimiter(2)
        self.rotslew = SlewRateLimiter(1)
        self.addRequirements(drivetrain)

    def execute(self) -> None:
        xSpeed = self.xslew.calculate(self.controller.get_drive_x())
        xsign = 1 if xSpeed > 0 else -1
        xSpeed = xSpeed * xSpeed * xsign * kMaxSpeed

        ySpeed = self.yslew.calculate(self.controller.get_drive_y())
        ysign = 1 if ySpeed > 0 else -1
        ySpeed = ySpeed * ySpeed * ysign * kMaxSpeed

        rot = self.rotslew.calculate(self.controller.get_drive_rot())
        rotsign = 1 if rot > 0 else -1
        rot = rot * rot * rotsign * kMaxAngularSpeed

        SmartDashboard.putNumber('xspeed', xSpeed)
        SmartDashboard.putNumber('yspeed', ySpeed)
        SmartDashboard.putNumber('rot', rot)
        self.drivetrain.drive(xSpeed, ySpeed, rot)


class Drivetrain(Subsystem):
    """
    Represents a swerve drive style drivetrain.
    """
    def __init__(self, gyro, driver_controller) -> None:
        super().__init__()
        # TODO: Set these to the right numbers in centimeters
        self.frontLeftLocation = Translation2d(swerve_offset, swerve_offset)
        self.frontRightLocation = Translation2d(swerve_offset, -swerve_offset)
        self.backLeftLocation = Translation2d(-swerve_offset, swerve_offset)
        self.backRightLocation = Translation2d(-swerve_offset, -swerve_offset)

        self.frontLeft = swervemodule.SwerveModule(
            RobotMap.front_left_drive,
            RobotMap.front_left_turn,
            RobotMap.front_left_turn_encoder,
            False,
            'Front left')
        self.frontRight = swervemodule.SwerveModule(
            RobotMap.front_right_drive,
            RobotMap.front_right_turn,
            RobotMap.front_right_turn_encoder,
            True,
            'Front right')
        self.backLeft = swervemodule.SwerveModule(
            RobotMap.back_left_drive,
            RobotMap.back_left_turn,
            RobotMap.back_left_turn_encoder,
            False,
            'Back left')
        self.backRight = swervemodule.SwerveModule(
            RobotMap.back_right_drive,
            RobotMap.back_right_turn,
            RobotMap.back_right_turn_encoder,
            True,
            'Back right')

        self.ntinst = ntcore.NetworkTableInstance.getDefault().getTable('limelight')
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

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.get_heading_rotation_2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
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
                ReplanningConfig()
            ),
            # Supplier to control path flipping based on alliance color
            self.shouldFlipPath,
            self  # Reference to this subsystem to set requirements
        )

        defcmd = DrivetrainDefaultCommand(self, self.controller)
        self.setDefaultCommand(defcmd)

    def llJson(self) -> str:
        return self.ll_json.getEntry("[]")

    def getSpeeds(self):
        cs = self.kinematics.toChassisSpeeds(
            [
                self.frontLeft.getState(),
                self.frontRight.getState(),
                self.backLeft.getState(),
                self.backRight.getState(),
            ]
        )
        SmartDashboard.putNumber("csvx", cs.vx)
        SmartDashboard.putNumber("csvy", cs.vy)
        return cs

    def driveRobotRelative(self, speeds):
        self.fieldRelative = False
        # self.drive(speeds.vx, speeds.vy, speeds.omega)
        # SmartDashboard.putNumber("vx", speeds.vx)
        # SmartDashboard.putNumber("vy", speeds.vy)
        # SmartDashboard.putNumber("omega", speeds.omega)
        # self.cs = speeds
        swerveModuleStates = self.kinematics.toSwerveModuleStates(speeds)

        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )

        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])
        pass

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
                modulePositions=[
                    self.frontLeft.getPosition(),
                    self.frontRight.getPosition(),
                    self.backLeft.getPosition(),
                    self.backRight.getPosition(),
                ],
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
        SmartDashboard.putBoolean("Fr", self.fieldRelative)
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

        swerveModuleStates = self.kinematics.toSwerveModuleStates(self.cs)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def lockWheels(self):
        for sm in [self.frontLeft, self.frontRight,
                   self.backLeft, self.backRight]:
            sm.lock()

    def setStates(self, fl: SwerveModuleState, fr: SwerveModuleState,
                  bl: SwerveModuleState, br: SwerveModuleState):
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(bl)
        self.backRight.setDesiredState(br)

    def getAngles(self) -> tuple[float, float, float, float]:
        flAng = self.frontLeft.getState().angle.radians()
        frAng = self.frontRight.getState().angle.radians()
        blAng = self.backLeft.getState().angle.radians()
        brAng = self.backRight.getState().angle.radians()
        return flAng, frAng, blAng, brAng

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
        pose = self.odometry.getPose()
        SmartDashboard.putNumber("x", pose.X())
        SmartDashboard.putNumber("y", pose.Y())
        SmartDashboard.putNumber("heading",
                                 self.get_heading_rotation_2d().degrees())

    def getPose(self) -> Pose2d:
        return self.odometry.getPose()
