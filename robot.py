# All units of length will be in meters unless otherwise noted

import json
import wpimath
import subsystems.drivetrain as drivetrain
import subsystems.photoeyes as photoeyes
import subsystems.shooter as shooter
import subsystems.intake as intake

from wpilib import SmartDashboard, Joystick
from commands2 import TimedCommandRobot, SequentialCommandGroup
from commands2.button import CommandJoystick
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.commands import FollowPathHolonomic
from pathplannerlib.config import (
    HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
)

from commands.rotate import Rotate
from commands.drivefordistance import DriveForDistance
from commands.haltdrive import HaltDrive
from commands.drivetopoint import DriveToPoint
from commands.shooter_launch_note import ShooterLaunchNote
from subsystems.gyro import Gyro

from constants import RobotMap

from controllers.driver import DriverController
from controllers.commander import CommanderController


class MyRobot(TimedCommandRobot):

    def robotInit(self) -> None:
        """Robot initialization function"""
        self.driver = DriverController(
            Joystick(RobotMap.driver_controller)
        )
        self.commander = CommanderController(
            Joystick(RobotMap.commander_controller)
        )
        self.gyro = Gyro()
        self.photoeyes = photoeyes.PhotoEyes()

        self.shooter = shooter.Shooter()
        self.intake = intake.Intake(self.commander, self.photoeyes)
        self.swerve = drivetrain.Drivetrain(self.gyro, self.driver)

    def autonomousInit(self):
        # cmd = DriveForDistance(self.swerve, 50)
        # cmd = HaltDrive(self.swerve)
        self.swerve.resetOdometry()
        self.gyro.set_yaw(45)
        cmd = PathPlannerAuto("Goofy")
        # cmd = Rotate(self.swerve, self.gyro, 0)
        haltcmd = HaltDrive(self.swerve)
        scg = SequentialCommandGroup([cmd, haltcmd])
        scg.schedule()
        """
        drive1 = DriveToPoint(self.swerve, self.gyro, 200, 100, 180)
        halt1 = HaltDrive(self.swerve)
        drive2 = DriveToPoint(self.swerve, self.gyro, 0, 0, 0)
        halt2 = HaltDrive(self.swerve)
        scg = commands2.SequentialCommandGroup([drive1, halt1, drive2, halt2])
        scg.schedule()
        """
        pass

    def autonomousPeriodic(self) -> None:
        self.swerve.updateOdometry()
        SmartDashboard.putNumber("yaw", self.gyro.get_yaw())

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:

        self.swerve.updateOdometry()
        SmartDashboard.putNumber("yaw", self.gyro.get_yaw())
        currx, curry = self.getVisionXY()

        if currx is not None and curry is not None:
            SmartDashboard.putNumber('vx', currx)
            SmartDashboard.putNumber('vy', curry)

    def getVisionXY(self):
        data = self.swerve.ll_json_entry.get()
        obj = json.loads(data)
        totalx, totaly = 0, 0
        currx, curry = None, None
        if len(obj) > 0 and 'Results' in obj.keys():
            obj = obj['Results']
            if 'Fiducial' in obj.keys():
                obj = obj['Fiducial']
                targets = len(obj)
                pp = json.dumps(obj, indent=4)
                for f in obj:
                    totalx += f['tx']
                    totaly += f['ty']
                if targets > 0:
                    currx = totalx / targets
                    curry = totaly / targets
        return currx, curry

    def disabledPeriodic(self) -> None:
        SmartDashboard.putNumber("FL encoder", self.swerve.frontLeft.driveMotor.get_position().value)

    '''
    def followPathCommand(self, pathName: str):
        path = PathPlannerPath.fromPathFile(pathName)
        return FollowPathHolonomic(
            path,
            self.swerve.getPose, # Robot pose supplier
            self.swerve.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.swerve.driveRobotRelative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0), # Rotation PID constants
                4.5, # Max module speed, in m/s
                0.4, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.swerve.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )
    '''
