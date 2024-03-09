import commands2
import math

from wpilib import SmartDashboard, Timer
from subsystems.drivetrain import Drivetrain
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState


class HaltDrive(commands2.CommandBase):

    def __init__(self, drive: Drivetrain, forever = False):
        super().__init__()
        self.drive = drive
        self.forever = forever
        self.addRequirements(drive)

    def initialize(self):
        print('locking robot into x shape')
        # We'll use a timer to end this command after a certain amount of time
        # that way if there's a malfunction it still ends and also in the
        # simulator it eventually ends even if the physics engine is broken.
        self.timeout = Timer()
        self.timeout.start()
        # Once this command has started reste the drivetrain's subsystem lock
        # value. That way we do't lock it over and over.
        self.drive.lockable = False
        pass

    def execute(self):
        # Drive the robot wheels into an X configuration by asking the
        # drivetrain kinematis to push the wheels into the correct angles.
        defaultState = [
            SwerveModuleState(0, Rotation2d(-math.pi/4)),
            SwerveModuleState(0, Rotation2d(math.pi/4)),
            SwerveModuleState(0, Rotation2d(math.pi/4)),
            SwerveModuleState(0, Rotation2d(-math.pi/4)),
        ]
        self.drive.setStates(defaultState[0], defaultState[1],
                             defaultState[2], defaultState[3])

    def end(self, i):
        SmartDashboard.putString("halt", "done again")
        print('robot locked in x shape')
        self.drive.locked = True
        self.drive.lockWheels()

    def isFinished(self):
        if self.forever:
            return False
        # The command end when the wheels are close enough to our X shape
        # to call it good. The value of 4 degrees of accumlated error in
        # all the wheels seems to work well on the chassis bot.
        fl, fr, bl, br = self.drive.getAngles()
        error = 0
        error += abs(fl-(-math.pi/4))
        error += abs(fr-(math.pi/4))
        error += abs(bl-(math.pi/4))
        error += abs(br-(-math.pi/4))
        pn = SmartDashboard.putNumber
        error_threshold = 4
        # pn("commands/haltdrive/angle_error", math.degrees(error))
        # pn("commands/haltdrive/error_threshold", error_threshold)
        timeout = self.timeout.get() > 1  # End after 1 second no matter what
        return error < math.radians(error_threshold) or timeout
