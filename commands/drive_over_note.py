from wpilib import Timer
from wpimath.controller import PIDController
from commands2 import Command
from subsystems.note_tracker import NoteTracker
from subsystems.drivetrain import Drivetrain

from constants import RobotPIDConstants as PIDC


class DriveOverNote(Command):

    def __init__(self, photon: NoteTracker, drive: Drivetrain):
        self.timer = Timer()
        self.ontop_timer = Timer()
        self.photon = photon
        self.drive = drive
        self.pid = PIDController(*PIDC.note_tracking_pid)
        self.addRequirements(drive)
        pass

    def initialize(self):
        self.timer.start()
        pass

    def execute(self):
        print('picking up note')
        yaw = self.photon.getYawOffset()
        pitch = self.photon.getPitchOffset()
        if yaw is None or abs(yaw) < 1.7:
            rot = 0
        else:
            rot = self.pid.calculate(yaw, 0)
        if pitch is not None and pitch < -15:
            self.ontop_timer.start()
        self.drive.drive(0.3, 0, rot)

    def end(self, isInterrupted):
        self.drive.drive(0, 0, 0)

    def isFinished(self):
        timeout = self.timer.get() > 5
        pitch = self.photon.getPitchOffset()
        note_gone = (pitch is None or pitch < -15)
        ontop = note_gone and self.ontop_timer.get() > 0.25
        return timeout or ontop
