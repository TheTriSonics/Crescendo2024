from wpilib import Timer
from wpimath.controller import PIDController
from commands2 import Command
from subsystems.note_tracker import NoteTracker
from subsystems.drivetrain import Drivetrain

class DriveOverNote(Command):

    def __init__(self, photon: NoteTracker, drive: Drivetrain):
        self.timer = Timer()
        self.photon = photon
        self.pid = PIDController(0.1, 0, 0)
        self.addRequirements(drive)
        pass

    def initialize(self):
        self.timer.start()
        pass

    def execute(self):
        print('picking up note')
        yaw = self.photon.getYawOffset()
        if yaw is None or abs(yaw) < 1.7:
            rot = 0
        else:
            rot = self.pid.calculate(yaw, 0)
        if pitch < -14:
            self.ontop_timer = Timer()
            self.ontop_timer.start()
        self.drive.drive(0.3, 0, rot)


    def isFinished(self):
        timeout = self.timer.get() > 5
        pitch = self.photon.getPitchOffset()
        if pitch is None:
            return False
        ontop = pitch < -15 and self.ontop_timer.get() > 0.25
        return timeout or ontop
