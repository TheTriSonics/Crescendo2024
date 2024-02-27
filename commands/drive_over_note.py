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
        pitch = self.photon.getPitchOffset()
        rot = 0.1 if yaw > 0 else -0.1


    def isFinished(self):
        timeout = self.timer.get() > 5
        ontop = False
        return timeout or ontop
