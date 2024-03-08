from wpilib import Timer
from wpimath.controller import PIDController
from commands2 import Command
from commands2.wrappercommand import WrapperCommand

from subsystems.drivetrain import Drivetrain
from subsystems.intake import Intake
from subsystems.note_tracker import NoteTracker

from constants import RobotPIDConstants as PIDC

class AutoPickupNote(Command):

    def __init__(self, drive: Drivetrain, intake: Intake, photon: NoteTracker):
        self.drive = drive
        self.intake = intake
        self.photon = photon
        self.timer = Timer()
        self.ontop_timer = Timer()
        self.pid = PIDController(*PIDC.note_tracking_pid)
        self.addRequirements(drive, intake)

    def initialize(self):
        self.timer.restart()
        self.intake.tilt_down()
        self.intake.feed()
        pass

    def execute(self):
        yaw = self.photon.getYawOffset()
        pitch = self.photon.getPitchOffset()
        if yaw is None or abs(yaw) < 1.0:
            rot = 0
        else:
            rot = self.pid.calculate(yaw, 0)
        if pitch is not None and pitch < -15:
            self.ontop_timer.restart()
        self.drive.drive(0.5, 0, rot)
    
    def end(self, isInterrupted):
        self.drive.drive(0, 0, 0)
        self.intake.tilt_up()
        self.intake.halt()

    def isFinished(self):
        timeout = self.timer.get() > 5
        timeout = False 
        pitch = self.photon.getPitchOffset()
        gotnote = self.intake.is_loaded()
        ontop = (pitch is None or pitch < -15) and self.ontop_timer.get() > 1.0
        return gotnote or timeout or ontop
