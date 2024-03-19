from wpilib import Timer
from wpimath.controller import PIDController
from commands2 import Command

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
        self.drive.defcmd.note_tracking_on()
        self.addRequirements(drive, intake)

    def initialize(self):
        self.timer.restart()
        self.intake.tilt_down()
        self.intake.feed()
        pass

    def execute(self):
        pitch = self.photon.getPitchOffset()
        if pitch is not None and pitch < -15:
            self.ontop_timer.restart()

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
