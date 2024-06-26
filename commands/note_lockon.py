
from commands2 import CommandScheduler, Subsystem, Command
from wpilib import SmartDashboard, DriverStation
import subsystems.note_tracker as note_tracker
from subsystems.intake import Intake
from subsystems.photoeyes import Photoeyes
import subsystems.gyro as gyro
from wpimath.filter import SlewRateLimiter, LinearFilter

pn = SmartDashboard.putNumber
yaw_raw = None

class NoteLockOnCommand(Command):
    def __init__(self, drivetrain, intake_note, photon: note_tracker.NoteTracker):
        super().__init__()
        self.drivetrain = drivetrain
        self.intake_note = intake_note
        self.photon = photon
        
    def periodic(self):
        yaw_raw = self.photon.getYawOffset()
        pn('drivetrain/note_tracker/yaw_raw', yaw_raw)

    def execute(self):
        if yaw_raw == None: return
        self.note_visible = True
        self.note_yaw_filtered = LinearFilter.highPass(0.1, 0.02)
        self.current_yaw = self.note_yaw_filtered.calculate(yaw_raw)
        pn('drivetrain/note_tracker/yaw', self.current_yaw)
        # pitch = self.photon.getPitchOffset()

    def end(self):
        self.note_visible = False
        pass