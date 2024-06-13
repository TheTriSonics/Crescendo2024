from commands2 import Command
from subsystems.drivetrain import Drivetrain
from subsystems.speaker_tracker import SpeakerTracker


class SpeakerLockon(Command):

    def __init__(self, drive: Drivetrain, speaker_tracker: SpeakerTracker()):
        self.drive = drive
        self.speaker_tracker = speaker_tracker
        self.addRequirements(self.drive)

    def initialize(self):
        pass

    def execute(self):
        heading = self.speaker_tracker.speaker_heading
        if abs(heading) < 3:
            # Make no adjustment, we're already locked on
            pass
        else:

    def isFinished(self) -> bool:
        return False