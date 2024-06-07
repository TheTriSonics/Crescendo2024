from enum import Enum
from subsystems.speaker_tracker import SpeakerTracker
from subsystems.note_tracker import NoteTracker
from subsystems.drivetrain import DrivetrainDefaultCommand


class TrackerType(Enum):
    SPEAKER = 1
    NOTE = 2
    AMP = 3
    STAGE_POS_L = 4
    STAGE_POS_R = 5
    STAGE_POS_C = 6


class DrivetrainHandler(DrivetrainDefaultCommand):

    def __init__(self, speaker_tracker: SpeakerTracker,
                 note_tracker: NoteTracker, tracker_type: TrackerType):
        super().__init__()
        self.speaker_tracker = speaker_tracker
        self.note_tracker = note_tracker
        self.tracking = tracker_type

    def execute(self):
        xSpeed, ySpeed, rot = self.get_stick_data()

        robot_centric = False

        speaker_rot = self.speaker_tracker.desired_rotation
        note_rot = self.note_tracker.desired_rotation
        if self.tracking == TrackerType.SPEAKER and speaker_rot is not None:
            rot = self.speaker_tracker.desired_rotation
        elif self.tracking == TrackerType.NOTE and note_rot is not None:
            # Cut the xpeed down too
            # and force to robot centric
            # Should we also force the ySpeed to 0?
            rot = self.note_tracker.desired_rotation
            robot_centric = True
        # add more tracking ideas here.

        self.drivetrain.drive(xSpeed, ySpeed, rot,
                              robot_centric_force=robot_centric)

    def isFinished(self):
        return False
