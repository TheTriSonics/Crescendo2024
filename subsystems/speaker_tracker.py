# Subsystem to tell the rest of the robot
# where the FIDs on the speakers are relative to the roobt
# Uses the limelight camera system.

import json
from commands2 import Subsystem
from ntcore import NetworkTableInstance


class SpeakerTracker(Subsystem):

    def __init__(self):
        self.ntinst = NetworkTableInstance.getDefault().getTable('limelight')
        self.ll_json = self.ntinst.getStringTopic("json")
        self.ll_json_entry = self.ll_json.getEntry('[]')

    # This should get the data from limelight
    def periodic(self):
        # read data from limelight
        data = self.ll_json_entry.get()
        obj = json.loads(data)
        if len(obj) == 0:
            # Let the system know we have no clue
            self.speaker_heading = None
            pass
        results = obj['Results']
        if 'Fiducial' not in results:
            # Let the system know we have no clue
            self.speaker_heading = None
            pass
        fids = results['Fiducial']
        for f in fids:
            if f['fID'] != id:
                continue
            tag_heading = f['tx']
        # pn = SmartDashboard.putNumber
        # pn(f'fids/{id}', tag_heading)
        # Let the system know what the speaker's heading is
        self.speaker_heading = tag_heading
        # Now determine what to do with the rotation
        # value.
        self.desired_rotation = None
        if abs(tag_heading) < 3:
            self.desired_rotation = 0
        else:
            if tag_heading < 0:
                self.desired_rotation = -1
            else:
                self.desired_rotation = 1
