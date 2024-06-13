from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard
from networktables import NetworkTables



pn = SmartDashboard.putNumber
gn = SmartDashboard.getNumber
pb = SmartDashboard.putBoolean
gb = SmartDashboard.getBoolean

sdbase = 'fakesensors/drivetrain'



class SpeakerTracker(Subsystem):
    def __init__(self):
        super().__init__()
      
        self.ntinst = NetworkTableInstance.getDefault().getTable('limelight')
        self.ll_json = self.ntinst.getStringTopic("json")
        self.ll_json_entry = self.ll_json.getEntry('[]')

        self.fiducial_id = 14
        self.speaker_tracking = False
        self.speaker_visible = False
        self.speaker_aimed = False


    def is_red_alliance(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed


    def periodic(self):
        self.speaker_tracking = self.is_speaker_tracking()
        self.speaker_visible = self.is_speaker_visible()
        self.speaker_aimed = self.is_speaker_aimed()
        self.fid_heading = self.get_fid_heading(self.fiducial_id)


    def get_rotational_offset(self):
        targets = self.table.getNumberArray("tcornx", [])
        if self.fiducial_id in targets:
            return self.table.getNumber("tx", 0.0)
        else:
            return None
        
    def get_fid_heading(self, id) -> tuple[list[Pose2d], float]:
        tag_heading = None
        data = self.ll_json_entry.get()
        obj = json.loads(data)
        # tl = None
        # Short circuit any JSON processing if we got back an empty list, which
        # is the default value for the limelight network table entry
        if len(obj) == 0:
            return None
        results = obj['Results']
        if 'Fiducial' not in results:
            return None
        fids = results['Fiducial']
        for f in fids:
            if f['fID'] != id:
                continue
            tag_heading = f['tx']
        # pn = SmartDashboard.putNumber
        # pn(f'fids/{id}', tag_heading)
        return tag_heading




# Check all of these functions as they were just copy pasted from the drivetrain subsystem        
    def is_speaker_tracking(self):
        fake = gb(f'{sdbase}/speaker_tracking', False)
        return self.defcmd.is_speaker_tracking() or fake

    def is_speaker_visible(self):
        fake = gb(f'{sdbase}/speaker_visible', False)
        return self.defcmd.is_speaker_visible() or fake

    def is_speaker_aimed(self):
        fake = gb(f'{sdbase}/speaker_aimed', False)
        return self.defcmd.is_speaker_aimed() or fake
    
    def llJson(self) -> str:
        return self.ll_json.getEntry("[]")

