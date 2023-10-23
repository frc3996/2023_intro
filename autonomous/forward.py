from magicbot import AutonomousStateMachine, timed_state, tunable
from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

from subsystems.drivesubsystem import DriveSubsystem


class DriveForward(StatefulAutonomous):
    MODE_NAME = "Drive Forward"
    drive: DriveSubsystem
    drive_speed = 0

    def initialize(self):
        # This allows you to tune the variable via the SmartDashboard over
        # networktables
        self.register_sd_var("drive_speed", 1)

    @timed_state(duration=0.5, next_state="drive_forward", first=True)
    def drive_wait(self):
        self.drive.arcadeDrive(0, 0)

    @timed_state(duration=1, next_state="stop")
    def drive_forward(self):
        self.drive.arcadeDrive(self.drive_speed, 0)

    @state()  # Remove or modify this to add additional states to this class.
    def stop(self):
        self.drive.arcadeDrive(0, 0)
