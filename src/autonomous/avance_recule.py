import ntcore
from magicbot import AutonomousStateMachine, timed_state, tunable
from magicbot.state_machine import AutonomousStateMachine, state, timed_state

from subsystems.drivetrain import DriveSubsystem


class AvanceEtRecule(AutonomousStateMachine):
    MODE_NAME = "AvanceEtRecule"
    DEFAULT = False
    drive: DriveSubsystem
    drive_speed = tunable(1)

    @timed_state(duration=1, next_state="avance", first=True)
    def init(self):
        # Ne pas oublier de pré-définir les variables dans la fonction
        # 'robotInit' de robot.py
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")
        self.skip_recule = self.nt.getNumber("auto_mode/skip_recule", False)

    @timed_state(duration=1, next_state="recule")
    def avance(self):
        self.drive.arcadeDrive(self.drive_speed, 0)

    @timed_state(duration=1, next_state="stop")
    def recule(self):
        if self.skip_recule is True:
            # Il est possible de forcer le changement immédiat vers un autre état avec:
            self.next_state("stop")

        self.drive.arcadeDrive(-self.drive_speed, 0)

    @state()
    def stop(self):
        self.drive.arcadeDrive(0, 0)
