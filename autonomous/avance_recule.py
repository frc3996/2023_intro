import ntcore
from magicbot.state_machine import AutonomousStateMachine, state, timed_state

from subsystems.drivesubsystem import DriveSubsystem


class BaseAuto(AutonomousStateMachine):
    """
    Cette classe est utilisée pour terminer les divers mode autonomes.
    l'état 'failed' ou 'finish' devrait être appelé une fois terminé.
    """

    # driver.flush() permet de réduire des problèmes potentiel sur la drive.
    drive: DriveSubsystem

    @state
    def failed(self):
        """
        Cet état est appelé par défaut si un mode auto a échoué
        """
        self.next_state("finish")

    @state
    def finish(self):
        self.drive.flush()
        self.done()


class AvanceEtRecule(BaseAuto):
    MODE_NAME = "AvanceEtRecule"
    DEFAULT = False

    @timed_state(duration=1, next_state="avance", first=True)
    def init(self):
        # Ne pas oublier de pré-définir les variables dans la fonction
        # 'robotInit' de robot.py
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")
        self.skip_recule = self.nt.getNumber("auto_mode/skip_recule", False)

    @timed_state(duration=1, next_state="recule")
    def avance(self):
        self.drive.arcadeDrive(0.1, 0)

    @timed_state(duration=1, next_state="finish")
    def recule(self):
        if self.skip_recule is True:
            # Il est possible de forcer le changement immédiat vers un autre état avec:
            self.next_state("finish")

        self.drive.arcadeDrive(-0.1, 0)
