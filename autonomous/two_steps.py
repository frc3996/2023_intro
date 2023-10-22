from magicbot import AutonomousStateMachine, timed_state, tunable

from components.component2 import Component2
from subsystems.drivesubsystem import DriveSubsystem


class TwoSteps(AutonomousStateMachine):
    MODE_NAME = "Two Steps"
    DEFAULT = False

    component2: Component2

    # Injected from the definition in robot.py
    drive: DriveSubsystem

    drive_speed = tunable(-1)

    @timed_state(duration=2, next_state="do_something", first=True)
    def dont_do_something(self):
        """This happens first"""
        pass

    @timed_state(duration=5)
    def do_something(self):
        """This happens second"""
        self.component2.do_something()

    #    on_enable - Called when autonomous mode is initially enabled
    #
    #    on_disable - Called when autonomous mode is no longer active
    #
    #    on_iteration - Called for each iteration of the autonomous control loop

    def on_enable(self) -> None:
        """This function is run once each time the robot enters autonomous mode."""
        super().on_enable()

    def on_iteration(self, tm: float) -> None:
        """This function is called periodically during autonomous."""
        super().on_iteration(tm)

        # Drive for two seconds
        if tm < 2.0:
            # Drive forwards half speed, make sure to turn input squaring off
            self.drive.arcadeDrive(0.5, 0, squareInputs=False)
        else:
            self.drive.stopMotor()  # Stop robot
