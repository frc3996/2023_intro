import logging
from subsystems.drivetrain import DriveSubsystem 
from magicbot import will_reset_to


class Dummy:
    logger: logging.Logger
    drive: DriveSubsystem
    # This is changed to the value in robot.py
    SOME_CONSTANT: int

    # This gets reset after each invocation of execute()
    did_something = will_reset_to(False)

    def on_enable(self):
        """Called when the robot enters teleop or autonomous mode"""
        self.logger.info(
            "Robot is enabled: I have SOME_CONSTANT=%s", self.SOME_CONSTANT
        )

    def do_something(self):
        self.did_something = True

    def execute(self):
        pass
