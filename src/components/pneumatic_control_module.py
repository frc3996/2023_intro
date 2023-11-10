import logging

import wpilib


class PneumaticControlModule:
    compressor: wpilib.Compressor
    logger: logging.Logger

    def on_enable(self):
        """Called when the robot enters teleop or autonomous mode"""
        self.logger.info("Robot is enabled: Starting compressor")

    def execute(self):
        pass
