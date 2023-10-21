#!/usr/bin/env python3

import wpilib
import wpilib.drive
from components.component1 import Component1
from components.component2 import Component2
from magicbot import MagicRobot
from subsystems.drivesubsystem import DriveSubsystem


class MyRobot(MagicRobot):
    #
    # Define components here
    #

    component1: Component1
    component2: Component2

    # You can even pass constants to components
    SOME_CONSTANT = 1

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""

        self.component1_motor = wpilib.Talon(1)
        self.some_motor = wpilib.Talon(2)

        # self.gyro = navx.AHRS.create_spi()
        self.joystick = wpilib.XboxController(0)

        self.drive = DriveSubsystem()

    #
    # No autonomous routine boilerplate required here, anything in the
    # autonomous folder will automatically get added to a list
    #

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        with self.consumeExceptions():
            if self.joystick.getLeftBumper():
                self.component2.do_something()

            self.drive.arcadeDrive(-self.joystick.getLeftY(), self.joystick.getLeftX())


if __name__ == "__main__":
    wpilib.run(MyRobot)
