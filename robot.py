#!/usr/bin/env python3

import magicbot
import navx
import wpilib
import wpilib.drive
from ntcore import NetworkTableInstance

from components.dummy import Dummy
from components.pneumatic_control_module import PneumaticControlModule
from misc.keyboardsim import Keyboard
from subsystems.drivetrain import DriveSubsystem


class MyRobot(magicbot.MagicRobot):
    #
    # Define components here
    #

    dummy: Dummy
    # pmu: PneumaticControlModule

    # You can even pass constants to components
    SOME_CONSTANT = 1

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""

        # self.gyro = navx.AHRS.create_spi()

        self.joystick = wpilib.XboxController(0)
        self.keyboard = Keyboard(1)

        self.drive = DriveSubsystem()

        # self.compressor = wpilib.Compressor(wpilib.PneumaticsModuleType.CTREPCM)

    #
    # No autonomous routine boilerplate required here, anything in the
    # autonomous folder will automatically get added to a list
    #

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        with self.consumeExceptions():
            # We do this so we don't crash in teleop

            # Lets try to handle both the XboxController and Keyboard
            if self.keyboard.getRawButtonPressed(1):
                self.dummy.do_something()

            if self.joystick.getLeftBumper():
                self.dummy.do_something()

            if self.keyboard.getRawAxis(1) or self.keyboard.getRawAxis(0):
                self.drive.arcadeDrive(
                    -self.keyboard.getRawAxis(1), -self.keyboard.getRawAxis(0)
                )
            if self.joystick.getLeftX() or self.joystick.getLeftY():
                self.drive.arcadeDrive(
                    -self.joystick.getLeftY(), self.joystick.getLeftX()
                )


if __name__ == "__main__":
    wpilib.run(MyRobot)
