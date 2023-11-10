#!/usr/bin/env python3

import magicbot
import navx
import wpilib
import constants
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
    # drive: DriveSubsystem
    # dummy: Dummy
    # pmu: PneumaticControlModule

    # You can even pass constants to components
    # SOME_CONSTANT = 1

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""

        self.drive = DriveSubsystem()
        self.dummy = Dummy()
        # self.gyro = navx.AHRS.create_spi()

        self.joystick = wpilib.XboxController(0)
        self.keyboard = Keyboard(1)

        self.compressor = wpilib.Compressor(constants.pcm_can_id, wpilib.PneumaticsModuleType.CTREPCM)
        self.pcm = wpilib.Solenoid(constants.pcm_can_id, wpilib.PneumaticsModuleType.CTREPCM, 0)

    #
    # No autonomous routine boilerplate required here, anything in the
    # autonomous folder will automatically get added to a list
    #

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        with self.consumeExceptions():

            if self.joystick.getXButton():
                self.pcm.set(1)
            else:
                self.pcm.set(0)
                

            if self.joystick.getYButton():
                self.compressor.enableDigital()
            else:
                self.compressor.disable()
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
