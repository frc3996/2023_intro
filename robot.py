#!/usr/bin/env python3

import navx
import wpilib
import wpilib.drive
from magicbot import MagicRobot
from ntcore import NetworkTableInstance

from components.dummy import Dummy
from components.pneumatic_control_module import PneumaticControlModule
from subsystems.drivesubsystem import DriveSubsystem


class MyRobot(MagicRobot):
    #
    # Define components here
    #

    dummy: Dummy
    pmu: PneumaticControlModule

    # You can even pass constants to components
    SOME_CONSTANT = 1

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""

        self.gyro = navx.AHRS.create_spi()

        self.joystick = wpilib.XboxController(0)
        self.keyboard = wpilib.Joystick(1)

        self.drive = DriveSubsystem()

        self.compressor = wpilib.Compressor(wpilib.PneumaticsModuleType.CTREPCM)

    #
    # No autonomous routine boilerplate required here, anything in the
    # autonomous folder will automatically get added to a list
    #

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        # We do this so we don't crash in teleop
        with self.consumeExceptions():
            if self.keyboard.getRawButtonPressed(1):
                self.dummy.do_something()

            self.drive.arcadeDrive(
                -self.keyboard.getRawAxis(1), -self.keyboard.getRawAxis(0)
            )

            # if self.joystick.getLeftBumper():
            #    self.dummy.do_something()

            # self.drive.arcadeDrive(
            #    -self.joystick.getLeftY(), self.joystick.getLeftX()
            # )

    def Update_Limelight_Tracking(self) -> None:
        """
        This function implements a simple method of generating driving and
        steering commands based on the tracking data from a limelight camera.
        """
        # These numbers must be tuned for your Robot!  Be careful!
        STEER_K = 0.03  # how hard to turn toward the target
        DRIVE_K = 0.26  # how hard to drive fwd toward the target
        DESIRED_TARGET_AREA = 13.0  # Area of the target when the robot reaches the wall
        MAX_DRIVE = 0.7  # Simple speed limit so we don't drive too fast

        tv: float = (
            NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("tv")
            .getDouble(0)
        )  # type: ignore
        tx: float = (
            NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("tx")
            .getDouble(0)
        )  # type: ignore
        ty: float = (
            NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("ty")
            .getDouble(0)
        )  # type: ignore
        ta: float = (
            NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("ta")
            .getDouble(0)
        )  # type: ignore

        if tv < 1.0:
            self.m_LimelightHasValidTarget = False
            self.m_LimelightDriveCommand = 0.0
            self.m_LimelightSteerCommand = 0.0
            return

        self.m_LimelightHasValidTarget = True

        # Start with proportional steering
        steer_cmd = tx * STEER_K
        self.m_LimelightSteerCommand = steer_cmd

        # try to drive forward until the target area reaches our desired area
        drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K

        # don't let the robot drive too fast into the goal
        if drive_cmd > MAX_DRIVE:
            drive_cmd = MAX_DRIVE
        self.m_LimelightDriveCommand = drive_cmd


if __name__ == "__main__":
    wpilib.run(MyRobot)
