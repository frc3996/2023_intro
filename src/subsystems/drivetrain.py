from dataclasses import dataclass
from typing import cast

import commands2
import rev
import wpilib
import wpilib.drive
from wpimath.controller import PIDController

import constants
from misc.sparksim import CANSparkMax, SparkMaxPIDController


@dataclass
class RikiMotor:
    motor: CANSparkMax
    pid_controller: SparkMaxPIDController


# This feature should allow a motor to follow the voltage of another, it doesn't
# work in simulation, is it better than a MotorControllerGroup?
EXPERIMENTAL_USE_FOLLOW = 0


class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.kP: float = constants.kP
        self.kI: float = constants.kI
        self.kD: float = constants.kD
        self.kIz: float = constants.kIz
        self.kFF: float = constants.kFF
        self.kMaxOutput: float = constants.kMaxOutput
        self.kMinOutput: float = constants.kMinOutput
        self.max_rpm: float = constants.max_rpm
        self.max_vel: float = constants.max_vel
        self.max_acc: float = constants.max_acc
        self.min_vel: float = constants.min_vel
        self.allowed_err: float = constants.allowed_err

        self.init_drivetrain()
        self.init_dashboard()

    def init_dashboard(self):
        wpilib.SmartDashboard.putNumber("P Gain", self.kP)
        wpilib.SmartDashboard.putNumber("I Gain", self.kI)
        wpilib.SmartDashboard.putNumber("D Gain", self.kD)
        wpilib.SmartDashboard.putNumber("I Zone", self.kIz)
        wpilib.SmartDashboard.putNumber("Feed Forward", self.kFF)
        wpilib.SmartDashboard.putNumber("Max Output", self.kMaxOutput)
        wpilib.SmartDashboard.putNumber("Min Output", self.kMinOutput)

        wpilib.SmartDashboard.putNumber("Max Velocity", self.max_vel)
        wpilib.SmartDashboard.putNumber("Min Velocity", self.min_vel)
        wpilib.SmartDashboard.putNumber("Max Acceleration", self.max_acc)
        wpilib.SmartDashboard.putNumber("Allowed Closed Loop Error", self.allowed_err)
        wpilib.SmartDashboard.putNumber("Set Position", 0)
        wpilib.SmartDashboard.putNumber("Set Velocity", 0)

        wpilib.SmartDashboard.putBoolean("Mode", True)

    def init_drivetrain(self):
        # Left side
        self.left_motor_1 = CANSparkMax(
            constants.kLeftMotor1Port, CANSparkMax.MotorType.kBrushless
        )
        self.left_motor_2 = CANSparkMax(
            constants.kLeftMotor2Port, CANSparkMax.MotorType.kBrushless
        )
        # Right side
        self.right_motor_1 = CANSparkMax(
            constants.kRightMotor1Port, CANSparkMax.MotorType.kBrushless
        )
        self.right_motor_2 = CANSparkMax(
            constants.kRightMotor2Port, CANSparkMax.MotorType.kBrushless
        )

        # This is a differential drive
        if EXPERIMENTAL_USE_FOLLOW:
            self.left_motor_2.follow(self.left_motor_1, constants.kLeftMotor2Inverted)
            self.right_motor_2.follow(
                self.right_motor_1, constants.kRightMotor2Inverted
            )
            self.drive = wpilib.drive.DifferentialDrive(
                self.left_motor_1, self.right_motor_1
            )
        else:
            left_group = wpilib.MotorControllerGroup(
                self.left_motor_1, self.left_motor_2
            )
            right_group = wpilib.MotorControllerGroup(
                self.right_motor_1, self.right_motor_2
            )
            self.drive = wpilib.drive.DifferentialDrive(left_group, right_group)

        # Easier in a list

        self.motors: list[RikiMotor] = [
            RikiMotor(self.left_motor_1, None),
            RikiMotor(self.left_motor_2, None),
            RikiMotor(self.right_motor_1, None),
            RikiMotor(self.right_motor_2, None),
        ]

        for riki_motor in self.motors:
            # Reset the parameters
            riki_motor.motor.restoreFactoryDefaults()
            # Brake or Coast?
            # motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
            riki_motor.motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

            riki_motor.pid_controller = riki_motor.motor.getPIDController()
            # encoder = motor.getEncoder()

            riki_motor.pid_controller.setP(self.kP)
            riki_motor.pid_controller.setI(self.kI)
            riki_motor.pid_controller.setD(self.kD)
            riki_motor.pid_controller.setIZone(self.kIz)
            riki_motor.pid_controller.setFF(self.kFF)
            riki_motor.pid_controller.setOutputRange(self.kMinOutput, self.kMaxOutput)

            smart_motion_slot = 0
            riki_motor.pid_controller.setSmartMotionMaxVelocity(
                self.max_vel, smart_motion_slot
            )
            riki_motor.pid_controller.setSmartMotionMinOutputVelocity(
                self.min_vel, smart_motion_slot
            )
            riki_motor.pid_controller.setSmartMotionMaxAccel(
                self.max_acc, smart_motion_slot
            )
            riki_motor.pid_controller.setSmartMotionAllowedClosedLoopError(
                self.allowed_err, smart_motion_slot
            )

        self.left_motor_1.setInverted(constants.kLeftMotor1Inverted)
        self.left_motor_2.setInverted(constants.kLeftMotor2Inverted)
        self.right_motor_1.setInverted(constants.kRightMotor1Inverted)
        self.right_motor_2.setInverted(constants.kRightMotor2Inverted)

    def arcadeDrive(
        self, xSpeed: float, zRotation: float, squareInputs: bool = True
    ) -> None:
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        # self.motors[0].pid_controller.setReference(
        #     xSpeed * 1000, CANSparkMax.ControlType.kSmartMotion
        # )
        # self.motors[1].pid_controller.setReference(
        #     xSpeed * 1000, CANSparkMax.ControlType.kSmartMotion
        # )
        # self.motors[2].pid_controller.setReference(
        #     xSpeed * 1000, CANSparkMax.ControlType.kSmartMotion
        # )
        # self.motors[3].pid_controller.setReference(
        #     xSpeed * 1000, CANSparkMax.ControlType.kSmartMotion
        # )
        self.drive.arcadeDrive(xSpeed, zRotation)

    def stopMotor(self):
        self.drive.stopMotor()

    def flush(self):
        pass
        # self.drive.flush()
