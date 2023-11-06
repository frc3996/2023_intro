from typing import cast

import commands2
import rev
import wpilib
import wpilib.drive

import constants
from misc.sparksim import CANSparkMax

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
        self.left_motor_1.setInverted(constants.kLeftMotor1Inverted)

        self.left_motor_2 = CANSparkMax(
            constants.kLeftMotor2Port, CANSparkMax.MotorType.kBrushless
        )

        if EXPERIMENTAL_USE_FOLLOW:
            self.left_motor_2.follow(self.left_motor_1, constants.kLeftMotor2Inverted)
        else:
            self.left_motor_2.setInverted(constants.kLeftMotor2Inverted)
            left_group = wpilib.MotorControllerGroup(
                self.left_motor_1, self.left_motor_2
            )

        # Right side
        self.right_motor_1 = CANSparkMax(
            constants.kRightMotor1Port, CANSparkMax.MotorType.kBrushless
        )
        self.right_motor_1.setInverted(constants.kRightMotor1Inverted)

        self.right_motor_2 = CANSparkMax(
            constants.kRightMotor2Port, CANSparkMax.MotorType.kBrushless
        )
        if EXPERIMENTAL_USE_FOLLOW:
            self.right_motor_2.follow(
                self.right_motor_1, constants.kRightMotor2Inverted
            )
        else:
            self.right_motor_2.setInverted(constants.kRightMotor2Inverted)
            right_group = wpilib.MotorControllerGroup(
                self.right_motor_1, self.right_motor_2
            )

        # This is a differential drive
        if EXPERIMENTAL_USE_FOLLOW:
            self.drive = wpilib.drive.DifferentialDrive(
                self.left_motor_1, self.right_motor_1
            )
        else:
            self.drive = wpilib.drive.DifferentialDrive(left_group, right_group)

        # Easier in a list
        self.motors: list[CANSparkMax] = [
            self.left_motor_1,
            self.left_motor_2,
            self.right_motor_1,
            self.right_motor_2,
        ]

        for motor in self.motors:
            # Reset the parameters
            motor.restoreFactoryDefaults()
            # Brake or Coast?
            # motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
            # motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

            pid_controller = motor.getPIDController()
            # encoder = motor.getEncoder()

            pid_controller.setP(self.kP)
            pid_controller.setI(self.kI)
            pid_controller.setD(self.kD)
            pid_controller.setIZone(self.kIz)
            pid_controller.setFF(self.kFF)
            pid_controller.setOutputRange(self.kMinOutput, self.kMaxOutput)

            smart_motion_slot = 0
            pid_controller.setSmartMotionMaxVelocity(self.max_vel, smart_motion_slot)
            pid_controller.setSmartMotionMinOutputVelocity(
                self.min_vel, smart_motion_slot
            )
            pid_controller.setSmartMotionMaxAccel(self.max_acc, smart_motion_slot)
            pid_controller.setSmartMotionAllowedClosedLoopError(
                self.allowed_err, smart_motion_slot
            )

    def teleopPeriodic(self):
        p = cast(float, wpilib.SmartDashboard.getNumber("P Gain", 0))
        i = cast(float, wpilib.SmartDashboard.getNumber("I Gain", 0))
        d = cast(float, wpilib.SmartDashboard.getNumber("D Gain", 0))
        iz = cast(float, wpilib.SmartDashboard.getNumber("I Zone", 0))
        ff = cast(float, wpilib.SmartDashboard.getNumber("Feed Forward", 0))
        max_out = cast(float, wpilib.SmartDashboard.getNumber("Max Output", 0))
        min_out = cast(float, wpilib.SmartDashboard.getNumber("Min Output", 0))
        maxV = cast(float, wpilib.SmartDashboard.getNumber("Max Velocity", 0))
        minV = cast(float, wpilib.SmartDashboard.getNumber("Min Velocity", 0))
        maxA = cast(float, wpilib.SmartDashboard.getNumber("Max Acceleration", 0))
        allE = cast(
            float, wpilib.SmartDashboard.getNumber("Allowed Closed Loop Error", 0)
        )

        for motor in self.motors:
            pid_controller = motor.getPIDController()
            encoder = motor.getEncoder()
            if p != self.kP:
                pid_controller.setP(p)
                self.kP = p
            if i != self.kI:
                pid_controller.setI(i)
                self.kI = i
            if d != self.kD:
                pid_controller.setD(d)
                self.kD = d
            if iz != self.kIz:
                pid_controller.setIZone(iz)
                self.kIz = iz
            if ff != self.kFF:
                pid_controller.setFF(ff)
                self.kFF = ff
            if max_out != self.kMaxOutput or min_out != self.kMinOutput:
                pid_controller.setOutputRange(min_out, max_out)
                self.kMinOutput = min_out
                self.kMaxOutput = max_out
            if maxV != self.max_vel:
                pid_controller.setSmartMotionMaxVelocity(maxV, 0)
                self.max_vel = maxV
            if minV != self.min_vel:
                pid_controller.setSmartMotionMinOutputVelocity(minV, 0)
                self.min_vel = minV
            if maxA != self.max_acc:
                pid_controller.setSmartMotionMaxAccel(maxA, 0)
                self.max_acc = maxA
            if allE != self.allowed_err:
                pid_controller.setSmartMotionAllowedClosedLoopError(allE, 0)
                self.allowed_err = allE

            mode = wpilib.SmartDashboard.getBoolean("Mode", False)
            if mode:
                setpoint = cast(
                    float, wpilib.SmartDashboard.getNumber("Set Velocity", 0)
                )
                pid_controller.setReference(
                    setpoint, rev.CANSparkMax.ControlType.kVelocity
                )
                pv = encoder.getVelocity()
            else:
                setpoint = cast(
                    float, wpilib.SmartDashboard.getNumber("Set Position", 0)
                )
                pid_controller.setReference(
                    setpoint, rev.CANSparkMax.ControlType.kSmartMotion
                )
                pv = encoder.getPosition()

            wpilib.SmartDashboard.putNumber("SetPoint", setpoint)
            wpilib.SmartDashboard.putNumber("Process Variable", pv)
            wpilib.SmartDashboard.putNumber("Output", motor.getAppliedOutput())

    def arcadeDrive(
        self, xSpeed: float, zRotation: float, squareInputs: bool = True
    ) -> None:
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        self.drive.arcadeDrive(xSpeed, zRotation, squareInputs)

    def stopMotor(self):
        self.drive.stopMotor()

    def flush(self):
        pass
        # self.drive.flush()


# class Robot(wpilib.TimedRobot):
#     def robotInit(self):
#         self.motor = rev.CANSparkMax(1, rev.MotorType.kBrushless)
#
#         self.motor.restoreFactoryDefaults()
#
#         self.pid_controller = self.motor.getPIDController()
#         self.encoder = self.motor.getEncoder()
#
#         self.pid_controller.setP(constants.kP)
#         self.pid_controller.setI(constants.kI)
#         self.pid_controller.setD(constants.kD)
#         self.pid_controller.setIZone(constants.kIz)
#         self.pid_controller.setFF(constants.kFF)
#         self.pid_controller.setOutputRange(constants.kMinOutput, constants.kMaxOutput)
#
#         smart_motion_slot = 0
#         self.pid_controller.setSmartMotionMaxVelocity(constants.max_vel, smart_motion_slot)
#         self.pid_controller.setSmartMotionMinOutputVelocity(constants.min_vel, smart_motion_slot)
#         self.pid_controller.setSmartMotionMaxAccel(constants.max_acc, smart_motion_slot)
#         self.pid_controller.setSmartMotionAllowedClosedLoopError(constants.allowed_err, smart_motion_slot)
#
#         wpilib.SmartDashboard.putNumber("P Gain", constants.kP)
#         wpilib.SmartDashboard.putNumber("I Gain", constants.kI)
#         wpilib.SmartDashboard.putNumber("D Gain", constants.kD)
#         wpilib.SmartDashboard.putNumber("I Zone", constants.kIz)
#         wpilib.SmartDashboard.putNumber("Feed Forward", constants.kFF)
#         wpilib.SmartDashboard.putNumber("Max Output", constants.kMaxOutput)
#         wpilib.SmartDashboard.putNumber("Min Output", constants.kMinOutput)
#
#         wpilib.SmartDashboard.putNumber("Max Velocity", constants.max_vel)
#         wpilib.SmartDashboard.putNumber("Min Velocity", constants.min_vel)
#         wpilib.SmartDashboard.putNumber("Max Acceleration", constants.max_acc)
#         wpilib.SmartDashboard.putNumber("Allowed Closed Loop Error", constants.allowed_err)
#         wpilib.SmartDashboard.putNumber("Set Position", 0)
#         wpilib.SmartDashboard.putNumber("Set Velocity", 0)
#
#         wpilib.SmartDashboard.putBoolean("Mode", True)
#
#     def teleopPeriodic(self):
#         p = wpilib.SmartDashboard.getNumber("P Gain", 0)
#         i = wpilib.SmartDashboard.getNumber("I Gain", 0)
#         d = wpilib.SmartDashboard.getNumber("D Gain", 0)
#         iz = wpilib.SmartDashboard.getNumber("I Zone", 0)
#         ff = wpilib.SmartDashboard.getNumber("Feed Forward", 0)
#         max_out = wpilib.SmartDashboard.getNumber("Max Output", 0)
#         min_out = wpilib.SmartDashboard.getNumber("Min Output", 0)
#         maxV = wpilib.SmartDashboard.getNumber("Max Velocity", 0)
#         minV = wpilib.SmartDashboard.getNumber("Min Velocity", 0)
#         maxA = wpilib.SmartDashboard.getNumber("Max Acceleration", 0)
#         allE = wpilib.SmartDashboard.getNumber("Allowed Closed Loop Error", 0)
#
#         if p != self.kP:
#             self.pid_controller.setP(p)
#             self.kP = p
#         if i != self.kI:
#             self.pid_controller.setI(i)
#             self.kI = i
#         if d != self.kD:
#             self.pid_controller.setD(d)
#             self.kD = d
#         if iz != self.kIz:
#             self.pid_controller.setIZone(iz)
#             self.kIz = iz
#         if ff != self.kFF:
#             self.pid_controller.setFF(ff)
#             self.kFF = ff
#         if max_out != self.kMaxOutput or min_out != self.kMinOutput:
#             self.pid_controller.setOutputRange(min_out, max_out)
#             self.kMinOutput = min_out
#             self.kMaxOutput = max_out
#         if maxV != self.max_vel:
#             self.pid_controller.setSmartMotionMaxVelocity(maxV, 0)
#             self.max_vel = maxV
#         if minV != self.min_vel:
#             self.pid_controller.setSmartMotionMinOutputVelocity(minV, 0)
#             self.min_vel = minV
#         if maxA != self.max_acc:
#             self.pid_controller.setSmartMotionMaxAccel(maxA, 0)
#             self.max_acc = maxA
#         if allE != self.allowed_err:
#             self.pid_controller.setSmartMotionAllowedClosedLoopError(allE, 0)
#             self.allowed_err = allE
#
#         mode = wpilib.SmartDashboard.getBoolean("Mode", False)
#         if mode:
#             setpoint = wpilib.SmartDashboard.getNumber("Set Velocity", 0)
#             self.pid_controller.setReference(setpoint, rev.ControlType.kVelocity)
#             pv = self.encoder.getVelocity()
#         else:
#             setpoint = wpilib.SmartDashboard.getNumber("Set Position", 0)
#             self.pid_controller.setReference(setpoint, rev.ControlType.kSmartMotion)
#             pv = self.encoder.getPosition()
#
#         wpilib.SmartDashboard.putNumber("SetPoint", setpoint)
#         wpilib.SmartDashboard.putNumber("Process Variable", pv)
#         wpilib.SmartDashboard.putNumber("Output", self.motor.getAppliedOutput())
#
#
# if __name__ == "__main__":
#     wpilib.run(Robot)
