import typing

import constants
import wpilib
import wpilib.simulation
from pyfrc.physics import drivetrains
from pyfrc.physics.core import PhysicsInterface
from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        # Configure drivetrain
        # self.drivetrain = drivetrains.TwoMotorDrivetrain(
        #    deadzone=drivetrains.linear_deadzone(0.2)
        # )

        # Setup the motors
        self.l_motor = wpilib.simulation.PWMSim(10)
        self.r_motor = wpilib.simulation.PWMSim(12)

        self.system = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3)
        self.drivetrain = wpilib.simulation.DifferentialDrivetrainSim(
            self.system,
            constants.kTrackWidth,
            DCMotor.CIM(constants.kDriveTrainMotorCount),
            constants.kGearingRatio,
            constants.kWheelRadius,
        )

        # Configure encoders
        self.leftEncoderSim = wpilib.simulation.EncoderSim(robot.drive.leftEncoder)
        self.rightEncoderSim = wpilib.simulation.EncoderSim(robot.drive.rightEncoder)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        # speeds = self.drivetrain.calculate(l_motor, r_motor)
        # self.physics_controller.drive(speeds, tm_diff)
        voltage = wpilib.RobotController.getInputVoltage()
        self.drivetrain.setInputs(l_motor * voltage, r_motor * voltage)
        self.drivetrain.update(tm_diff)

        self.physics_controller.field.setRobotPose(self.drivetrain.getPose())

        # optional: compute encoder
        # encoder = self.drivetrain.wheelSpeeds.left * tm_diff
