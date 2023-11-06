import typing
from dataclasses import dataclass
from typing import cast

import wpilib
import wpilib.simulation
from pyfrc.physics.core import PhysicsInterface
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor

import constants
from misc.sparksim import CANSparkMax

if typing.TYPE_CHECKING:
    from robot import MyRobot

RED = wpilib.Color8Bit(wpilib.Color.kRed)
BLUE = wpilib.Color8Bit(wpilib.Color.kBlue)
GREEN = wpilib.Color8Bit(wpilib.Color.kGreen)
GRAY = wpilib.Color8Bit(wpilib.Color.kGray)


BALL_DIAMETER = 9.5
BALL_RADIUS = 4.75
BALL_Y = 10.5
BALL_MOVE = 12 / 604.0

# positions along the intake
EDGE = 8
INDEXER_LEN = 12
SENSOR_Y = 15

ENTRY_MOTOR_START = 0
ENTRY_MOTOR_END = EDGE + 2


ENTRY_SENSOR_POS = 9
EXIT_SENSOR_POS = 20

BELT_MOTOR_START = EDGE - BALL_RADIUS
BELT_MOTOR_END = EDGE + INDEXER_LEN

SHOOTER_START = EDGE + INDEXER_LEN
SHOOTER_END = SHOOTER_START + BALL_RADIUS

BEAM_SIZE = 0.5

# climber simulation
CLIMBER_X = 21
CLIMBER_RAISED_LEN = 18
CLIMBER_LOWERED_LEN = 12


@dataclass
class Shape:
    root: wpilib.MechanismRoot2d
    items: typing.List[wpilib.MechanismLigament2d]

    def setPosition(self, x: float, y: float):
        self.root.setPosition(x, y)

    def setColor(self, c: wpilib.Color8Bit):
        for item in self.items:
            item.setColor(c)


class Mechanism(wpilib.Mechanism2d):
    def __init__(self, width: float, height: float) -> None:
        super().__init__(width, height)

    def make_point(self, name: str, cx: float, cy: float, sz: int = 10) -> Shape:
        root = self.getRoot(name, cx, cy)
        return self._make(root, 0.50, 0, 1, sz)

    def make_triangle(
        self, name: str, cx: float, cy: float, side: float, line_width=6
    ) -> Shape:
        x = cx + side / 2
        y = cy - side / 2
        root = self.getRoot(name, x, y)
        return self._make(root, side, 120, 3, line_width)

    def make_hex(
        self, name: str, x: float, y: float, side: float, line_width=6
    ) -> Shape:
        # x, y are near bottom right corner
        root = self.getRoot(name, x, y)
        return self._make(root, side, 60, 6, line_width)

    def _make(
        self,
        root: wpilib.MechanismRoot2d,
        side: float,
        angle: float,
        n: int,
        line_width,
    ) -> Shape:
        item = root
        items = []
        for i in range(n):
            item = item.appendLigament(f"{i}", side, angle, line_width)
            items.append(item)

        return Shape(root, items)


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller: PhysicsInterface = physics_controller

        # Setup drive, as long as the ports are matching..
        self.left_motor: CANSparkMax = robot.drive.left_motor_1
        self.left_motor_sim = wpilib.simulation.PWMSim(self.left_motor)
        self.right_motor: CANSparkMax = robot.drive.right_motor_1
        self.right_motor_sim = wpilib.simulation.PWMSim(self.right_motor)

        self.system = LinearSystemId.identifyDrivetrainSystem(
            constants.kV_linear,
            constants.kA_linear,
            constants.kV_angular,
            constants.kA_angular,
        )
        self.drive = wpilib.simulation.DifferentialDrivetrainSim(
            self.system,
            constants.kTrackWidth,
            DCMotor.CIM(constants.kDriveTrainMotorCount),
            constants.kGearingRatio,
            constants.kWheelRadius,
        )

        # NavX
        self.navx = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")

        # Configure encoders
        self.leftEncoderSim = (
            self.left_motor.getEncoder()
        )  # wpilib.simulation.EncoderSim(robot.encoder_l)
        self.rightEncoderSim = (
            self.right_motor.getEncoder()
        )  # wpilib.simulation.EncoderSim(robot.encoder_r)

        initialPose = Pose2d(7, 2.5, Rotation2d.fromDegrees(-160))
        self.drive.setPose(initialPose)
        self.navx_yaw.set(-cast(float, self.drive.getPose().rotation().degrees()))

        # self.intake_tuner = hal.SimDevice("Intake Tuner")
        # self.entry_sensor_pos = self.intake_tuner.createDouble(
        #     "entry sensor pos", False, ENTRY_SENSOR_POS
        # )
        # self.exit_sensor_pos = self.intake_tuner.createDouble(
        #     "exit sensor pos", False, EXIT_SENSOR_POS
        # )
        #
        # # Ball control
        # self.ball_device = hal.SimDevice("Balls")
        # self.ball_insert = self.ball_device.createBoolean("insert", False, False)

        self.draw_robot()

    def draw_robot(self):
        # drawn robot model (scale 1inch=1)

        self.model = Mechanism(30, 40)
        wpilib.SmartDashboard.putData("Model", self.model)

        outside = self.model.getRoot("outside", EDGE, 10)
        outside.appendLigament("l1", INDEXER_LEN, 0, color=GRAY)

        inside = self.model.getRoot("inside", EDGE, 20)
        inside.appendLigament("l1", INDEXER_LEN, 0, color=GRAY)

        # self.entry_sensor_pt = self.model.make_point(
        #     "entry-sensor", self.entry_sensor_pos.get(), SENSOR_Y
        # )
        #
        # self.exit_sensor_pt = self.model.make_point(
        #     "exit-sensor",
        #     self.exit_sensor_pos.get(),
        #     SENSOR_Y,
        # )
        #
        # self.entry_motor_pt = self.model.make_hex("intake-motor", 4, 20, 2.5, 4)
        # self.entry_motor_pt.setColor(GRAY)
        #
        # self.belt_motor_pt = self.model.make_hex("belt-motor", 10, 4, 2.5, 4)
        # self.belt_motor_pt.setColor(GRAY)
        #
        # self.shooter = self.model.make_hex("shooter", 27, 20, 3, 2)
        # self.shooter.setColor(GRAY)

        # The 'value' of each ball is either 'nan' (not present) or it
        # is the distance the ball lies along the track in inches
        # self.balls = []
        #
        # for i in range(2):
        #     v = self.ball_device.createDouble(f"center {i}", False, float("nan"))
        #     m = self.model.make_hex(f"ball {i}", -400, BALL_Y, 5, 1)
        #     m.setColor(RED)
        #
        #     self.balls.append((v, m))
        #
        # # climber
        # climber = self.model.getRoot("climber", CLIMBER_X, 20)
        # self.climb_extender = climber.appendLigament(
        #     "ext", CLIMBER_LOWERED_LEN, 90, color=GRAY
        # )
        # l1 = self.climb_extender.appendLigament("l1", 1, 60, color=GRAY)
        # l2 = l1.appendLigament("l2", 1, 60, color=GRAY)
        # l2.appendLigament("l3", 1, 60, color=GRAY)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drive
        left_motor_speed = self.left_motor_sim.getSpeed()
        right_motor_speed = self.right_motor_sim.getSpeed()
        voltage = wpilib.RobotController.getInputVoltage()
        self.drive.setInputs(left_motor_speed * voltage, right_motor_speed * voltage)
        self.drive.update(tm_diff)

        # Simulate the encoders
        self.leftEncoderSim.setPosition(self.drive.getLeftPosition())
        self.leftEncoderSim.setVelocity(self.drive.getLeftVelocity())
        self.rightEncoderSim.setPosition(self.drive.getRightPosition())
        self.rightEncoderSim.setVelocity(self.drive.getRightVelocity())

        # Update the gyro simulation
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        # self.gyro.setAngle(-pose.rotation().degrees())
        self.navx_yaw.set(-cast(float, self.drive.getPose().rotation().degrees()))

        # Update the Field2d
        field = cast(wpilib.Field2d, self.physics_controller.field)
        field.setRobotPose(self.drive.getPose())
