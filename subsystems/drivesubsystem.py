import commands2
import constants
import wpilib
import wpilib.drive


class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.init_drivetrain()
        self.init_encoders()

    def init_encoders(self):
        # The left-side drive encoder
        self.leftEncoder = wpilib.Encoder(
            *constants.kLeftEncoderPorts,
            reverseDirection=constants.kLeftEncoderReversed
        )

        # The right-side drive encoder
        self.rightEncoder = wpilib.Encoder(
            *constants.kRightEncoderPorts,
            reverseDirection=constants.kRightEncoderReversed
        )

        # Sets the distance per pulse for the encoders
        self.leftEncoder.setDistancePerPulse(constants.kEncoderDistancePerPulse)
        self.rightEncoder.setDistancePerPulse(constants.kEncoderDistancePerPulse)

    def init_drivetrain(self):
        br, fr, bl, fl = (10, 11, 12, 13)

        self.br_motor = wpilib.PWMSparkMax(br)
        self.bl_motor = wpilib.PWMSparkMax(bl)
        self.fl_motor = wpilib.PWMSparkMax(fl)
        self.fr_motor = wpilib.PWMSparkMax(fr)
        # self.fr_motor.setInverted(True)
        # self.br_motor.setInverted(True)

        self.drive = wpilib.drive.DifferentialDrive(
            wpilib.MotorControllerGroup(self.bl_motor, self.fl_motor),
            wpilib.MotorControllerGroup(self.br_motor, self.fr_motor),
        )

    def arcadeDrive(
        self, xSpeed: float, zRotation: float, squareInputs: bool = True
    ) -> None:
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        self.drive.arcadeDrive(xSpeed, zRotation, squareInputs)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""

    def getAverageEncoderDistance(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return (self.leftEncoder.getDistance() + self.rightEncoder.getDistance()) / 2.0

    def setMaxOutput(self, maxOutput: float):
        """
        Sets the max output of the drive. Useful for scaling the
        drive to drive more slowly.
        """
        self.drive.setMaxOutput(maxOutput)

    def stopMotor(self):
        self.drive.stopMotor()
