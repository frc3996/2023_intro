import rev
import wpilib
from wpimath.controller import PIDController

if wpilib.RobotBase.isSimulation():

    class SparkMaxRelativeEncoder:
        def __init__(self) -> None:
            self._velocity = 0
            self._position = 0

        def getVelocity(self):
            return self._velocity

        def getPosition(self):
            return self._position

        def setVelocity(self, value):
            self._velocity = value

        def setPosition(self, value):
            self._position = value

    class SparkMaxPIDController:
        def __init__(self, motor: "CANSparkMax") -> None:
            self._motor = motor
            self._min_output = -1
            self._max_output = 1
            self._controller = PIDController(0, 0, 0)

        def setP(self, p):
            self._controller.setP(p)

        def setI(self, i):
            self._controller.setI(i)

        def setD(self, d):
            self._controller.setD(d)

        def setIZone(self, izone):
            pass

        def setFF(self, ff):
            pass

        def setOutputRange(self, min_output, max_output):
            self._min_output = min_output
            self._max_output = max_output

        def setReference(self, sp: float, type: "rev.CANSparkMax.ControlType"):
            v = self._controller.calculate(self._motor._encoder._position, sp)
            if v > self._max_output:
                v = self._max_output
            elif v < self._min_output:
                v = self._min_output
            self._motor.set(v)

        def setSmartMotionMaxVelocity(self, *_):
            pass

        def setSmartMotionMinOutputVelocity(self, *_):
            pass

        def setSmartMotionMaxAccel(self, *_):
            pass

        def setSmartMotionAllowedClosedLoopError(self, *_):
            pass

    class CANSparkMax(wpilib.Spark):
        class MotorType:
            kBrushless = 1

        class ControlType:
            kCurrent = rev.CANSparkMax.ControlType.kCurrent
            kDutyCycle = rev.CANSparkMax.ControlType.kDutyCycle
            kPosition = rev.CANSparkMax.ControlType.kPosition
            kSmartMotion = rev.CANSparkMax.ControlType.kSmartMotion
            kSmartVelocity = rev.CANSparkMax.ControlType.kSmartVelocity
            kVelocity = rev.CANSparkMax.ControlType.kVelocity
            kVoltage = rev.CANSparkMax.ControlType.kVoltage

        class IdleMode:
            kCoast = 0
            kBrake = 1

        def __init__(self, channel: int, ignored) -> None:
            super().__init__(channel)
            self._encoder = SparkMaxRelativeEncoder()
            self._pidController = SparkMaxPIDController(self)

        def follow(self, motor, invert):
            pass

        def getEncoder(self):
            return self._encoder

        def getPIDController(self):
            return self._pidController

        def setIdleMode(self, mode):
            pass

        def restoreFactoryDefaults(self):
            pass

else:
    import rev

    CANSparkMax = rev.CANSparkMax
    SparkMaxPIDController = rev.SparkMaxPIDController
