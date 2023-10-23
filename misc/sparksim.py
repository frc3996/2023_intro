import rev
import wpilib

# if wpilib.RobotBase.isSimulation():
#
#     class SparkMaxRelativeEncoder:
#         def __init__(self) -> None:
#             self._velocity = 0
#
#         def getVelocity(self):
#             return self._velocity
#
#     class CANSparkMax(wpilib.Spark):
#         def __init__(self, channel: int, ignored) -> None:
#             super().__init__(channel)
#             self._encoder = SparkMaxRelativeEncoder()
#
#         def getEncoder(self):
#             return self._encoder
#
#         def setIdleMode(self, mode):
#             pass
#
# else:
#     import rev
#
#     CANSparkMax = rev.CANSparkMax

if wpilib.RobotBase.isSimulation():

    class SparkMaxPIDController:
        def __init__(self):
            pass

        def setP(self, _):
            pass

        def setI(self, _):
            pass

        def setD(self, _):
            pass

        def setIZone(self, _):
            pass

        def setFF(self, _):
            pass

        def setOutputRange(self, *_):
            pass

        def setSmartMotionMaxVelocity(self, *_):
            pass

        def setSmartMotionMinOutputVelocity(self, *_):
            pass

        def setSmartMotionMaxAccel(self, *_):
            pass

        def setSmartMotionAllowedClosedLoopError(self, *_):
            pass

    class SparkMaxRelativeEncoder:
        def __init__(self) -> None:
            self._velocity = 0

        def getVelocity(self):
            return self._velocity

    class CANSparkMax(wpilib.Spark):
        def __init__(self, channel: int, _) -> None:
            super().__init__(channel)
            self._encoder = SparkMaxRelativeEncoder()
            self._pid_controller = SparkMaxPIDController()

        def getEncoder(self):
            return self._encoder

        def setIdleMode(self, mode):
            pass

        def restoreFactoryDefaults(self):
            pass

        def getPIDController(self) -> SparkMaxPIDController:
            return self._pid_controller

        class MotorType(rev.CANSparkMax.MotorType):
            pass

else:
    from rev import CANSparkMax
