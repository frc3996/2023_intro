import wpilib

if wpilib.RobotBase.isSimulation():
    Keyboard = wpilib.Joystick
else:

    class Keyboard:
        def __init__(self, id):
            pass

        def getRawAxis(self, value):
            return 0

        def getRawButtonPressed(self, value):
            return 0
