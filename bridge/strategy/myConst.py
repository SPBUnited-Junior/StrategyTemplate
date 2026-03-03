from enum import Enum

class whatWeDoStates(Enum):
    """Класс с типо запускаемого нами кода"""

    Play = 0
    BothPlay = 1
    TestPass = 2
    SimpleTest = 3
    TestRotateWithBall = 4
    NewIsBallInTest = 5



idFirstAttacker: int = 2
idSecondAttacker: int = 3

timerForRotate = 0.5
timerForHoldBall = 3

whatWeDoAtThisRun: whatWeDoStates = whatWeDoStates.TestPass

minErrAngleForRotateWithBall: int = 5