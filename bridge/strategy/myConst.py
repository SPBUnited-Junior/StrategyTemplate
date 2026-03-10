from enum import Enum

class whatWeDoStates(Enum):
    """Класс с типо запускаемого нами кода"""

    Play = 0
    BothPlay = 1
    TestPass = 2
    SimpleTest = 3
    TestRotateWithBall = 4
    NewIsBallInTest = 5



idFirstAttacker: int = 1
idSecondAttacker: int = 2

timerForRotate = 0.5
timerForHoldBall = 3
constForTimerWeTryDoPass = 3

velRotateWithBall = 0.4*10#rad/sec

whatWeDoAtThisRun: whatWeDoStates = whatWeDoStates.SimpleTest

minErrAngleForRotateWithBall: int = 5