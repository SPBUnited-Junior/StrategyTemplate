from enum import Enum
from bridge import const
from math import asin

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
constForTimerWeTryDoPass = 3

velRotateWithBall = 0.4#rad/sec
if const.IS_SIMULATOR_USED:
    velRotateWithBall *= 5

maxDistForScore = 2000
minDistForScorePenalty = 1200

distBetweenRsInWall = 250
angleBetweenRsInWall = asin((distBetweenRsInWall/2)/((distBetweenRsInWall/2)**2+(const.KEEP_BALL_DIST+50)**2)**0.5)

whatWeDoAtThisRun: whatWeDoStates = whatWeDoStates.BothPlay

minErrAngleForRotateWithBall: int = 5