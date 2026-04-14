from enum import Enum
from bridge import const
from math import asin
from environment.setup_environment import get_from_env

class whatWeDoStates(Enum):
    """Класс с типо запускаемого нами кода"""

    Play = 0
    BothPlay = 1
    TestPass = 2
    SimpleTest = 3
    TestRotateWithBall = 4
    NewIsBallInTest = 5
    TestGK = 6

minDistForOpeningForPass = 700

idFirstAttacker: int = 6
idSecondAttacker: int = 4

timerForRotate = 0.5/2#sec
timerForHoldBall = 3#sec
constForTimerWeTryDoPass = 3#sec

velRotateWithBall = 0.4#rad/sec
if const.IS_SIMULATOR_USED:
    velRotateWithBall *= 5

maxDistForScore = 1500
minDistForScorePenalty = 2000
maxDistToChangeModeForScroreBallInPenalty = 350

distToBallForGoOutGK = 600
velBallForGoOutGK = 400
distToStopForGoOutGK = 200

spaceFromEdgedForFindingPointsForScore = 100

dForCatchBall = 10

distBetweenRsInWall = 250
angleBetweenRsInWall = asin((distBetweenRsInWall/2)/((distBetweenRsInWall/2)**2+(const.KEEP_BALL_DIST+50)**2)**0.5)

whatWeDoAtThisRun: whatWeDoStates = whatWeDoStates.TestPass

useDebug = get_from_env("DEBUG_MODE", bool)
if not const.IS_SIMULATOR_USED and not useDebug:
    whatWeDoAtThisRun = whatWeDoStates.Play#DONT TOUCH!!!!!!!!

minErrAngleForRotateWithBall: float = 2.5