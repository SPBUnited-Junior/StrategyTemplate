from enum import Enum
from bridge import const
from math import asin
from environment.setup_environment import get_from_env
from bridge.auxiliary import aux, fld, rbt  # type: ignore


class whatWeDoStates(Enum):
    """Класс с типо запускаемого нами кода"""

    Play = 0
    BothPlay = 1
    TestPass = 2
    SimpleTest = 3
    TestRotateWithBall = 4
    NewIsBallInTest = 5
    TestGK = 6


distToBlockEnemyPass = 300

minDistForOpeningForPass = 700

idFirstAttacker: int = 1
idSecondAttacker: int = 2

timerForRotate = 0.5 / 2  # sec
timerForHoldBallForMyIsBallIn = 1.5 / 1.5  # 3#sec
constForTimerWeTryDoPass = 3  # sec

velRotateWithBall = 0.4  # rad/sec
if const.IS_SIMULATOR_USED:
    velRotateWithBall *= 5

maxDistForScore = 1500
minDistForScorePenalty = 1250
maxDistToChangeModeForScroreBallInPenalty = 700

distToBallForGoOutGK = 1000
distToBallForGoOutGKForPenalty = 1500
velBallForGoOutGK = 400
distToStopForGoOutGK = 100

dForCatchBall = 10

distBetweenRsInWall = 250
angleBetweenRsInWall = asin(
    (distBetweenRsInWall / 2)
    / ((distBetweenRsInWall / 2) ** 2 + (const.KEEP_BALL_DIST + 50) ** 2) ** 0.5
)

""""""
whatWeDoAtThisRun: whatWeDoStates = whatWeDoStates.TestGK
""""""

weUseDribbler = False

if not weUseDribbler:
    timerForHoldBallForMyIsBallIn = 0
    timerForRotate = 0

spaceFromEdgedForFindingPointsForScore = 100 + weUseDribbler * 25
spaceFromEdgedForFindingPointsForScorePenalty = 175 + weUseDribbler * 25

# useDebug = get_from_env("DEBUG_MODE", bool)
# if not const.IS_SIMULATOR_USED and not useDebug:
#     whatWeDoAtThisRun = whatWeDoStates.Play#DONT TOUCH!!!!!!!!

lowerEdgeForMinAngleErr = 0.25
upperEdgeForMinAngleErr = 2.5 * 2
koeffForCalculatingMinAngleErr = (upperEdgeForMinAngleErr - lowerEdgeForMinAngleErr) / (
    ((const.FIELD_DX) ** 2 + (const.FIELD_DY) ** 2) ** 0.5 - minDistForOpeningForPass
)


def calculateMinAngleErrForRotate(distToPointForPass: float) -> float:
    minAngleErrForRotate = aux.minmax(
        distToPointForPass * koeffForCalculatingMinAngleErr,
        lowerEdgeForMinAngleErr,
        upperEdgeForMinAngleErr,
    )
    # print("minAngleErrForRotate =", minAngleErrForRotate)
    return minAngleErrForRotate
