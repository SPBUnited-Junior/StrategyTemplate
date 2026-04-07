from typing import Optional
import math  # type: ignore

from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.strategy.myConst import angleBetweenRsInWall, idFirstAttacker, idSecondAttacker
from bridge import const
from bridge.router.base_actions import Actions, Action  # type: ignore

def getAngleFromOurGoalToEnemysGoal(field: fld.Field)->float:
    return math.pi * (1+field.polarity)/2

def isBallOnOurPartOfField(field: fld.Field)-> bool:
    return field.ball.get_pos().x * field.polarity > 0

def buildWallInFrontOfBall(field: fld.Field, actions: list[Optional[Action]])->None:
    if not isBallOnOurPartOfField(field):
        angle = angleBetweenRsInWall
    else:
        nowDistBetweenRsInWall = 110
        angle = math.asin((nowDistBetweenRsInWall/2)/((nowDistBetweenRsInWall/2)**2+(const.KEEP_BALL_DIST+50)**2)**0.5)

    vectFromBallToCenter = field.ally_goal.center-field.ball.get_pos()
    angleFromOurGoalToEnemysGoal = getAngleFromOurGoalToEnemysGoal(field)
    if field.allies[idFirstAttacker].is_used() and field.allies[idSecondAttacker].is_used():
        point1 = field.ball.get_pos()+aux.rotate(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50), angle)
        point2 = field.ball.get_pos()+aux.rotate(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50), -angle)
        actions[idFirstAttacker] = Actions.GoToPoint(point1, angleFromOurGoalToEnemysGoal)#TODO fix angle
        actions[idSecondAttacker] = Actions.GoToPoint(point2, angleFromOurGoalToEnemysGoal)#TODO fix angle
    elif field.allies[idFirstAttacker].is_used():#TODO check
        actions[idFirstAttacker] = Actions.GoToPoint(field.ball.get_pos()+(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50)), angleFromOurGoalToEnemysGoal)#TODO fix angle
    else:
        actions[idSecondAttacker] = Actions.GoToPoint(field.ball.get_pos()+(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50)), angleFromOurGoalToEnemysGoal)#TODO fix angle
