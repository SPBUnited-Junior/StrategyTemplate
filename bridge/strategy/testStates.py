from bridge.auxiliary import aux, fld, rbt  # type: ignore
from typing import Optional
from bridge.router.base_actions import Action, Actions, KickActions, DribblerActions  # type: ignore
from bridge import const
from math import pi
from time import time

import bridge.strategy.myConst as myConst
from bridge.strategy.ClassWithMyStaticVariables import ClassWithMyStaticVariables  # type: ignore
from bridge.strategy.myLogicFunc import (
    isBallOnOurPartOfField,
    gettingPass,
    GK,
    getStatusOfPassLogic,
    block2EnemyRs,
    blockEnemyR,
)
from bridge.strategy.myFunc import (
    doPassNearAllly,
    findPointForScore,
    openForPass,
    isBallKickedToR,
)


def simpleTest(
    staticVariables: ClassWithMyStaticVariables,
    field: fld.Field,
    actions: list[Optional[Action]],
) -> None:
    point_for_score: Optional[aux.Point] = findPointForScore(
        field, field.ball.get_pos(), reverse=True, draw=True
    )
    # field.strategy_image.draw_circle(point_for_score)
    # field.strategy_image.draw_circle(field.ally_goal.center_up)
    # field.strategy_image.draw_circle(field.ally_goal.center_down)
    # field.strategy_image.draw_circle(field.ally_goal.center_up, size_in_mms=myConst.distToBallForGoOutGK)
    # field.strategy_image.draw_circle(field.ally_goal.center_down, size_in_mms=myConst.distToBallForGoOutGK)
    # field.strategy_image.draw_line(aux.Point(field.ally_goal.center_up.x+myConst.distToBallForGoOutGK, field.ally_goal.center_up.y), aux.Point(field.ally_goal.center_down.x+myConst.distToBallForGoOutGK, field.ally_goal.center_down.y))
    # constt = 4
    # if staticVariables.PointFromBallKicked is None:
    #     staticVariables.PointFromBallKicked = aux.Point(-1000)
    # if staticVariables.TimerWeHoldBall == None:
    #      staticVariables.TimerWeHoldBall = time()
    # if time() - staticVariables.TimerWeHoldBall > constt:
    #     staticVariables.TimerWeHoldBall = time()
    #     if staticVariables.PointFromBallKicked.x == 1000:
    #         staticVariables.PointFromBallKicked = aux.Point(-1000)
    #     else:
    #         staticVariables.PointFromBallKicked = aux.Point(1000)
    # point = staticVariables.PointFromBallKicked
    # actions[5] = Actions.GoToPoint(aux.Point(point.x, point.y-400), 0)
    # actions[const.GK] = Actions.GoToPoint(aux.Point(point.x, point.y+400), 0)


def testPass(
    staticVariables: ClassWithMyStaticVariables,
    field: fld.Field,
    actions: list[Optional[Action]],
) -> None:
    actions[const.GK] = Actions.GoToPoint(aux.Point(const.FIELD_DX, const.FIELD_DY), 0)

    if field.ball.get_pos().y > 5000:
        print("!!!!!!!!!!!!!!!!!!!!!!!!")
    else:
        if (
            staticVariables.maxVelBall < field.ball.get_vel().mag()
            and field.ball.get_vel().mag() < 10000
        ):
            staticVariables.maxVelBall = field.ball.get_vel().mag()
            # print("staticVariables.maxVelBall =", staticVariables.maxVelBall)

        staticVariables.myIsBallInClass.updateTimerWeHoldBall(field)
        # print("ballVel =", field.ball.get_vel().mag())
        # field.strategy_image.send_telemetry("ballVel", str(field.ball.get_vel().mag()))

        ballPos = field.ball.get_pos()
        nearestR = fld.find_nearest_robot(ballPos, field.active_allies(False))
        oldIdDoPass = staticVariables.idDoPass
        oldIdGettingPass = staticVariables.idGettingPass

        if (
            staticVariables.idDoPass is None and staticVariables.idGettingPass is None
        ):  # FOR TEST
            staticVariables.idDoPass = nearestR.r_id

        for thisR in field.active_allies(False):
            idxThisR = thisR.r_id
            thisRPos = thisR.get_pos()
            otherAttackerR = field.allies[
                (idxThisR == staticVariables.idFirstAttacker)
                * staticVariables.idSecondAttacker
                + (idxThisR == staticVariables.idSecondAttacker)
                * staticVariables.idFirstAttacker
            ]
            idxOtherAttacker = otherAttackerR.r_id

            status = getStatusOfPassLogic(
                staticVariables, field, actions, idxThisR, idxOtherAttacker
            )
            if status is None:
                openForPass(field, thisR.r_id, actions)
            else:
                print(idxThisR, status)
        # if oldIdDoPass != staticVariables.idDoPass or oldIdGettingPass != staticVariables.idGettingPass:
        print(staticVariables.idDoPass, staticVariables.idGettingPass)
        # field.strategy_image.send_telemetry("ids:", "staticVariables.idDoPass" + str(staticVariables.idDoPass) + "staticVariables.idGettingPass:" + str(staticVariables.idGettingPass))


def testGK(
    staticVariables: ClassWithMyStaticVariables,
    field: fld.Field,
    actions: list[Optional[Action]],
) -> None:
    if field.ally_color == const.COLOR:
        staticVariables.GKLastState = GK(
            field,
            actions,
            staticVariables.GKLastState,
            staticVariables.PointFromBallKicked,
            staticVariables.AngleWithWhatBallKicked,
        )
        print(staticVariables.GKLastState)
    else:
        if (
            field.allies[staticVariables.idFirstAttacker].is_used()
            or field.allies[staticVariables.idSecondAttacker].is_used()
        ):
            if field.allies[staticVariables.idFirstAttacker].is_used():
                activeId = staticVariables.idFirstAttacker
            elif field.allies[staticVariables.idSecondAttacker].is_used():
                activeId = staticVariables.idSecondAttacker
            pointForScore = findPointForScore(field)
            if pointForScore is not None:
                actions[activeId] = Actions.DelayedSlowKick(pointForScore)
            else:
                print("havent points for score")
        else:
            print("HAVENT Rs")


def testRotateWithBall(
    staticVariables: ClassWithMyStaticVariables,
    field: fld.Field,
    actions: list[Optional[Action]],
) -> None:
    thisR = field.allies[staticVariables.idFirstAttacker]
    if (
        staticVariables.maxVelBall < field.ball.get_vel().mag()
        and field.ball.get_vel().mag() < 10000
    ):
        staticVariables.maxVelBall = field.ball.get_vel().mag()
    if field.is_ball_in(thisR):
        actions[staticVariables.idFirstAttacker] = Actions.DelayedSlowKick(
            field.enemy_goal.center
        )
    else:
        actions[staticVariables.idFirstAttacker] = Actions.BallGrab(pi / 2)


def newIsBallInTest(
    staticVariables: ClassWithMyStaticVariables,
    field: fld.Field,
    actions: list[Optional[Action]],
) -> None:
    thisR = field.allies[staticVariables.idFirstAttacker]
    dist2Ball = (thisR.get_pos() - field.ball.get_pos()).mag()
    angle2Ball = abs(
        aux.wind_down_angle(
            (field.ball.get_pos() - thisR.get_pos()).arg() - thisR.get_angle()
        )
    )
    print(
        round(dist2Ball),
        const.BALL_GRABBED_DIST,
        round(angle2Ball / pi * 180, 2),
        round(const.BALL_GRABBED_ANGLE / pi * 180),
    )
    print(
        int(dist2Ball < const.BALL_GRABBED_DIST),
        int(angle2Ball < const.BALL_GRABBED_ANGLE),
        int(
            dist2Ball < const.BALL_GRABBED_DIST
            and angle2Ball < const.BALL_GRABBED_ANGLE
        ),
    )
