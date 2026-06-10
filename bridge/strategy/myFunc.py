import math  # type: ignore
from time import time  # type: ignore
from typing import Optional  # type: ignore

from bridge import const
import bridge.strategy.myConst as myConst
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates

# from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, DribblerActions  # type: ignore
from bridge.strategy.ClassWithMyStaticVariables import ClassWithMyStaticVariables

# class State(Enum):#do GK
#     """Класс с состояниями игры"""

#     HALT = 0
#     TIMEOUT = 1
#     STOP = 2
#     PREPARE_KICKOFF = 3
#     BALL_PLACEMENT = 4
#     PREPARE_PENALTY = 5
#     KICKOFF = 6
#     FREE_KICK = 7
#     PENALTY = 8
#     RUN = 9


def isBallCatchedWeNotUseDribbler(
    field: fld.Field,
    receiverRId: int,
    givingRId: int,
    check: bool = True,
    forEnemyes: bool = False,
) -> bool:
    ballPos = field.ball.get_pos()
    receiverR = field.allies[receiverRId]
    receiverRPos = receiverR.get_pos()
    givingR = field.allies[givingRId]
    givingRPos = givingR.get_pos()

    vectFormReceiverPassToGiving = -receiverRPos + givingRPos
    vectNormalTovectFormReceiverPassToGiving = aux.rotate(
        vectFormReceiverPassToGiving, 90 / 180 * math.pi
    )
    koefForErr = 1.5
    newErrAngle = myConst.calculateMinAngleErrForRotate(
        aux.dist(receiverRPos, givingRPos)
    )

    pointPlusErr = aux.get_line_intersection(
        receiverRPos,
        receiverRPos + vectNormalTovectFormReceiverPassToGiving,
        givingRPos,
        givingRPos
        + aux.rotate(
            vectFormReceiverPassToGiving, newErrAngle / 180 * math.pi * koefForErr
        ),
        "LL",
    )
    pointMinusErr = aux.get_line_intersection(
        receiverRPos,
        receiverRPos + vectNormalTovectFormReceiverPassToGiving,
        givingRPos,
        givingRPos
        + aux.rotate(
            vectFormReceiverPassToGiving, -newErrAngle / 180 * math.pi * koefForErr
        ),
        "LL",
    )
    if pointPlusErr is not None and pointMinusErr is not None:
        polygon1: list[aux.Point] = [givingRPos, pointMinusErr, pointPlusErr]
        field.strategy_image.draw_poly(polygon1)

        return (
            aux.dist(aux.nearest_point_in_poly(ballPos, polygon1), ballPos) < 100
            and field.ball.get_vel().mag() < 300
        )

    return False


def isBallOnOurPartOfField(field: fld.Field) -> bool:
    return field.ball.get_pos().x * field.polarity > 0


def findBetterPointForOpen(
    center: aux.Point,
    points: list[aux.Point],
    centerEnemyGoal: aux.Point,
    field: fld.Field,
    draw: bool = True,
) -> aux.Point:
    closests = [
        (10.0**10, 10.0**10, aux.Point(0, 0)),
        (10.0**10, 10.0**10, aux.Point(1, 0)),
        (10.0**10, 10.0**10, aux.Point(0, 1)),
        (10.0**10, 10.0**10, aux.Point(0, 1)),
    ]
    minDist = 10.0**10

    delta = 500  # magic koef which we need for normal working programm

    for point in points:
        dist2r = aux.dist(center, point)

        dist2Goal = aux.dist(point, centerEnemyGoal)
        if dist2r < minDist:
            for i in range(4):
                if dist2r - closests[i][0] < -delta:
                    if i != 3:
                        for j in range(i + 1, 4):
                            closests[j] = (
                                closests[j - 1][0],
                                closests[j - 1][1],
                                closests[j - 1][2],
                            )
                    closests[i] = (dist2r, dist2Goal, point)
                    break
                elif (
                    abs(dist2r - closests[i][0]) < delta and dist2Goal < closests[i][1]
                ):
                    if i != 3:
                        for j in range(i + 1, 4):
                            closests[j] = (
                                closests[j - 1][0],
                                closests[j - 1][1],
                                closests[j - 1][2],
                            )
                    closests[i] = (dist2r, dist2Goal, point)
                    break
            minDist = closests[3][0]

    minDist2Goal = 10.0**10
    betterPoint = aux.Point(0, 0)

    for i in range(4):
        dist2Goal = closests[i][1]
        if dist2Goal < minDist2Goal:
            minDist2Goal = dist2Goal
            betterPoint = closests[i][2]
        if draw:
            field.strategy_image.draw_circle(closests[i][2], (255, 255, 255), 150)
            # print(closests)

    if draw:
        field.strategy_image.draw_circle(betterPoint, (0, 255, 0), 15)
    return betterPoint


def canRDoScoreAndInWhatPoint(
    field: fld.Field, robotPos: Optional[aux.Point] = None, draw: bool = True
) -> Optional[aux.Point]:
    if robotPos is None:
        robotPos = field.ball.get_pos()

    pointForScore = findPointForScore(field, robotPos)

    if (
        draw
        and pointForScore is not None
        and aux.dist(robotPos, pointForScore) > myConst.maxDistForScore
    ):
        field.strategy_image.draw_line(
            field.ball.get_pos(), pointForScore, (255, 0, 0), 12
        )

    if (
        pointForScore is not None
        and aux.dist(robotPos, pointForScore) < myConst.maxDistForScore
    ):
        return pointForScore
    else:
        return None


def nearest2BallEnemy(field: fld.Field, includeGK: bool = True) -> rbt.Robot:
    return fld.find_nearest_robot(field.ball.get_pos(), field.active_enemies(includeGK))


def isBallKickedToR(
    field: fld.Field,
    receiverRId: int,
    givingRId: int,
    check: bool = False,
    forEnemyes: bool = False,
) -> bool:
    ballPos = field.ball.get_pos()
    if not forEnemyes:
        receiverR = field.allies[receiverRId]
        receiverRPos = receiverR.get_pos()
        givingR = field.allies[givingRId]
        givingRPos = givingR.get_pos()
    else:
        givingR = field.enemies[givingRId]
        givingRPos = givingR.get_pos()
        if receiverRId != -1:
            receiverR = field.enemies[receiverRId]
            receiverRPos = receiverR.get_pos()
        else:
            receiverRPos = (
                aux.rotate(aux.RIGHT, givingR.get_angle())
                * (((const.FIELD_DX * 2) ** 2 + (const.FIELD_DY * 2) ** 2) ** 0.5)
                + givingRPos
            )

    vectFormGivingPassToReceiver = receiverRPos - givingRPos
    vectNormalToVectFormGivingPassToReceiver = aux.rotate(
        vectFormGivingPassToReceiver, 90 / 180 * math.pi
    )
    koefForErr = 1.5
    if not myConst.weUseDribbler:
        koefForErr = 4.5
    newErrAngle = myConst.calculateMinAngleErrForRotate(
        aux.dist(receiverRPos, givingRPos)
    )
    pointPlusErr = aux.get_line_intersection(
        receiverRPos,
        receiverRPos + vectNormalToVectFormGivingPassToReceiver,
        givingRPos,
        givingRPos
        + aux.rotate(
            vectFormGivingPassToReceiver, newErrAngle / 180 * math.pi * koefForErr
        ),
        "LL",
    )

    if check:
        field.strategy_image.draw_line(
            receiverRPos, receiverRPos + vectNormalToVectFormGivingPassToReceiver
        )
        field.strategy_image.draw_line(
            receiverRPos, receiverRPos - vectNormalToVectFormGivingPassToReceiver
        )
        field.strategy_image.draw_line(
            givingRPos,
            givingRPos
            + aux.rotate(
                vectFormGivingPassToReceiver, newErrAngle / 180 * math.pi * koefForErr
            ),
            (255, 0, 0),
            20,
        )

    pointMinusErr = aux.get_line_intersection(
        receiverRPos,
        receiverRPos + vectNormalToVectFormGivingPassToReceiver,
        givingRPos,
        givingRPos
        + aux.rotate(
            vectFormGivingPassToReceiver, -newErrAngle / 180 * math.pi * koefForErr
        ),
        "LL",
    )
    givingRPos = givingR.get_pos()

    if check:
        field.strategy_image.send_telemetry("test", "in")

    if pointPlusErr is not None and pointMinusErr is not None:
        """check is prog work how it must"""
        if check:
            field.strategy_image.send_telemetry("test", "if we have points")
        polygon1: list[aux.Point] = [givingRPos, pointMinusErr, pointPlusErr]

        if aux.dist(aux.nearest_point_in_poly(ballPos, polygon1), ballPos) < 100:
            """if ball moves in triangle in what it must move if it kicked"""
            if check:
                field.strategy_image.send_telemetry("test", "if point in triangle")
            vectFromBallToReceiver = receiverRPos - ballPos

            if (
                abs(field.ball.get_vel().arg() - vectFromBallToReceiver.arg())
                / math.pi
                * 180
                < newErrAngle * koefForErr
            ):
                """if ball moves to point where he must be kicked"""
                if check:
                    field.strategy_image.send_telemetry(
                        "test", "if ball moving to point"
                    )

                if field.ball.get_vel().mag() > 300:
                    """if he moves fast enough"""
                    if check:
                        field.strategy_image.send_telemetry(
                            "test", "if ball moving fast enough"
                        )
                    if check:
                        field.strategy_image.draw_circle(
                            aux.Point(0, 0), size_in_mms=1000
                        )
                    return True
    return False


def getKoefForEnemysRobotR(ballPos: aux.Point, enemyRPos: aux.Point) -> float:
    if aux.dist(enemyRPos, ballPos) < 100:
        k = 1.0
    else:
        k = (
            0.85 + 0.15 * aux.dist(enemyRPos, ballPos) / const.ROBOT_R
        )  # depend from distans from r to maybe pass point
    return k


# TODO do comments
def goToNearestScorePoint(
    field: fld.Field,
    actions: list[Optional[Action]],
    idFrom: int,
    idOtherAttacker: int | None,
) -> None:
    rCircle = 1100
    thisR = field.allies[idFrom]
    enemysGoalCenter = field.enemy_goal.center
    if idOtherAttacker != None:
        aimForLookPos = field.allies[idOtherAttacker].get_pos()
    else:
        aimForLookPos = enemysGoalCenter
    pointsForScore = []
    vectFromCenterToR = field.enemy_goal.eye_forw * rCircle
    delta = 180 - ((not myConst.weUseDribbler) * 45)
    print("delta", delta)
    for angel in range(-delta, delta + 1, 10):
        angelInRad = angel / 180 * math.pi
        maybeScorePoint = aux.rotate(vectFromCenterToR, angelInRad) + enemysGoalCenter
        argVectFromCenterToMaybeScorePoint = aux.wind_down_angle(
            (maybeScorePoint - enemysGoalCenter).arg()
        )
        if field.polarity == 1:
            if (
                -math.pi / 2 >= argVectFromCenterToMaybeScorePoint
                or argVectFromCenterToMaybeScorePoint >= math.pi / 2
            ):
                continue
        else:
            if -math.pi / 2 <= argVectFromCenterToMaybeScorePoint <= math.pi / 2:
                continue
        field.strategy_image.draw_circle(maybeScorePoint)
        pointForScore = findPointForScore(field, maybeScorePoint)
        if pointForScore != None:
            pointsForScore.append(maybeScorePoint)
            field.strategy_image.draw_circle(maybeScorePoint)
    nearestScorePoint = aux.find_nearest_point(thisR.get_pos(), pointsForScore)
    field.strategy_image.draw_circle(nearestScorePoint, (0, 0, 255), 50)
    actions[idFrom] = Actions.GoToPoint(
        nearestScorePoint, (aimForLookPos - thisR.get_pos()).arg()
    )


def filterPointsForPass(
    field: fld.Field, points: list[aux.Point], pointFromOpen: aux.Point
) -> list[aux.Point]:
    filteredPointsForPass = []
    enemysR = field.active_enemies(True)
    ballPos = field.ball.get_pos()
    pointForScore = findPointForScore(field, ballPos)

    for maybePassPoint in points:
        rPreventPass = False
        for enemyR in enemysR:
            k = getKoefForEnemysRobotR(ballPos, enemyR.get_pos())

            if (
                aux.is_point_inside_circle(
                    maybePassPoint, enemyR.get_pos(), const.ROBOT_R * k
                )
                or len(
                    aux.line_circle_intersect(
                        ballPos,
                        maybePassPoint,
                        enemyR.get_pos(),
                        const.ROBOT_R * k,
                        "S",
                    )
                )
                > 0
            ):
                """if this point dont near enemy r and line from ball to point dont intersected enemy robot's circle"""
                rPreventPass = True
                field.strategy_image.draw_circle(
                    enemyR.get_pos(), (0, 255, 200), const.ROBOT_R * k
                )
                field.strategy_image.draw_circle(maybePassPoint, (0, 0, 0))
        distToEnemyHull = aux.dist(
            maybePassPoint,
            aux.nearest_point_in_poly(maybePassPoint, field.enemy_goal.hull),
        )
        distToAllyHull = aux.dist(
            maybePassPoint,
            aux.nearest_point_in_poly(maybePassPoint, field.ally_goal.hull),
        )
        isPointInHull = aux.is_point_inside_poly(maybePassPoint, field.hull)

        if pointForScore != None:
            """if we can do score, we check does this point prevent score"""
            clPoint = aux.closest_point_on_line(ballPos, pointForScore, maybePassPoint)
            thisPointPreventScore = aux.dist(clPoint, maybePassPoint) < 250
            isOurPathPreventScore = (
                aux.get_line_intersection(
                    pointFromOpen, maybePassPoint, ballPos, pointForScore
                )
                != None
            )

            if thisPointPreventScore:
                field.strategy_image.draw_circle(maybePassPoint, (200, 0, 255))
            if isOurPathPreventScore and isPointInHull:
                field.strategy_image.draw_circle(maybePassPoint, (255, 200, 0))
        else:
            """if we cant do score"""
            isOurPathPreventScore = False
            thisPointPreventScore = False
        if (
            rPreventPass == False
            and isPointInHull
            and distToEnemyHull > 150
            and distToAllyHull > 150
            and not thisPointPreventScore
            and not isOurPathPreventScore
        ):
            """if enemy r doesnt prevent pass, point in hull, point on dist from ally's or enemy's hull 150 and
            point doesnt prevent score, we can go at this point and our path doesnt intersect score line
            """
            filteredPointsForPass.append(maybePassPoint)
            field.strategy_image.draw_circle(maybePassPoint, (0, 255, 0))
    return filteredPointsForPass


def openForPass(
    field: fld.Field, idRWhichOpen: int, actions: list[Optional[Action]]
) -> tuple[Optional[aux.Point], int]:
    ballPos = field.ball.get_pos()
    thisR = field.allies[idRWhichOpen]
    thisRPos = thisR.get_pos()
    maybePointsForOpening = []
    vectFromBallToR = thisRPos - ballPos
    nearestPointForOpening = None

    if vectFromBallToR.mag() < myConst.minDistForOpeningForPass:
        """if we try open for pass at dist < minDistForOpeningForPass, we open for pass at dist minDistForOpeningForPass"""
        newVect = field.enemy_goal.eye_forw * myConst.minDistForOpeningForPass
    elif vectFromBallToR.mag() > myConst.maxDistForOpeningForPass:
        newVect = field.enemy_goal.eye_forw * myConst.maxDistForOpeningForPass
    else:
        newVect = field.enemy_goal.eye_forw * vectFromBallToR.mag()

    delta = 180 - ((not myConst.weUseDribbler) * 45)
    for angel in range(-delta, delta + 1, 10):
        """add points on circle"""
        angelInRad = angel / 180 * math.pi
        point = aux.rotate(newVect, angelInRad) + ballPos
        maybePointsForOpening.append(point)

    pointOnCentreVectFromBallToR = vectFromBallToR / 2
    if pointOnCentreVectFromBallToR.mag() < myConst.minDistForOpeningForPass:
        """if we try open for pass at dist < minDistForOpeningForPass, we open for pass at dist minDistForOpeningForPass"""
        pointOnCentreVectFromBallToR = (
            pointOnCentreVectFromBallToR.unity() * myConst.minDistForOpeningForPass
        )

    # maybePointsForOpening.append(pointOnCentreVectFromBallToR + ballPos)

    pointsForOpening = filterPointsForPass(field, maybePointsForOpening, thisRPos)

    if len(pointsForOpening) != 0:
        """if we can open for pass or take pass"""
        if isBallOnOurPartOfField(field):
            nearestPointForOpening = findBetterPointForOpen(
                thisRPos, pointsForOpening, field.enemy_goal.center, field
            )
        else:
            nearestPointForOpening = aux.find_nearest_point(
                field.enemy_goal.center, pointsForOpening
            )
        """depend from side of field we go to different points"""
        field.strategy_image.draw_circle(nearestPointForOpening, (0, 0, 255), 50)
        actions[idRWhichOpen] = Actions.GoToPoint(
            nearestPointForOpening, (ballPos - thisR.get_pos()).arg()
        )

    return nearestPointForOpening, idRWhichOpen


def getPointToPassAndRToPass(
    field: fld.Field,
    actions: list[Optional[Action]],
    ourRsSortedByDistToBall: list[rbt.Robot],
    enemys: list[rbt.Robot],
    pointFrom: aux.Point,
    idFrom: int = const.GK,
) -> tuple[rbt.Robot | None, aux.Point | None]:
    rToPass: Optional[rbt.Robot] = None
    pointToPass = None
    ballPos = field.ball.get_pos()
    if len(enemys) == 0:
        """if enemy rs not on field"""
        rToPass = ourRsSortedByDistToBall[0]
        pointToPass = rToPass.get_pos()
    else:
        """if enemy rs on field"""
        for nearestR in ourRsSortedByDistToBall:
            if nearestR == field.allies[const.GK]:
                """we dont do pass to GK"""
                continue
            maybePassPoint = nearestR.get_pos()

            for enemyR in enemys:
                k = getKoefForEnemysRobotR(ballPos, enemyR.get_pos())
                if (
                    aux.dist(
                        aux.closest_point_on_line(
                            pointFrom, maybePassPoint, enemyR.get_pos()
                        ),
                        enemyR.get_pos(),
                    )
                    < const.ROBOT_R * k
                ):
                    """if enemy r prevent pass"""
                    break
            else:
                """if no one enemy r prevent our pass"""
                rToPass = nearestR
                pointToPass = maybePassPoint
                break
        if rToPass is None:
            """if our r now isnt open"""
            openForPass(field, nearestR.r_id, actions)
    return (rToPass, pointToPass)


def doPassNearAllly(
    field: fld.Field,
    actions: list[Optional[Action]],
    staticVariables: Optional[ClassWithMyStaticVariables] = None,
    idFrom: int = const.GK,
) -> int | None:
    points = field.active_allies(False)
    exclude = [idFrom]
    pointFrom = field.ball.get_pos()
    enemys = field.active_enemies(True)
    pointToPass = None
    rToPass = None

    if len(points) > 1:
        """if our rs on field, except GK"""
        if idFrom == const.GK:
            ourRsSortedByDistToBall = fld.find_nearest_robots(pointFrom, points)
        else:
            """we do not do pass to GK"""
            ourRsSortedByDistToBall = [
                fld.find_nearest_robot(pointFrom, points, avoid=exclude)
            ]

        rToPass, pointToPass = getPointToPassAndRToPass(
            field, actions, ourRsSortedByDistToBall, enemys, pointFrom, idFrom
        )

        if pointToPass != None and rToPass != None:
            """if enemy r dont prevent pass"""
            field.strategy_image.send_telemetry("do pass", "have point")
            pointToOpenForPass, idRWhichOpen = openForPass(field, rToPass.r_id, actions)
            if pointToOpenForPass is not None:
                """if code works how it must"""
                distToPointForPassFromRWhichOpen = (
                    pointToOpenForPass - field.allies[idRWhichOpen].get_pos()
                ).mag()
                distToPointForPassFromBall = (
                    pointToOpenForPass - field.ball.get_pos()
                ).mag()
                if (distToPointForPassFromRWhichOpen / rToPass.get_vel().mag()) < (
                    distToPointForPassFromBall / const.MAX_SPEED_BALL
                ) * 1.5:
                    """if this r will arrive at point earlyer that ball"""
                    """TODO maybe we do openForPass several times for one run - bad"""
                    pointToPass = pointToOpenForPass
                if idFrom != field.gk_id and staticVariables is not None:
                    """if we need delay before next pass"""
                    if (
                        time() - staticVariables.TimerFromLastPass
                        >= myConst.constDelayBeforeNextPass
                    ):
                        actions[idFrom] = Actions.DelayedSlowKick(
                            pointToPass, is_pass=True
                        )
                else:
                    actions[idFrom] = Actions.DelayedSlowKick(pointToPass, is_pass=True)

        else:
            """if enemy r prevent pass"""
            field.strategy_image.send_telemetry(
                "do pass", "dont have straight pass point"
            )
            if actions[ourRsSortedByDistToBall[0].r_id] is not None:
                """do pass ahead"""
                actions[idFrom] = Actions.DelayedSlowKick(
                    field.enemy_goal.center,
                    timer_for_rotate=0,
                    timerForHoldBallForMyIsBallIn=0,
                )
                """ TODO DONT WORK, NEED REWORK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"""
                """ TODO DONT WORK, NEED REWORK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"""
                """ TODO DONT WORK, NEED REWORK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"""
                # idRWhichOpen = ourRsSortedByDistToBall[0].r_id
                # pointToOpenForPass = None
                # try:
                #     maybePointToOpenForPass = actions[idRWhichOpen].target_pos# type:ignore
                #     if maybePointToOpenForPass is not None and aux.dist(maybePointToOpenForPass, field.allies[idRWhichOpen].get_pos()) < 50 and field.allies[idRWhichOpen].get_vel().mag() < 50:
                #         # print
                #         pointToOpenForPass = maybePointToOpenForPass
                #     field.strategy_image.draw_line( field.ball.get_pos(), pointToOpenForPass, (150, 0, 255), 20)  # type:ignore
                # except:
                #     pointToOpenForPass = field.allies[idRWhichOpen].get_pos()
                # if pointToOpenForPass is not None and rToPass is not None:
                #     """if code works how it must"""
                #     distToPointForPassFromRWhichOpen = (pointToOpenForPass-field.allies[idRWhichOpen].get_pos()).mag()
                #     distToPointForPassFromBall = (pointToOpenForPass-field.ball.get_pos()).mag()
                #     if (distToPointForPassFromRWhichOpen/rToPass.get_vel().mag()) < (distToPointForPassFromBall/const.MAX_SPEED_BALL)*1.5:
                #         """if this r will arrive at point earlyer that ball"""
                #         field.strategy_image.draw_circle(pointToOpenForPass, color=(255, 255, 0), size_in_mms=1000)
                #         actions[idFrom] = Actions.DelayedSlowKick(pointToOpenForPass, is_pass=True)  # type:ignore
    if actions[idFrom] is None:
        """if this r now cant do pass"""
        actions[idFrom] = Actions.GoToPoint(
            field.allies[idFrom].get_pos(),
            (field.ball.get_pos() - field.allies[idFrom].get_pos()).arg(),
        )
    if rToPass != None:
        """Have point for pass"""
        return rToPass.r_id
    else:
        """None point for pass"""
        return None


def findPointForScore(
    field: fld.Field,
    pointFrom: None | aux.Point = None,
    draw: bool = True,
    OtherK: float | None = None,
    reverseGoal: bool = False,
    reverse: bool = False,
) -> aux.Point | None:
    if pointFrom == None:
        pointFrom = field.ball.get_pos()
    spaceForFindingPoints = myConst.spaceFromEdgedForFindingPointsForScore
    if field.game_state == GameStates.PENALTY:
        """for penalty we need be more accurate"""
        if not reverseGoal:
            spaceForFindingPoints = (
                myConst.spaceFromEdgedForFindingPointsForScorePenalty
            )
        qPoint = 24
        OtherK = 1.1
        nearestGoalPoint = aux.closest_point_on_line(
            field.enemy_goal.up, field.enemy_goal.down, field.ball.get_pos()
        )
        if (
            aux.dist(field.ball.get_pos(), nearestGoalPoint)
            < myConst.maxDistToChangeModeForScroreBallInPenalty
        ):
            """if we so close we try to score goal in closest score point"""
            reverse = False
    else:
        qPoint = 8
    qPoint += 2
    ballPos = field.ball.get_pos()
    if not reverseGoal:
        """if we want calculate for enemy goal"""
        d = const.GOAL_DY - spaceForFindingPoints * 2
        points = [
            aux.Point(
                field.enemy_goal.up.x,
                max(field.enemy_goal.up.y, field.enemy_goal.down.y)
                - spaceForFindingPoints
                - (d / qPoint * i),
            )
            for i in range(1, qPoint)
        ]
        enemys = field.active_enemies(True)
    else:
        """if we want calculate for ally goal"""
        d = const.GOAL_DY - spaceForFindingPoints * 2
        points = [
            aux.Point(
                field.ally_goal.up.x,
                max(field.ally_goal.up.y, field.ally_goal.down.y)
                - spaceForFindingPoints
                - (d / qPoint * i),
            )
            for i in range(1, qPoint)
        ]
        enemys = field.active_allies(True)

    closest = None
    for point in points:
        field.strategy_image.draw_circle(point)

    if not reverse:
        """find closest score point"""
        min_dist = 10e10
        for point in points:
            if aux.dist(pointFrom, point) < min_dist:
                if len(enemys) != 0:
                    for enemyR in enemys:
                        if OtherK is None:
                            k = getKoefForEnemysRobotR(ballPos, enemyR.get_pos())
                        else:
                            k = OtherK
                        if (
                            len(
                                aux.line_circle_intersect(
                                    pointFrom,
                                    point,
                                    enemyR.get_pos(),
                                    const.ROBOT_R * k,
                                    "S",
                                )
                            )
                            != 0
                        ):
                            break
                    else:
                        """if no one enemy r prevent this kick"""
                        min_dist = aux.dist(pointFrom, point)
                        closest = point
                else:
                    """if no one enemy r prevent this kick"""
                    min_dist = aux.dist(pointFrom, point)
                    closest = point
    else:
        """find farthest score point"""
        max_dist = 0.0
        for point in points:
            if aux.dist(pointFrom, point) > max_dist:
                if len(enemys) != 0:
                    for enemyR in enemys:
                        if OtherK is None:
                            k = getKoefForEnemysRobotR(ballPos, enemyR.get_pos())
                        else:
                            k = OtherK
                        if (
                            len(
                                aux.line_circle_intersect(
                                    pointFrom,
                                    point,
                                    enemyR.get_pos(),
                                    const.ROBOT_R * k,
                                    "S",
                                )
                            )
                            != 0
                        ):
                            break
                    else:
                        """if no one enemy r prevent this kick"""
                        max_dist = aux.dist(pointFrom, point)
                        closest = point
                else:
                    """if no one enemy r prevent this kick"""
                    max_dist = aux.dist(pointFrom, point)
                    closest = point

    if draw:
        if closest != None:
            field.strategy_image.draw_line(
                pointFrom, closest, color=(0, 150, 0), size_in_pixels=15
            )
            for enemyR in enemys:
                if OtherK is None:
                    k = getKoefForEnemysRobotR(ballPos, enemyR.get_pos())
                else:
                    k = OtherK
                field.strategy_image.draw_circle(
                    enemyR.get_pos(), (0, 255, 255), const.ROBOT_R * k
                )
        else:
            field.strategy_image.draw_circle(pointFrom, color=(0, 0, 0), size_in_mms=50)
    return closest
