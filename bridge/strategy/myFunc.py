import math  # type: ignore
from enum import Enum
from typing import Optional  # type: ignore

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore

# from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, DribblerActions  # type: ignore


class whatWeDoStates(Enum):
    """Класс с типо запускаемого нами кода"""

    Play = 0
    BothPlay = 1
    TestPass = 2
    SimpleTest = 3
    TestRotateWithBall = 4


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


def getKoefForEnemysRobotR(ballPos: aux.Point, enemyRPos: aux.Point) -> float:
    if aux.dist(enemyRPos, ballPos) < 100:
        k = 1.0
    else:
        k = 0.75 + 0.15 * aux.dist(enemyRPos, ballPos) / const.ROBOT_R  # depend from distans from r to maybe pass point
    return k


# TODO do comments
def goToNearestScorePoint(
    field: fld.Field, actions: list[Optional[Action]], idFrom: int, idOtherAttacker: int | None
) -> None:
    # field.allies[idFrom].set_dribbler_speed(15)
    rCircle = 1100
    thisR = field.allies[idFrom]
    enemysGoalCenter = field.enemy_goal.center
    if idOtherAttacker != None:
        aimForLookPos = field.allies[idOtherAttacker].get_pos()
    else:
        aimForLookPos = enemysGoalCenter
    # field.strategy_image.draw_circle(enemysGoalCenter, (255, 255, 255), 1000)
    pointsForScore = []
    # if aux.is_point_inside_circle(thisR.get_pos(), enemysGoalCenter, rCircle+150):
    # vectFromCenterToR = (thisR.get_pos()-enemysGoalCenter)
    vectFromCenterToR = field.enemy_goal.eye_forw * rCircle
    # field.strategy_image.draw_line(vectFromCenterToR, enemysGoalCenter)
    for angel in range(-180, 180 + 1, 10):
        angelInRad = angel / 180 * math.pi
        maybeScorePoint = aux.rotate(vectFromCenterToR, angelInRad) + enemysGoalCenter
        argVectFromCenterToMaybeScorePoint = aux.wind_down_angle((maybeScorePoint - enemysGoalCenter).arg())
        if field.polarity == 1:
            # field.strategy_image.draw_circle(thisR.get_pos(), size_in_mms=500)

            # print(aux.wind_down_angle(maybeScorePoint.arg()))
            if -math.pi / 2 >= argVectFromCenterToMaybeScorePoint or argVectFromCenterToMaybeScorePoint >= math.pi / 2:
                continue
        else:
            if -math.pi / 2 <= argVectFromCenterToMaybeScorePoint <= math.pi / 2:
                continue
        field.strategy_image.draw_circle(maybeScorePoint)
        pointForScore = findPointForScore(field, maybeScorePoint)
        # field.strategy_image.draw_line(maybeScorePoint, enemysGoalCenter, (200, 0, 0), 100)
        # field.strategy_image.draw_line(pointForScore, enemysGoalCenter)
        if pointForScore != None:
            pointsForScore.append(maybeScorePoint)
            field.strategy_image.draw_circle(maybeScorePoint)
            # field.strategy_image.draw_line(maybeScorePoint, enemysGoalCenter, (200, 0, 0), 100)
    nearestScorePoint = aux.find_nearest_point(thisR.get_pos(), pointsForScore)
    field.strategy_image.draw_circle(nearestScorePoint, (0, 0, 255), 50)
    actions[idFrom] = Actions.GoToPoint(nearestScorePoint, (aimForLookPos - thisR.get_pos()).arg())
    # else:
    #     nearestPoint = aux.nearest_point_on_circle(thisR.get_pos(), enemysGoalCenter, rCircle)
    #     actions[idFrom] = Actions.GoToPoint(nearestPoint, (aimForLookPos-thisR.get_pos()).arg())


def filterPointsForPass(field: fld.Field, points: list[aux.Point], pointFromOpen: aux.Point) -> list[aux.Point]:
    filteredPointsForPass = []
    enemysR = field.active_enemies(True)
    ballPos = field.ball.get_pos()
    pointForScore = findPointForScore(field, ballPos)

    for maybePassPoint in points:
        rPreventPass = False
        for enemyR in enemysR:
            k = getKoefForEnemysRobotR(ballPos, enemyR.get_pos())
            # if len(aux.line_circle_intersect(ballPos, maybePassPoint, enemyR.get_pos(), const.ROBOT_R*k, "S")) > 0:
            if (
                aux.is_point_inside_circle(maybePassPoint, enemyR.get_pos(), const.ROBOT_R * k)
                or len(aux.line_circle_intersect(ballPos, maybePassPoint, enemyR.get_pos(), const.ROBOT_R * k, "S")) > 0
            ):
                """if this point dont near enemy r and line from ball to point dont intersected enemy robot's circle"""
                rPreventPass = True
                field.strategy_image.draw_circle(enemyR.get_pos(), (0, 255, 200), const.ROBOT_R * k)
                field.strategy_image.draw_circle(maybePassPoint, (0, 0, 0))
        # field.strategy_image.draw_line(points, ballPos, (200, 0, 0), 100)
        # field.strategy_image.draw_line(pointForScore, ballPos)
        distToEnemyHull = aux.dist(maybePassPoint, aux.nearest_point_in_poly(maybePassPoint, field.enemy_goal.hull))
        distToAllyHull = aux.dist(maybePassPoint, aux.nearest_point_in_poly(maybePassPoint, field.ally_goal.hull))
        isPointInHull = aux.is_point_inside_poly(maybePassPoint, field.hull)
        if pointForScore != None:
            """if we can do score, we check does this point prevent score"""
            clPoint = aux.closest_point_on_line(ballPos, pointForScore, maybePassPoint)
            thisPointPreventScore = aux.dist(clPoint, maybePassPoint) < 250
            isOurPathPreventScore = aux.get_line_intersection(pointFromOpen, maybePassPoint, ballPos, pointForScore) != None
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
            point doesnt prevent score, we can go at this point and our path doesnt intersect score line"""
            filteredPointsForPass.append(maybePassPoint)
            field.strategy_image.draw_circle(maybePassPoint, (0, 255, 0))
            # field.strategy_image.draw_line(points, ballPos, (200, 0, 0), 100)
    return filteredPointsForPass


def openForPass(field: fld.Field, idRWhichOpen: int, actions: list[Optional[Action]]) -> Optional[aux.Point]:
    ballPos = field.ball.get_pos()
    thisR = field.allies[idRWhichOpen]
    thisRPos = thisR.get_pos()
    maybePointsForOpening = []
    vectFromBallToR = thisRPos - ballPos
    nearestPointForOpening = None
    # vectForRotate =
    # step = 200
    # for i in range(step, int(vectFromBallToR.mag()), step):
    #     maybePointsForOpening.append(ballPos+(vectFromBallToRUnity*i))
    #     # field.strategy_image.draw_circle(ballPos+(vectFromBallToRUnity*i))

    if vectFromBallToR.mag() < 700:
        """if we try open for pass at dist < 700, we open for pass at dist 700"""
        vectFromBallToR = aux.UP * 700
    else:
        vectFromBallToR = aux.UP * vectFromBallToR.mag()
    isBallOnOurPartOfField = ballPos.x * field.polarity > 0

    for angel in range(-180, 180 + 1, 10):
        """add points on circle"""
        angelInRad = angel / 180 * math.pi
        point = aux.rotate(vectFromBallToR, angelInRad) + ballPos
        maybePointsForOpening.append(point)
        # field.strategy_image.draw_circle(point)
        # pointForScore = findPointForScore(field, maybePassPoint)

    pointsForOpening = filterPointsForPass(field, maybePointsForOpening, thisRPos)

    # print(pointsForOpening, field.ball.get_pos())
    if len(pointsForOpening) != 0:
        """if we can open for pass or take pass"""
        if isBallOnOurPartOfField:
            nearestPointForOpening = aux.find_nearest_point(thisRPos, pointsForOpening)
        else:
            nearestPointForOpening = aux.find_nearest_point(field.enemy_goal.center, pointsForOpening)
        """depend from side of field we go to different points"""
        field.strategy_image.draw_circle(nearestPointForOpening, (0, 0, 255), 50)
        # field.strategy_image.draw_line(ballPos, nearestPointForOpening, (0, 0, 0), 20)
        actions[idRWhichOpen] = Actions.GoToPoint(nearestPointForOpening, (ballPos - thisR.get_pos()).arg())

    return nearestPointForOpening


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
                # field.strategy_image.draw_circle(field.allies[const.GK].get_pos(), (0, 200, 255), 50)
                continue
            # for nearestR in ourRsSortedByDistToBall:
            maybePassPoint = nearestR.get_pos()

            for enemyR in enemys:
                k = getKoefForEnemysRobotR(ballPos, enemyR.get_pos())
                if k > 15:
                    k = 1
                if (
                    aux.dist(aux.closest_point_on_line(pointFrom, maybePassPoint, enemyR.get_pos()), enemyR.get_pos())
                    < const.ROBOT_R * k
                ):
                    break
            else:
                """if noone enemy r prevent our pass"""
                rToPass = nearestR
                pointToPass = maybePassPoint
        if rToPass == None:
            """if our r now isnt open"""
            openForPass(field, nearestR.r_id, actions)
    return (rToPass, pointToPass)


def doPassNearAllly(field: fld.Field, actions: list[Optional[Action]], idFrom: int = const.GK) -> int | None:
    points = field.active_allies(False)
    exclude = [idFrom]
    # pointFrom = field.allies[idFrom].get_pos()
    pointFrom = field.ball.get_pos()
    enemys = field.active_enemies(True)
    pointToPass = None
    rToPass = None

    if len(points) > 1:
        """if our rs on field, except GK"""
        if idFrom == const.GK:
            ourRsSortedByDistToBall = fld.find_nearest_robots(pointFrom, points)
            # ourRsSortedByDistToBall = ourRsSortedByDistToBall.remove(field.allies[idFrom])
        else:
            ourRsSortedByDistToBall = [fld.find_nearest_robot(pointFrom, points, avoid=exclude)]

        rToPass, pointToPass = getPointToPassAndRToPass(field, actions, ourRsSortedByDistToBall, enemys, pointFrom, idFrom)
        ourRsSortedByDistToBall[0].r_id

        if pointToPass != None and rToPass != None:
            """if enemy r dont prevent pass"""
            field.strategy_image.send_telemetry("status pass", "have point")
            # field.strategy_image.draw_line(pointFrom, pointToPass, color=(255, 0, 0))
            # field.strategy_image.draw_circle(pointToPass, color=(255, 0, 0), size_in_mms=1000)
            pointToOpenForPass = openForPass(field, rToPass.r_id, actions)
            if (rToPass.get_vel()).mag() > 100 and not pointToOpenForPass is None:
                """if this r moving, we must kick ball ahead"""
                """TODO maybe we do openForPass several times for one run - bad"""
                pointToPass = pointToOpenForPass
            actions[idFrom] = Actions.Kick(pointToPass, is_pass=True)# type: ignore
        else:
            """if enemy r prevent pass"""
            field.strategy_image.send_telemetry("status pass", "dont have straight pass point")
            # actions[1].
            if actions[ourRsSortedByDistToBall[0].r_id] is not None:
                """do pass ahead"""
                field.strategy_image.draw_line( field.ball.get_pos(), actions[ourRsSortedByDistToBall[0].r_id].target_pos, (150, 0, 255), 20)  # type:ignore
                actions[idFrom] = Actions.Kick(actions[ourRsSortedByDistToBall[0].r_id].target_pos, is_pass=False, is_upper=True)  # type:ignore
    if actions[idFrom] is None:
        """if this r now cant do pass"""
        actions[idFrom] = Actions.GoToPoint(
            field.allies[idFrom].get_pos(), (field.ball.get_pos() - field.allies[idFrom].get_pos()).arg()
        )  # TODO change koef for slow rotate with ball
    if rToPass != None:
        return rToPass.r_id
    else:
        return None
    return ourRsSortedByDistToBall[0].r_id
    # else: # consider this case


# TODO do comments
def GK(
    field: fld.Field, actions: list[Optional[Action]], oldGKState: str | None
) -> str:  # TODO change string variable on enum class
    GKState = None

    # field.allies[const.GK].set_dribbler_speed(0)

    oldBallPos = field.ball_start_point
    ballPos = field.ball.get_pos()
    GKPos = field.allies[const.GK].get_pos()
    enenmies = field.active_enemies(False).copy()
    allies = field.active_allies(True).copy()
    allR = enenmies + allies

    nearestEnemyRToBall = fld.find_nearest_robot(ballPos, field.active_enemies(False))
    nearestRToBall = fld.find_nearest_robot(ballPos, allR)
    # field.strategy_image.draw_circle(nearestRToBall.get_pos(), color=(0, 255, 0), size_in_mms=50)
    enemyRGrabBall = field.is_ball_in(nearestEnemyRToBall)

    if nearestRToBall == field.allies[const.GK] and oldGKState != "Intersept":
        # field.strategy_image.send_telemetry("GK State", "Pass")
        GKState = "Pass"
        doPassNearAllly(field, actions)
    elif field.is_ball_moves_to_goal() and not enemyRGrabBall:
        if not aux.is_point_on_line(GKPos, oldBallPos, ballPos, "R"):
            interseptBallPoint = aux.closest_point_on_line(oldBallPos, ballPos, GKPos, "R")
            field.strategy_image.draw_circle(interseptBallPoint, color=(255, 0, 0), size_in_mms=50)
            # field.strategy_image.draw_line(GKPos, interseptBallPoint, color=(0, 0, 200), size_in_pixels=20)
            if interseptBallPoint != ballPos:
                # field.strategy_image.send_telemetry("GK State", "Intersept")
                GKState = "Intersept"
                """ intersept ball"""
                actions[const.GK] = Actions.GoToPointIgnore(interseptBallPoint, (ballPos - interseptBallPoint).arg())
            else:
                GKState = "Grab ball"
                # field.strategy_image.send_telemetry("GK State", "Grab ball")
                """grab ball if it maybe in hull and we cant intersept him"""
                actions[const.GK] = Actions.BallGrab((ballPos - GKPos).arg())
        else:
            GKState = "Pass interstpted ball"
            # field.strategy_image.send_telemetry("GK State", "Pass interstpted ball")
            """grab intersepted ball and pass nearly ally"""
            # actions[const.GK] = Actions.BallGrab((ballPos-GKPos).arg)
            doPassNearAllly(field, actions)
    # elif field.is_ball_in(field.allies[const.GK]):
    #     """"""
    #     doPassNearAllly(field, actions)
    elif aux.is_point_inside_poly(ballPos, field.ally_goal.hull):
        GKState = "Knock out ball"
        # field.strategy_image.send_telemetry("GK State", "Knock out ball")
        """knock out the ball from hull"""
        doPassNearAllly(field, actions)
    else:
        GKState = "block maybe kick"
        # field.strategy_image.send_telemetry("GK State", "Block maybe kick")
        # if enemyRGrabBall:
        """block maybe kick"""
        # pointForGK = aux.nearest_point_on_poly(ballPos, field.ally_goal.hull)
        mostLikelyPointForScore = aux.closest_point_on_line(field.ally_goal.up, field.ally_goal.down, ballPos)
        pointForGK = aux.segment_poly_intersect(ballPos, mostLikelyPointForScore, field.ally_goal.hull)
        if pointForGK != None:
            field.strategy_image.draw_circle(pointForGK, color=(0, 0, 255), size_in_mms=50)
            # print(pointForGK)
            actions[const.GK] = Actions.GoToPointIgnore(pointForGK, (ballPos - GKPos).arg()).compose(DribblerActions.SetDribblerSpeed(15))
            # field.allies[const.GK].set_dribbler_speed(15)  # TODO check in real we need it?
        else:
            """err"""
            print("ERROR IN GK")
    field.strategy_image.send_telemetry("GK State", GKState)
    return GKState


def findPointForScore(
    field: fld.Field, pointFrom: None | aux.Point = None, draw: bool = True, k: float | None = None, reverse: bool = False
) -> aux.Point | None:  # TODO do comments
    if pointFrom == None:
        pointFrom = field.ball.get_pos()
    qPoint = 8
    qPoint += 2
    ballPos = field.ball.get_pos()
    if not reverse:
        d = field.enemy_goal.up.y - field.enemy_goal.down.y
        points = [aux.Point(field.enemy_goal.up.x, field.enemy_goal.up.y - (d / qPoint * i)) for i in range(1, qPoint)]
        enemys = field.active_enemies(True)
    else:
        d = field.ally_goal.up.y - field.ally_goal.down.y
        points = [aux.Point(field.ally_goal.up.x, field.ally_goal.up.y - (d / qPoint * i)) for i in range(1, qPoint)]
        enemys = field.active_allies(True)
    # enemys = field.enemies
    closest = None
    min_dist = 10e10
    for _, point in enumerate(points):
        if aux.dist(pointFrom, point) < min_dist:
            if len(enemys) != 0:
                for enemyR in enemys:
                    if k == None:
                        k = getKoefForEnemysRobotR(ballPos, enemyR.get_pos())
                    if len(aux.line_circle_intersect(pointFrom, point, enemyR.get_pos(), const.ROBOT_R * k, "S")) != 0:
                        break
                else:
                    """if no one enemy r prevent this kick"""
                    min_dist = aux.dist(pointFrom, point)
                    closest = point
            else:
                """if no one enemy r prevent this kick"""
                min_dist = aux.dist(pointFrom, point)
                closest = point
    if draw:
        if closest != None:
            field.strategy_image.draw_line(pointFrom, closest, color=(0, 255, 0))
        else:
            field.strategy_image.draw_circle(pointFrom, color=(0, 0, 0), size_in_mms=50)
    return closest
