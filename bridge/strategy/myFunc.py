import math  # type: ignore
from typing import Optional  # type: ignore

from bridge import const
import bridge.strategy.myConst as myConst
from bridge.auxiliary import aux, fld, rbt  # type: ignore

# from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, DribblerActions  # type: ignore

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

def findBetterPointForOpen(center: aux.Point, points: list[aux.Point], centerEnemyGoal: aux.Point, field: fld.Field, draw: bool = True) -> aux.Point:
    closests = [(10.0**10, 10.0**10, aux.Point(0, 0)), (10.0**10, 10.0**10, aux.Point(1, 0)), (10.0**10, 10.0**10, aux.Point(0, 1)), (10.0**10, 10.0**10, aux.Point(0, 1))]
    minDist = 10.0**10

    a = 500

    for point in points:
        dist2r = aux.dist(center, point)

        dist2Goal = aux.dist(point, centerEnemyGoal)
        if dist2r < minDist:
            for i in range(4):                
                if dist2r - closests[i][0] < -a:
                    if i != 3:
                        for j in range(i+1, 4):
                            closests[j] = (closests[j-1][0], closests[j-1][1], closests[j-1][2])
                    closests[i] = (dist2r, dist2Goal, point)
                    break
                elif abs(dist2r - closests[i][0]) < a and dist2Goal < closests[i][1]:
                    if i != 3:
                        for j in range(i+1, 4):
                            closests[j] = (closests[j-1][0], closests[j-1][1], closests[j-1][2])
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

def canRDoScoreAndInWhatPoint(field: fld.Field, robotPos: Optional[aux.Point] = None, draw: bool = True) -> Optional[aux.Point]:
    if robotPos is None: robotPos = field.ball.get_pos()

    pointForScore = findPointForScore(field, robotPos)
    
    if draw and pointForScore is not None and aux.dist(robotPos, pointForScore) > myConst.maxDistForScore:
        field.strategy_image.draw_line(field.ball.get_pos(), pointForScore, (255, 0, 0), 12)

    if pointForScore is not None and aux.dist(robotPos, pointForScore) < myConst.maxDistForScore:
        return pointForScore
    else:
        return None


def nearest2BallEnemy(field: fld.Field, includeGK: bool = True) -> rbt.Robot:
    return fld.find_nearest_robot(field.ball.get_pos(), field.active_enemies(includeGK))

def isBallKickedToR(field: fld.Field, receiverRId: int, givingRId: int, check: bool = False, forEnemyes: bool = False) -> bool:
    ballPos = field.ball.get_pos()
    if not forEnemyes:
        receiverR = field.allies[receiverRId]
        givingR = field.allies[givingRId]
    else:
        givingR = field.enemies[givingRId]
        if receiverRId != -1:
            receiverR = field.enemies[receiverRId]
            receiverRPos = receiverR.get_pos()
        else:
            receiverRPos = aux.rotate(aux.RIGHT, givingR.get_angle())*(((const.FIELD_DX*2)**2+(const.FIELD_DY*2)**2)**0.5)

    givingRPos = givingR.get_pos()
    vectFormGivingPassToReceiver = receiverRPos-givingRPos
    vectNormalToVectFormGivingPassToReceiver = aux.rotate(vectFormGivingPassToReceiver, 90/180*math.pi)
    koefForErr = 1.5
    # newErrAngle = myConst.minErrAngleForRotateWithBall*koefForErr
    newErrAngle = myConst.minErrAngleForRotateWithBall
    pointPlusErr = aux.get_line_intersection(receiverRPos, receiverRPos+vectNormalToVectFormGivingPassToReceiver, givingRPos, givingRPos+aux.rotate(vectFormGivingPassToReceiver, newErrAngle/180*math.pi*koefForErr), "LL")
    
    if check:
        field.strategy_image.draw_line(receiverRPos, receiverRPos+vectNormalToVectFormGivingPassToReceiver)
        field.strategy_image.draw_line(receiverRPos, receiverRPos-vectNormalToVectFormGivingPassToReceiver)
        field.strategy_image.draw_line(givingRPos, givingRPos+aux.rotate(vectFormGivingPassToReceiver, newErrAngle/180*math.pi*koefForErr), (255, 0, 0), 20)
    
    pointMinusErr = aux.get_line_intersection(receiverRPos, receiverRPos+vectNormalToVectFormGivingPassToReceiver, givingRPos, givingRPos+aux.rotate(vectFormGivingPassToReceiver, -newErrAngle/180*math.pi*koefForErr), "LL")
    givingRPos = givingR.get_pos()
    
    if (check): field.strategy_image.send_telemetry("test", "in")
    
    if pointPlusErr is not None and pointMinusErr is not None:
        """check is prog work how it must"""
        if (check): field.strategy_image.send_telemetry("test", "if we have points")
        polygon1: list[aux.Point] = [givingRPos, pointMinusErr, pointPlusErr]
        field.strategy_image.draw_poly(polygon1, size_in_pixels=4)
        # if aux.is_point_inside_poly(ballPos, polygon1):
        if aux.dist(aux.nearest_point_in_poly(ballPos, polygon1), ballPos) < 100:
            """if ball moves in triangle in what it must move if it kicked"""
            if (check): field.strategy_image.send_telemetry("test", "if point in triangle")
            vectFromBallToReceiver = receiverRPos-ballPos
            if abs(field.ball.get_vel().arg()-vectFromBallToReceiver.arg())/math.pi*180 < myConst.minErrAngleForRotateWithBall*koefForErr:
                """if ball moves to point where he must be kicked"""
                if (check): field.strategy_image.send_telemetry("test", "if ball moving to point")
                if field.ball.get_vel().mag() > 1000:
                    """if he moves fast enough"""
                    if (check): field.strategy_image.send_telemetry("test", "if ball moving fast enough")
                    # print("True")
                    if (check): field.strategy_image.draw_circle(aux.Point(0, 0), size_in_mms=1000)
                    return True
    return False

def getKoefForEnemysRobotR(ballPos: aux.Point, enemyRPos: aux.Point) -> float:
    if aux.dist(enemyRPos, ballPos) < 100:
        k = 1.0
    else:
        k = 0.85 + 0.15 * aux.dist(enemyRPos, ballPos) / const.ROBOT_R  # depend from distans from r to maybe pass point
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


def openForPass(field: fld.Field, idRWhichOpen: int, actions: list[Optional[Action]]) -> tuple[Optional[aux.Point], int]:
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

    vectFromBallToR = thisRPos - ballPos
    pointOnCentreVectFromBallToR = vectFromBallToR/2
    if pointOnCentreVectFromBallToR.mag() < 700:
        """if we try open for pass at dist < 700, we open for pass at dist 700"""
        pointOnCentreVectFromBallToR = pointOnCentreVectFromBallToR.unity() * 700

    maybePointsForOpening.append(pointOnCentreVectFromBallToR+ballPos)

    pointsForOpening = filterPointsForPass(field, maybePointsForOpening, thisRPos)

    # print("len = ", len(pointsForOpening))
    if len(pointsForOpening) != 0:
        """if we can open for pass or take pass"""
        if isBallOnOurPartOfField:
            # nearestPointForOpening = aux.find_nearest_point(thisRPos, pointsForOpening)
            nearestPointForOpening = findBetterPointForOpen(thisRPos, pointsForOpening, field.enemy_goal.center, field)
        else:
            nearestPointForOpening = aux.find_nearest_point(field.enemy_goal.center, pointsForOpening)
            # nearestPointForOpening = findBetterPointForOpen(field.enemy_goal.center, pointsForOpening, field.enemy_goal.center, field)
        """depend from side of field we go to different points"""
        field.strategy_image.draw_circle(nearestPointForOpening, (0, 0, 255), 50)
        # field.strategy_image.draw_line(ballPos, nearestPointForOpening, (0, 0, 0), 20)
        actions[idRWhichOpen] = Actions.GoToPoint(nearestPointForOpening, (ballPos - thisR.get_pos()).arg())

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

    # print(len(points))
    if len(points) > 1 :
        """if our rs on field, except GK"""
        if idFrom == const.GK:
            ourRsSortedByDistToBall = fld.find_nearest_robots(pointFrom, points)
            # ourRsSortedByDistToBall = ourRsSortedByDistToBall.remove(field.allies[idFrom])
        else:
            ourRsSortedByDistToBall = [fld.find_nearest_robot(pointFrom, points, avoid=exclude)]
            # print(len(ourRsSortedByDistToBall))

        rToPass, pointToPass = getPointToPassAndRToPass(field, actions, ourRsSortedByDistToBall, enemys, pointFrom, idFrom)
        ourRsSortedByDistToBall[0].r_id

        if pointToPass != None and rToPass != None:
            """if enemy r dont prevent pass"""
            field.strategy_image.send_telemetry("status pass", "have point")
            # field.strategy_image.draw_line(pointFrom, pointToPass, color=(255, 0, 0))
            pointToOpenForPass, idRWhichOpen = openForPass(field, rToPass.r_id, actions)
            if not pointToOpenForPass is None:
                distToPointForPassFromRWhichOpen = (pointToOpenForPass-field.allies[idRWhichOpen].get_pos()).mag()
                distToPointForPassFromBall = (pointToOpenForPass-field.ball.get_pos()).mag()
                if (distToPointForPassFromRWhichOpen/rToPass.get_vel().mag()) < (distToPointForPassFromBall/const.MAX_SPEED_BALL)*1.5:
                    """if this r will arrive at point earlyer that ball"""
                    """TODO maybe we do openForPass several times for one run - bad"""
                    # field.strategy_image.draw_circle(pointToPass, color=(255, 255, 0), size_in_mms=1000)
                    pointToPass = pointToOpenForPass
                actions[idFrom] = Actions.DelayedSlowKick(pointToPass, is_pass=True)# type: ignore
        else:
            """if enemy r prevent pass"""
            field.strategy_image.send_telemetry("status pass", "dont have straight pass point")
            # actions[1].
            if actions[ourRsSortedByDistToBall[0].r_id] is not None:
                """do pass ahead"""
                idRWhichOpen = ourRsSortedByDistToBall[0].r_id
                try:
                    pointToOpenForPass = actions[idRWhichOpen].target_pos# type:ignore
                    field.strategy_image.draw_line( field.ball.get_pos(), pointToOpenForPass, (150, 0, 255), 20)  # type:ignore
                except:
                    pointToOpenForPass = field.allies[idRWhichOpen].get_pos()
                if pointToOpenForPass is not None and rToPass is not None:
                    distToPointForPassFromRWhichOpen = (pointToOpenForPass-field.allies[idRWhichOpen].get_pos()).mag()
                    distToPointForPassFromBall = (pointToOpenForPass-field.ball.get_pos()).mag()
                    if (distToPointForPassFromRWhichOpen/rToPass.get_vel().mag()) < (distToPointForPassFromBall/const.MAX_SPEED_BALL)*1.5:
                        """if this r will arrive at point earlyer that ball"""
                        """TODO maybe we do openForPass several times for one run - bad"""
                        field.strategy_image.draw_circle(pointToOpenForPass, color=(255, 255, 0), size_in_mms=1000)
                        actions[idFrom] = Actions.DelayedSlowKick(pointToOpenForPass, is_pass=False)  # type:ignore
    if actions[idFrom] is None:
        """if this r now cant do pass"""
        actions[idFrom] = Actions.GoToPoint(
            field.allies[idFrom].get_pos(), (field.ball.get_pos() - field.allies[idFrom].get_pos()).arg()
        )  # TODO change koef for slow rotate with ball
    if rToPass != None:
        # print("Have point for pass")
        return rToPass.r_id
    else:
        # print("None point for pass")
        return None
    return ourRsSortedByDistToBall[0].r_id
    # else: # consider this case


# TODO do comments
def GK(
    field: fld.Field, actions: list[Optional[Action]], oldGKState: str | None, pointFromBallKicked: Optional[aux.Point] = None, angleWithWhatBallKicked: Optional[float] = None, draw: bool = True
) -> str:  # TODO change string variable on enum class
    GKState = None

    a = aux.dist(field.allies[const.GK].get_pos(), field.ball.get_pos())
    field.strategy_image.send_telemetry("dist", str(a))

    # field.allies[const.GK].set_dribbler_speed(0)

    oldBallPos = field.ball_start_point
    ballPos = field.ball.get_pos()
    GKPos = field.allies[const.GK].get_pos()
    enenmies = field.active_enemies(False).copy()
    allies = field.active_allies(True).copy()
    allR = enenmies + allies

    if draw and pointFromBallKicked is not None and angleWithWhatBallKicked is not None:
        field.strategy_image.draw_line(oldBallPos, ballPos, (255, 0, 0))
        secondPointForLine = pointFromBallKicked+aux.rotate(aux.RIGHT, angleWithWhatBallKicked)
        field.strategy_image.draw_line(pointFromBallKicked, secondPointForLine, (0, 0, 255))


    nearestEnemyRToBall = fld.find_nearest_robot(ballPos, field.active_enemies(False))
    nearestRToBall = fld.find_nearest_robot(ballPos, allR)
    # field.strategy_image.draw_circle(nearestRToBall.get_pos(), color=(0, 255, 0), size_in_mms=50)
    enemyRGrabBall = field.is_ball_in(nearestEnemyRToBall)

    if nearestRToBall == field.allies[const.GK] and oldGKState != "Intersept":
        # field.strategy_image.send_telemetry("GK State", "Pass")
        GKState = "Pass"
        doPassNearAllly(field, actions)
    elif (field.ball.get_vel().mag() < myConst.velBallForGoOutGK and 
        aux.dist(aux.nearest_point_on_poly(ballPos, field.ally_goal.hull), ballPos) < myConst.distToBallForGoOutGK and 
        not aux.is_point_inside_poly(ballPos, field.ally_goal.hull)):
        """if ball dangerously close to goal GK need to go out"""
        GKState = "go out"
        vectFromBallToGK = (GKPos-ballPos)
        actions[const.GK] = Actions.GoToPointIgnore((vectFromBallToGK.unity()*myConst.distToStopForGoOutGK)+ballPos, aux.rotate(vectFromBallToGK, math.pi).arg()).compose(DribblerActions.SetDribblerSpeed(15))
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
        if len(field.active_allies(False)) != 0:
            doPassNearAllly(field, actions)
        else:
            actions[const.GK] = Actions.DelayedSlowKick(field.enemy_goal.center, is_upper=True)
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
    field: fld.Field, pointFrom: None | aux.Point = None, draw: bool = False, OtherK: float | None = None, reverseGoal: bool = False, reverse: bool = False
) -> aux.Point | None:  # TODO do comments
    if pointFrom == None:
        pointFrom = field.ball.get_pos()
    qPoint = 8
    qPoint += 2
    ballPos = field.ball.get_pos()
    if not reverseGoal:
        d = field.enemy_goal.up.y - field.enemy_goal.down.y
        points = [aux.Point(field.enemy_goal.up.x, field.enemy_goal.up.y - (d / qPoint * i)) for i in range(1, qPoint)]
        enemys = field.active_enemies(True)
    else:
        d = field.ally_goal.up.y - field.ally_goal.down.y
        points = [aux.Point(field.ally_goal.up.x, field.ally_goal.up.y - (d / qPoint * i)) for i in range(1, qPoint)]
        enemys = field.active_allies(True)
    # enemys = [field.enemies[1]]
    closest = None
    

    
    if not reverse:
        min_dist = 10e10
        for point in points:
            if aux.dist(pointFrom, point) < min_dist:
                if len(enemys) != 0:
                    for enemyR in enemys:
                        if OtherK is None:
                            k = getKoefForEnemysRobotR(ballPos, enemyR.get_pos())
                        else:
                            k = OtherK
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
    else:
        max_dist = 0.0
        for point in points:
            if aux.dist(pointFrom, point) > max_dist:
                if len(enemys) != 0:
                    for enemyR in enemys:
                        if OtherK is None:
                            k = getKoefForEnemysRobotR(ballPos, enemyR.get_pos())
                        else:
                            k = OtherK
                        if len(aux.line_circle_intersect(pointFrom, point, enemyR.get_pos(), const.ROBOT_R * k, "S")) != 0:
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
            field.strategy_image.draw_line(pointFrom, closest, color=(0, 150, 0), size_in_pixels=5)
            for enemyR in enemys:
                if OtherK is None:
                    k = getKoefForEnemysRobotR(ballPos, enemyR.get_pos())
                else:
                    k = OtherK
                field.strategy_image.draw_circle(enemyR.get_pos(), (0, 255, 255), const.ROBOT_R*k)    
        else:
            field.strategy_image.draw_circle(pointFrom, color=(0, 0, 0), size_in_mms=50)
    return closest
