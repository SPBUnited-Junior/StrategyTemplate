from time import time  # type: ignore
from typing import Optional
import math  # type: ignore

from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.strategy.myConst import angleBetweenRsInWall, idFirstAttacker, idSecondAttacker
from bridge import const
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, DribblerActions  # type: ignore
import bridge.strategy.myConst as myConst

from bridge.strategy.ClassWithMyStaticVariables import ClassWithMyStaticVariables, GKStates  # type: ignore
    
from bridge.strategy.myFunc import (
    doPassNearAllly,
    findPointForScore,
    openForPass,
    isBallKickedToR,
    nearest2BallEnemy,
    canRDoScoreAndInWhatPoint,
    isBallOnOurPartOfField
)

def blockEnemyR(field: fld.Field, actions: list[Optional[Action]], idxThisR: int, posEnemyRForBlock: aux.Point, reverse: bool = False)->None:
    if not reverse:
        pointGo = aux.point_on_line(field.ball.get_pos(), posEnemyRForBlock, myConst.distToBlockEnemyPass)
    else:
        pointGo = aux.point_on_line(posEnemyRForBlock, field.ball.get_pos(), myConst.distToBlockEnemyPass)
    actions[idxThisR] = Actions.GoToPoint(pointGo, (field.allies[idxThisR].get_pos() - posEnemyRForBlock).arg()).compose(DribblerActions.SetDribblerSpeed(15))

def block2EnemyRs(staticVariables: ClassWithMyStaticVariables, field: fld.Field, actions: list[Optional[Action]], idxThisR: int, idxOtherAttacker: int, reverse: bool = True)->None:
    enemies = field.active_enemies(False)
    thisAttackerRPos = field.allies[idxThisR].get_pos()
    otherAttackerRPos = field.allies[idxOtherAttacker].get_pos()
    nearest2ThisREnemy = fld.find_nearest_robot(thisAttackerRPos, enemies)
    nearest2OtherREnemy = fld.find_nearest_robot(otherAttackerRPos, enemies)
    if nearest2ThisREnemy.r_id != nearest2OtherREnemy.r_id:
        blockEnemyR(field, actions, idxThisR, nearest2ThisREnemy.get_pos(), reverse)
        blockEnemyR(field, actions, idxOtherAttacker, nearest2OtherREnemy.get_pos(), reverse)
    else:
        blockEnemyR(field, actions, idFirstAttacker, enemies[0].get_pos(), reverse)
        blockEnemyR(field, actions, idSecondAttacker, enemies[1].get_pos(), reverse)

def updates(staticVariables: ClassWithMyStaticVariables, field: fld.Field, actions: list[Optional[Action]], showTimerPass: bool, showIdsPass: bool)->None:
    """update variables for succesfull launch of programm"""
    staticVariables.myIsBallInClass.updateTimerWeHoldBall(field)
    updateTimerAndIdWeTryDoPass(staticVariables, field, actions)
    updatePointAndAngleFromWhatBallKicked(staticVariables, field)
    
    if showIdsPass: field.strategy_image.send_telemetry("ids", str(staticVariables.idDoPass)+" "+str(staticVariables.idGettingPass))
    if showTimerPass:
        if staticVariables.TimeWeTryDoPass is not None:
            field.strategy_image.send_telemetry("timerPass", str(time()-staticVariables.TimeWeTryDoPass))
        else:
            field.strategy_image.send_telemetry("timerPass", str(None))

def getAngleFromOurGoalToEnemysGoal(field: fld.Field)->float:
    return math.pi * (1+field.polarity)/2

def buildWallInFrontOfBall(field: fld.Field, actions: list[Optional[Action]])->None:
    ballPos = field.ball.get_pos()
    if not isBallOnOurPartOfField(field):
        angle = angleBetweenRsInWall
    else:
        """if ball of our part of field, build wall without space"""
        nowDistBetweenRsInWall = 200
        angle = math.asin((nowDistBetweenRsInWall/2)/((nowDistBetweenRsInWall/2)**2+(const.KEEP_BALL_DIST+50)**2)**0.5)

    vectFromBallToCenter = field.ally_goal.center-ballPos
    angleFromOurGoalToEnemysGoal = getAngleFromOurGoalToEnemysGoal(field)
    if field.allies[idFirstAttacker].is_used() and field.allies[idSecondAttacker].is_used():
        point1 = ballPos+aux.rotate(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50), angle)
        point2 = field.ball.get_pos()+aux.rotate(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50), -angle)
        idNearestRToPoint1 = fld.find_nearest_robot(point1, field.active_allies(False)).r_id
        idOtherR = (idNearestRToPoint1!=idFirstAttacker)*idFirstAttacker+(idNearestRToPoint1!=idSecondAttacker)*idSecondAttacker#TODO bad
        actions[idNearestRToPoint1] = Actions.GoToPoint(point1, angleFromOurGoalToEnemysGoal)
        actions[idOtherR] = Actions.GoToPoint(point2, angleFromOurGoalToEnemysGoal)
    elif field.allies[idFirstAttacker].is_used():
        actions[idFirstAttacker] = Actions.GoToPoint(field.ball.get_pos()+(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50)), angleFromOurGoalToEnemysGoal)
    else:
        actions[idSecondAttacker] = Actions.GoToPoint(field.ball.get_pos()+(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50)), angleFromOurGoalToEnemysGoal)

def updatePointAndAngleFromWhatBallKicked(staticVariables: ClassWithMyStaticVariables, field: fld.Field) -> None:
    list = field.active_enemies(True)+field.active_allies(True)
    nearestRToBall = fld.find_nearest_robot(field.ball.get_pos(), list)
    if nearestRToBall in field.active_enemies(True):
        if isBallKickedToR(field, -1, nearestRToBall.r_id, forEnemyes=True):
            staticVariables.PointFromBallKicked = nearestRToBall.get_pos()
            staticVariables.AngleWithWhatBallKicked = nearestRToBall.get_angle()


def updateTimerAndIdWeTryDoPass(staticVariables: ClassWithMyStaticVariables, field: fld.Field, actions: list[Optional[Action]]) -> None:
    ballPos = field.ball.get_pos()
    nearestRToBall = fld.find_nearest_robot(ballPos, field.allies)

    if staticVariables.idDoPass is None:
        """if we dont try do or get pass"""
        staticVariables.TimeWeTryDoPass = None
    
    elif staticVariables.idGettingPass is None and staticVariables.TimeWeTryDoPass is None and staticVariables.myIsBallInClass.myIsBallIn(nearestRToBall):
        """if we try do pass, but do not done this yet"""
        staticVariables.TimeWeTryDoPass = time()

    elif aux.dist(ballPos, field.allies[staticVariables.idDoPass].get_pos()) > 100 and field.ball.get_vel().mag()<200 and staticVariables.idGettingPass is not None:
        """if r so far away for ball"""
        staticVariables.TimeWeTryDoPass = None
        staticVariables.idDoPass = None
        staticVariables.idGettingPass = None
    
    if staticVariables.TimeWeTryDoPass is not None and time()-staticVariables.TimeWeTryDoPass > staticVariables.constForTimerWeTryDoPass:
        """if we try do pass too long"""
        idOtherR =  None
        for r in field.active_allies(False):
            if r.r_id != staticVariables.idDoPass:
                idOtherR = r.r_id
        if idOtherR is not None and staticVariables.idDoPass is not None:
            if time()-staticVariables.TimeWeTryDoPass > staticVariables.constForTimerWeTryDoPass*2 and staticVariables.idDoPass != const.GK:
                actions[staticVariables.idDoPass] = Actions.DelayedSlowKick(nearest2BallEnemy(field).get_pos())
            else:
                actions[staticVariables.idDoPass] = Actions.DelayedSlowKick(field.allies[idOtherR].get_pos(), is_upper=True)
            if isBallKickedToR(field, idOtherR, staticVariables.idDoPass):
                staticVariables.TimeWeTryDoPass = None
                staticVariables.idDoPass = None
                staticVariables.idGettingPass = None

def updateTimerWeHoldBall(staticVariables: ClassWithMyStaticVariables, field: fld.Field) -> None:
    if staticVariables.TimerWeHoldBall is None:
        if any(field.is_ball_in(r) for r in field.active_allies(True)):
            staticVariables.TimerWeHoldBall = time()
    elif all(not field.is_ball_in(r) for r in field.active_allies(True)):
        staticVariables.TimerWeHoldBall = None

def doingPass(staticVariables: ClassWithMyStaticVariables, field: fld.Field, actions: list[Optional[Action]], idxThisR: int)->None:
    if staticVariables.idGettingPass is not None and staticVariables.idDoPass is not None:
        if isBallKickedToR(field, staticVariables.idGettingPass, staticVariables.idDoPass):
            """pass done"""
            staticVariables.idDoPass = None
        else:
            """pass in process"""
            if staticVariables.idGettingPass is None:
                pointForScore = canRDoScoreAndInWhatPoint(field)
                if pointForScore != None:
                    """if r can do score he must break pass"""
                    actions[idxThisR] = Actions.DelayedSlowKick(pointForScore)
                else:
                    staticVariables.idGettingPass = doPassNearAllly(field, actions, idxThisR)

            else:
                staticVariables.idGettingPass = doPassNearAllly(field, actions, idxThisR)
    else:
        """pass in process"""
        staticVariables.idGettingPass = doPassNearAllly(field, actions, idxThisR)

def gettingPass(staticVariables: ClassWithMyStaticVariables, field: fld.Field, actions: list[Optional[Action]], test: bool = True, sendTelemetry: bool = False) -> None:
    thisRId: Optional[int] = staticVariables.idGettingPass
    if thisRId is not None:
        print("in")
        thisR = field.allies[thisRId]
        thisRPos = thisR.get_pos()
        ballPos = field.ball.get_pos()

        if staticVariables.idDoPass is not None:
            """r not yet kick ball"""
            if test and not sendTelemetry: print("status pass:", "r not yet kick ball")
            if test and sendTelemetry: field.strategy_image.send_telemetry("status pass", "r not yet kick ball")

            if actions[thisRId] is None:
                openForPass(field, thisRId, actions)
            staticVariables.PointFromBallKicked = field.allies[staticVariables.idDoPass].get_pos()
            staticVariables.AngleWithWhatBallKicked = field.allies[staticVariables.idDoPass].get_angle()
        elif not staticVariables.myIsBallInClass.myIsBallIn(thisR):
            """if ball already kicked"""
            if test and not sendTelemetry: print("status pass:", "if ball already kicked")
            if test and sendTelemetry: field.strategy_image.send_telemetry("status pass", "if ball already kicked")

            if staticVariables.AngleWithWhatBallKicked is None or staticVariables.PointFromBallKicked is None:
                print("PROBLEM WITH POINT")
            else:
                secondPointForLine = staticVariables.PointFromBallKicked+aux.rotate(aux.RIGHT, staticVariables.AngleWithWhatBallKicked)
                vectFromBallToR = field.allies[thisRId].get_pos()-ballPos
                if not aux.is_point_on_line(thisRPos, staticVariables.PointFromBallKicked, secondPointForLine, "R"):
                    """if we not yet on line between ball and point from what it was kicked"""
                    interseptBallPoint = aux.closest_point_on_line(staticVariables.PointFromBallKicked, secondPointForLine, thisRPos, "R")
                    if abs((vectFromBallToR.arg()-field.ball.get_vel().arg())) < 5/180*math.pi:
                        print("Intersept")
                        if test: field.strategy_image.send_telemetry("status pass", "Intersept")
                        """ intersept ball"""
                        actions[thisRId] = Actions.GoToPointIgnore(interseptBallPoint, (ballPos - interseptBallPoint).arg())
                    else:
                        if test: field.strategy_image.send_telemetry("status pass", "Grab ball")
                        # actions[thisRId] = Actions.BallGrab((ballPos - thisRPos).arg())
                        actions[thisRId] = Actions.BallGrab(aux.wind_down_angle(vectFromBallToR.arg()+math.pi))
                        print("First")
                else:
                    # actions[thisRId] = Actions.BallGrab((field.ball.get_pos()-field.allies[thisRId].get_pos()).arg())
                    actions[thisRId] = Actions.CatchBall()
                    print("Second")
        elif staticVariables.myIsBallInClass.myIsBallIn(field.allies[thisRId]):
            """get pass"""
            if test and not sendTelemetry: print("status pass:", "get pass")
            if test and sendTelemetry: field.strategy_image.send_telemetry("status pass", "get pass")
            staticVariables.idGettingPass = None

        actionThisR = actions[thisRId]
        if actionThisR is not None:
            actions[thisRId] = actionThisR.compose(DribblerActions.SetDribblerSpeed(15))

def getStatusOfPassLogic(staticVariables: ClassWithMyStaticVariables, field: fld.Field, actions: list[Optional[Action]], idxThisR: int, idxOtherAttacker: int, test: bool = False) -> Optional[str]:
    thisR = field.allies[idxThisR]
    thisRPos = thisR.get_pos()
    otherAttackerR = field.allies[idxOtherAttacker]
    status = None
    if staticVariables.idDoPass == idxThisR and not staticVariables.myIsBallInClass.myIsBallIn(thisR):
        """if we not yet catch ball"""
        status = "we not yet catch ball"
        if staticVariables.idGettingPass is None:
            actions[idxThisR] = Actions.BallGrab((-thisRPos+otherAttackerR.get_pos()).arg())
        elif isBallKickedToR(field, idxOtherAttacker, staticVariables.idDoPass):
            staticVariables.idDoPass = None
    elif staticVariables.idDoPass == idxThisR and staticVariables.myIsBallInClass.myIsBallIn(thisR):
        """if this R do pass"""
        status = "if this R do pass"
        doingPass(staticVariables, field, actions, idxThisR)
    elif idxThisR == staticVariables.idGettingPass:
        """if this R getting pass"""
        status = "if this R getting pass"
        gettingPass(staticVariables, field, actions)
    elif staticVariables.idGettingPass != None:
        """if we kick ball for pass, but ally dont yet catch him"""
        status = "if we kick ball for pass, but ally dont yet catch him"
        if isBallOnOurPartOfField(field):
            """if ball on our part of field"""
            status += "if ball on our part of field"
            openForPass(field, idxThisR, actions)
        else:
            """if ball not on our part of field"""
            status += "if ball not on our part of field"
            actions[idxThisR] = Actions.GoToPoint(
                thisRPos, (field.allies[idxOtherAttacker].get_pos() - thisR.get_pos()).arg()
            )
    return status

def GK(
    field: fld.Field, actions: list[Optional[Action]], oldGKState: GKStates, pointFromBallKicked: Optional[aux.Point] = None, angleWithWhatBallKicked: Optional[float] = None, draw: bool = False
) -> GKStates:  
    GKState: GKStates

    a = aux.dist(field.allies[const.GK].get_pos(), field.ball.get_pos())
    field.strategy_image.send_telemetry("dist", str(a))

    oldBallPos = field.ball_start_point
    ballPos = field.ball.get_pos()
    GKPos = field.allies[const.GK].get_pos()
    enenmies = field.active_enemies(False).copy()
    allies = field.active_allies(True).copy()
    allR = enenmies + allies

    if field.game_state is GameStates.PENALTY:
        distToGoOut = myConst.distToBallForGoOutGKForPenalty
    else:
        distToGoOut = myConst.distToBallForGoOutGK
    if draw and pointFromBallKicked is not None and angleWithWhatBallKicked is not None:
        secondPointForLine = pointFromBallKicked+aux.rotate(aux.RIGHT, angleWithWhatBallKicked)
        field.strategy_image.draw_line(pointFromBallKicked, secondPointForLine, (0, 0, 255), 120)
        field.strategy_image.draw_line(oldBallPos, ballPos.unity()*1000+oldBallPos, (255, 0, 0), 100)

    nearestEnemyRToBall = fld.find_nearest_robot(ballPos, field.active_enemies(False))
    nearestRToBall = fld.find_nearest_robot(ballPos, allR)
    # field.strategy_image.draw_circle(nearestRToBall.get_pos(), color=(0, 255, 0), size_in_mms=50)
    enemyRGrabBall = field.is_ball_in(nearestEnemyRToBall)

    if nearestRToBall == field.allies[const.GK] and oldGKState != GKStates.Intersept and not aux.is_point_inside_poly(ballPos, field.ally_goal.hull):
        # field.strategy_image.send_telemetry("GK State", "Pass")
        GKState = GKStates.Pass
        idRToPass = doPassNearAllly(field, actions)
        if idRToPass is None:
            actions[const.GK] = Actions.DelayedSlowKick(field.enemy_goal.center)
    elif (field.ball.get_vel().mag() < myConst.velBallForGoOutGK and 
        aux.dist(aux.closest_point_on_line(field.ally_goal.up, field.ally_goal.down, ballPos), ballPos) < distToGoOut and 
        not aux.is_point_inside_poly(ballPos, field.ally_goal.hull)):
        """if ball dangerously close to goal GK need to go out"""
        GKState = GKStates.GoOut
        pointForScore = findPointForScore(field, reverseGoal=True)
        if pointForScore is not None:
            vectFromBallToDefend = (pointForScore-ballPos)
        else:
            vectFromBallToDefend = (GKPos-ballPos)
        actions[const.GK] = Actions.GoToPointIgnore((vectFromBallToDefend.unity()*myConst.distToStopForGoOutGK)+ballPos, aux.rotate(vectFromBallToDefend, math.pi).arg()).compose(DribblerActions.SetDribblerSpeed(15))
    elif field.is_ball_moves_to_goal() and not enemyRGrabBall:
        if not aux.is_point_on_line(GKPos, oldBallPos, ballPos, "R"):
            interseptBallPoint = aux.closest_point_on_line(oldBallPos, ballPos, GKPos, "R")
            field.strategy_image.draw_circle(interseptBallPoint, color=(255, 0, 0), size_in_mms=50)
            # field.strategy_image.draw_line(GKPos, interseptBallPoint, color=(0, 0, 200), size_in_pixels=20)
            if interseptBallPoint != ballPos:
                # field.strategy_image.send_telemetry("GK State", "Intersept")
                GKState = GKStates.Intersept
                """ intersept ball"""
                actions[const.GK] = Actions.GoToPointIgnore(interseptBallPoint, (ballPos - interseptBallPoint).arg())
            else:
                GKState = GKStates.GrabBall
                # field.strategy_image.send_telemetry("GK State", "Grab ball")
                """grab ball if it maybe in hull and we cant intersept him"""
                actions[const.GK] = Actions.BallGrab((ballPos - GKPos).arg())
        else:
            GKState = GKStates.PassInterstptedBall
            # field.strategy_image.send_telemetry("GK State", "Pass interstpted ball")
            """grab intersepted ball and pass nearly ally"""
            # actions[const.GK] = Actions.BallGrab((ballPos-GKPos).arg)
            doPassNearAllly(field, actions)
    # elif field.is_ball_in(field.allies[const.GK]):
    #     """"""
    #     doPassNearAllly(field, actions)
    elif aux.is_point_inside_poly(ballPos, field.ally_goal.hull):
        GKState = GKStates.KnockOutBall
        # field.strategy_image.send_telemetry("GK State", "Knock out ball")
        """knock out the ball from hull"""
        if len(field.active_allies(False)) != 0:
            doPassNearAllly(field, actions)
        else:
            actions[const.GK] = Actions.DelayedSlowKick(field.enemy_goal.center, is_upper=True)
    else:
        GKState = GKStates.BlockMaybeKick
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
        else:
            """err"""
            print("ERROR IN GK")
    # print("GK State:", GKState, "old:", oldGKState)
    field.strategy_image.send_telemetry("GK State", str(GKState))
    return GKState


def attackerAloneOnField(staticVariables: ClassWithMyStaticVariables, field: fld.Field, actions: list[Optional[Action]], idxThisR: int, idxOtherAttacker: int)->None:
    enemies = field.active_enemies(True)
    enemysRsWithoutGK = field.active_enemies(False)
    thisR: rbt.Robot = field.allies[idxThisR]
    thisRPos = thisR.get_pos()
    ballPos = field.ball.get_pos()

    nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
    if isBallOnOurPartOfField(field):
        """if ball on our part of field"""
        if not staticVariables.myIsBallInClass.myIsBallIn(thisR):
            mostLikelyPointForScore = aux.closest_point_on_line(field.ally_goal.up, field.ally_goal.down, ballPos)
            actions[idxThisR] = Actions.BallGrab((ballPos-mostLikelyPointForScore).arg())
        else:
            """try replace ball from our part of field"""
            actions[idxThisR] = Actions.DelayedSlowKick(field.enemy_goal.center, is_upper=True)
    else:
        """if ball on other part of field"""
        if field.is_ball_in(thisR):
            pointForScore = findPointForScore(field, ballPos)
            if pointForScore != None:
                """if this r can do score, he do"""
                actions[idxThisR] = Actions.DelayedSlowKick(pointForScore)
            else:
                newPointForScore = findPointForScore(field, ballPos, OtherK=1)
                if newPointForScore != None:
                    """if this r can do score, he try do another score"""
                    actions[idxThisR] = Actions.DelayedSlowKick(newPointForScore)
                else:
                    """if this r cant do score, he kick to GK or do upper"""
                    if len(enemysRsWithoutGK) != 0:
                        actions[idxThisR] = Actions.DelayedSlowKick(
                            fld.find_nearest_robot(thisRPos, enemysRsWithoutGK).get_pos(), is_upper=True
                        )
                    else:
                        actions[idxThisR] = Actions.DelayedSlowKick(field.enemies[const.ENEMY_GK].get_pos())
        else:
            actions[idxThisR] = Actions.BallGrab((nearestEnemyR.get_pos() - ballPos).arg())

def attacker(
    staticVariables: ClassWithMyStaticVariables, field: fld.Field, actions: list[Optional[Action]], idxThisR: int, idxOtherAttacker: int
) -> None:  # TODO: solve problem with situation, when ball between 2 robots
    status = "Nothing"
    enemies = field.active_enemies(True)
    enemysRsWithoutGK = field.active_enemies(False)
    allies = field.active_allies(True)
    alliesRWithoutGK = field.active_allies(False)
    alliesWithoutGK = [r.get_pos() for r in alliesRWithoutGK]
    thisR: rbt.Robot = field.allies[idxThisR]
    thisRPos = thisR.get_pos()
    otherAttackerR = field.allies[idxOtherAttacker]
    ballPos = field.ball.get_pos()

    if field.allies[idxThisR].is_used():
        if not field.allies[idxOtherAttacker].is_used():
            """if this attacker alone on field"""
            status = "No 1 r"
            attackerAloneOnField(staticVariables, field, actions, idxThisR, idxOtherAttacker)

        elif getStatusOfPassLogic(staticVariables, field, actions, idxThisR, idxOtherAttacker) is not None:
            pass
            
            """ ↓ ↓ ↓ genegal logic  ↓ ↓ ↓""" 

        elif actions[idxThisR] == None:
            """if we dont send command on this robot"""
            allR = enemies.copy() + allies.copy()
            nearestRToBall = fld.find_nearest_robot(field.ball.get_pos(), allR)
            field.strategy_image.draw_circle(nearestRToBall.get_pos(), (200, 0, 255), 50)
            if nearestRToBall == thisR:
                """if nearest to ball bot this"""
                status = "if nearest to ball bot this"
                if staticVariables.myIsBallInClass.myIsBallIn(thisR):
                    """if this robot have ball"""
                    status += "if this robot have ball"
                    pointForScore = canRDoScoreAndInWhatPoint(field)
                    if pointForScore != None:
                        """try do score if r can"""
                        status += "try do score if r can"
                        actions[idxThisR] = Actions.DelayedSlowKick(pointForScore)
                    else:
                        """if this r cant do score"""
                        status += "if this r cant do score"
                        staticVariables.idDoPass = idxThisR
                        # staticVariables.TimeWeTryDoPass = time()
                else:
                    """if this r is nearest to ball, but dont grab him, grab ball"""
                    status += "if this r is nearest to ball, but dont grab him, grab ball"
                    actions[idxThisR] = Actions.BallGrab((-field.ball.get_pos() + field.enemy_goal.center).arg())
            elif nearestRToBall == field.allies[idxOtherAttacker]:
                """if other attacker have ball"""
                status = "if other attacker have ball"
                if isBallOnOurPartOfField(field) and aux.dist(nearest2BallEnemy(field).get_pos(), ballPos) < 200:
                    """defend on our part of field"""
                    # """code from bottom, copyed"""
                    mostLikelyPointForScore1 = findPointForScore(field, ballPos, reverseGoal=True)
                    if mostLikelyPointForScore1 != None:
                        pointForR = aux.closest_point_on_line(ballPos, mostLikelyPointForScore1, thisR.get_pos())
                        if (
                            not aux.is_point_on_line(thisR.get_pos(), ballPos, mostLikelyPointForScore1, "S")
                            or aux.dist2line(ballPos, mostLikelyPointForScore1, thisR.get_pos()) < 200
                        ):
                            """if this r not block maybe score, block"""
                            status += "if this r not block maybe score, block"
                            actions[idxThisR] = Actions.GoToPoint(pointForR, (ballPos - thisR.get_pos()).arg())
                else:
                    """if ball not on our part field"""
                    openForPass(field, idxThisR, actions)
            elif nearestRToBall == field.allies[const.GK]:
                """if GK have ball"""
                status = "if GK have ball"
                if len(alliesWithoutGK) != 0:
                    nearestRToGK = aux.find_nearest_point(field.ball.get_pos(), alliesWithoutGK)
                    if nearestRToGK == field.allies[idxThisR]:
                        """if this R nearest to ally GK"""
                        status += "if this R nearest to ally GK"
                        openForPass(field, idxThisR, actions)
                    else:
                        """if not this r nearest to ally GK"""
                        status += "if not this r nearest to ally GK"
                        """!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! NOT FINISHED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"""
            elif nearestRToBall == field.enemies[const.ENEMY_GK]:
                """if nearest r to ball is enemy GK"""
                status = "if nearest r to ball is enemy GK"
                dist2BallFromThisR = aux.dist(ballPos, thisR.get_pos())
                dist2BallFromOtherR = aux.dist(ballPos, otherAttackerR.get_pos())
                enemyRsPos = field.active_enemies(False)
                if len(enemies) == 2:
                    if dist2BallFromOtherR < dist2BallFromThisR:
                        """if not this ally r nearest to ball"""
                        status += "if not this ally r nearest to ball and nearest r to ball is enemy GK"
                        enemyRPos = enemyRsPos[0].get_pos()
                        blockEnemyR(field, actions, idxThisR, enemyRPos)
                    else:
                        """if this ally r nearest to ball"""
                        status += "if this ally r nearest to ball"
                    actions[idxThisR] = Actions.BallGrab((ballPos - field.enemy_goal.center).arg())
                elif len(enemies) == 3:
                    """block 2 enemy's attackers"""
                    status += "block 2 enemy's attackers"
                    block2EnemyRs(staticVariables, field, actions, idxThisR, idxOtherAttacker)
                elif len(enemies) == 1:
                    """if enemies have only GK"""
                    status += "enemies have only GK"
                    if aux.is_point_inside_poly(ballPos, field.enemy_goal.hull):
                        """wait when enemy GK kick off ball from hull"""
                        status += "wait when enemy GK kick off ball from hull"
                        actions[idxThisR] = Actions.GoToPoint(aux.Point(0, const.FIELD_DX*(idxThisR==idFirstAttacker)), getAngleFromOurGoalToEnemysGoal(field))
                    else:
                        """attack"""
                        status += "attack"
                        if dist2BallFromOtherR < dist2BallFromThisR:
                            """if not this ally r nearest to ball"""
                            openForPass(field, idxThisR, actions)
                        else:
                            """if this ally r nearest to ball"""
                            actions[idxThisR] = Actions.BallGrab((ballPos - field.enemy_goal.center).arg())
            elif isBallOnOurPartOfField(field):
                """if ball on our part of field"""
                status = "if ball on our part of field"
                dist2BallFromThisR = aux.dist(ballPos, thisR.get_pos())
                dist2BallFromOtherR = aux.dist(ballPos, otherAttackerR.get_pos())
                if dist2BallFromThisR < dist2BallFromOtherR:
                    """if this attacker nearest to ball"""
                    status += "if this attacker nearest to ball"
                    mostLikelyPointForScore1 = findPointForScore(field, ballPos, reverseGoal=True)
                    if mostLikelyPointForScore1 != None:
                        pointForR = aux.closest_point_on_line(ballPos, mostLikelyPointForScore1, thisR.get_pos())
                        if (
                            not aux.is_point_on_line(thisR.get_pos(), ballPos, mostLikelyPointForScore1, "S")
                            or aux.dist2line(ballPos, mostLikelyPointForScore1, thisR.get_pos()) < 200
                        ):
                            """if this r not block maybe score, block"""
                            status += "if this r not block maybe score, block"
                            actions[idxThisR] = Actions.GoToPoint(pointForR, (ballPos - thisR.get_pos()).arg()).compose(DribblerActions.SetDribblerSpeed(15))
                        else:
                            """if this r block maybe score, try grab ball"""
                            status += "if this r block maybe score, try grab ball"
                            nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
                            actions[idxThisR] = Actions.BallGrab((-thisRPos + ballPos).arg())
                    else:
                        """if enemy r cant do score, grab ball"""
                        status += "if enemy r cant do score, grab ball"
                        nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
                        actions[idxThisR] = Actions.BallGrab((-thisRPos + ballPos).arg())
                else:
                    """if nearest attacker for ball other, block maybe pass"""
                    status += "if nearest attacker for ball other, block maybe pass"
                    if len(enemies) > 1:
                        enemyRsPos = fld.find_nearest_robots(ballPos, field.active_enemies(True))
                        enemyRPos = enemyRsPos[1].get_pos()
                        blockEnemyR(field, actions, idxThisR, enemyRPos)
                    else:
                        openForPass(field, idxThisR, actions)
            else:
                """if ball not on our part of field"""
                status = "if ball not on our part of field"
                nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
                dist2BallFromThisR = aux.dist(ballPos, thisR.get_pos())
                dist2BallFromOtherR = aux.dist(ballPos, otherAttackerR.get_pos())
                if dist2BallFromThisR < dist2BallFromOtherR:
                    """if this attacker nearest to ball, take ball"""
                    status += "if this attacker nearest to ball, take ball"
                    vectFromBallToEnemyGoalCenter = -field.ball.get_pos() + field.enemy_goal.center
                    actions[idxThisR] = Actions.BallGrab((vectFromBallToEnemyGoalCenter).arg())
                else:
                    """if nearest attacker for ball other, block maybe pass"""
                    status += "if nearest attacker for ball other, block maybe pass"
                    # TODO resolve problem with choose enemy, which we will block
                    blockEnemyR(field, actions, idxThisR, nearestEnemyR.get_pos())

        """ ↑ ↑ ↑ genegal logic ↑ ↑ ↑ """ 
    print("statusAttacker" + str(idxThisR), status)
    field.strategy_image.send_telemetry("statusAttacker" + str(idxThisR), status)