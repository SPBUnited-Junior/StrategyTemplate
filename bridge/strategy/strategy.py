"""High-level strategy code"""

# !v DEBUG ONLY
import math  # type: ignore
from time import time  # type: ignore
from typing import Optional

import bridge.strategy.states as states
from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, DribblerActions  # type: ignore
from bridge.router.myDefaultFunc import myIsBallInClass  # type: ignore
import bridge.strategy.myConst as myConst
from bridge.strategy.myConst import whatWeDoStates
from bridge.strategy.myFunc import (
    GK,
    doPassNearAllly,
    findPointForScore,
    openForPass,
    isBallKickedToR,
    nearest2BallEnemy,
    canRDoScoreAndInWhatPoint
)


class Strategy:
    """Main class of strategy"""

    def __init__(self) -> None:
        self.we_active: bool = False
        self.maxVelBall: float = 0
        self.idGettingPass: Optional[int] = None
        # self.idGettingPass: SupportsIndex = None
        self.idDoPass: Optional[int] = None
        self.oldIdDoPass: Optional[int] = None
        self.GKLastState: Optional[str] = None
        self.idFirstAttacker: int = myConst.idFirstAttacker
        self.idSecondAttacker: int = myConst.idSecondAttacker
        self.TimeWeTryDoPass: Optional[float] = None
        self.TimerWeHoldBall: Optional[float] = None
        self.whatWeDoAtThisRun: whatWeDoStates = myConst.whatWeDoAtThisRun
        self.constForTimerWeTryDoPass: float = myConst.constForTimerWeTryDoPass
        self.myIsBallInClass = myIsBallInClass()

    def process(self, field: fld.Field) -> list[Optional[Action]]:
        """Game State Management"""
        if field.game_state not in [GameStates.KICKOFF, GameStates.PENALTY]:
            if field.active_team in [const.Color.ALL, field.ally_color]:
                self.we_active = True
            else:
                self.we_active = False

        actions: list[Optional[Action]] = []
        for _ in range(const.TEAM_ROBOTS_MAX_COUNT):
            actions.append(None)

        # TODO make game states
        print(field.game_state)#for real
        match field.game_state:
            case GameStates.RUN: # GOOD
                self.run(field, actions)

            case GameStates.TIMEOUT:
                states.TIMEOUT(field, actions, self.we_active, self.idFirstAttacker, self.idSecondAttacker)

            case GameStates.HALT:  # GOOD
                return [Actions.Stop()] * const.TEAM_ROBOTS_MAX_COUNT
            
            case GameStates.PREPARE_PENALTY:
                states.PREPARE_PENALTY(field, actions, self.we_active, self.idFirstAttacker, self.idSecondAttacker)

            case GameStates.PENALTY:
                self.GKLastState = states.PENALTY(field, actions, self.we_active, self.idFirstAttacker, self.idSecondAttacker, self.GKLastState)  # one r(our or not) kick ball from center of field, GK other team defend goal

            case GameStates.PREPARE_KICKOFF:
                self.GKLastState = states.PREPARE_KICKOFF(field, actions, self.we_active, self.idFirstAttacker, self.idSecondAttacker, self.GKLastState)  # our Rs on our part of field
                
            case GameStates.KICKOFF:# GOOD
                self.GKLastState = states.KICKOFF(field, actions, self.we_active, self.idFirstAttacker, self.idSecondAttacker, self.GKLastState)  # our Rs on our part of field

            case GameStates.FREE_KICK:
                self.run(field, actions)

            case GameStates.STOP:
                # The router will automatically prevent robots from getting too close to the ball
                self.run(field, actions)

        return actions

    def run(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        # TODO fix problem with that robots comes so close to each other,when they try take ball
        if len(field.active_allies(True)) != 0:  # if our Rs on field
            if field.ally_color == const.COLOR:
                """code for blue"""
                self.myIsBallInClass.updateTimerWeHoldBall(field)
                self.updateTimerAndIdWeTryDoPass(field, actions)
                if self.TimeWeTryDoPass is not None:
                    field.strategy_image.send_telemetry("timerPass", str(time()-self.TimeWeTryDoPass))
                if self.whatWeDoAtThisRun == whatWeDoStates.Play or self.whatWeDoAtThisRun == whatWeDoStates.BothPlay:
                    # print(self.idDoPass, self.idGettingPass)
                    field.strategy_image.send_telemetry("ids", str(self.idDoPass)+" "+str(self.idGettingPass))
                    self.attacker(field, actions, self.idFirstAttacker, self.idSecondAttacker)
                    self.attacker(field, actions, self.idSecondAttacker, self.idFirstAttacker)
                    if field.allies[const.GK].is_used():
                        self.GKLastState = GK(field, actions, self.GKLastState)
                    field.strategy_image.draw_circle(field.ally_goal.center, (0, 0, 255), 20)
                    # print(len(field.active_allies(True)), len(field.active_enemies(True)))#for real
                match self.whatWeDoAtThisRun:

                    case whatWeDoStates.TestPass:
                        actions[const.GK] = Actions.GoToPoint(aux.Point(const.FIELD_DX, const.FIELD_DY), 0)

                        if field.ball.get_pos().y > 5000:
                            print("!!!!!!!!!!!!!!!!!!!!!!!!")
                        else:
                            if self.maxVelBall < field.ball.get_vel().mag() and field.ball.get_vel().mag()<10000:
                                self.maxVelBall = field.ball.get_vel().mag()
                                print("self.maxVelBall =", self.maxVelBall)

                            self.myIsBallInClass.updateTimerWeHoldBall(field)
                            # print("ballVel =", field.ball.get_vel().mag())
                            # field.strategy_image.send_telemetry("ballVel", str(field.ball.get_vel().mag()))

                            ballPos = field.ball.get_pos()
                            nearestR = fld.find_nearest_robot(ballPos, field.active_allies(False))
                            oldIdDoPass = self.idDoPass
                            oldIdGettingPass = self.idGettingPass

                            if self.idDoPass is None and self.idGettingPass is None: #FOR TEST
                                self.idDoPass = nearestR.r_id

                            for thisR in field.active_allies(False):
                                idxThisR = thisR.r_id
                                thisRPos = thisR.get_pos()
                                otherAttackerR = field.allies[(idxThisR==self.idFirstAttacker)*self.idSecondAttacker + (idxThisR==self.idSecondAttacker)*self.idFirstAttacker]
                                idxOtherAttacker = otherAttackerR.r_id

                                if self.idDoPass == idxThisR and not field.is_ball_in(thisR):
                                    """if we not yet catch ball"""
                                    otherAttackerR = field.allies[(nearestR.r_id==self.idFirstAttacker)*self.idSecondAttacker + (nearestR.r_id==self.idSecondAttacker)*self.idFirstAttacker]
                                    if self.idGettingPass is None:
                                        actions[idxThisR] = Actions.BallGrab((-nearestR.get_pos()+otherAttackerR.get_pos()).arg())
                                    elif isBallKickedToR(field, idxOtherAttacker, self.idDoPass):
                                        self.idDoPass = None
                                elif self.idDoPass == idxThisR and self.myIsBallInClass.myIsBallIn(thisR):
                                    """if this R do pass"""
                                    status = "if this R do pass"
                                    if self.idGettingPass is not None and self.idDoPass is not None:
                                        if isBallKickedToR(field, self.idGettingPass, self.idDoPass):
                                            self.idDoPass = None
                                            # print("pass done")
                                            # field.strategy_image.send_telemetry("pass test state", "pass done")
                                        else:
                                            # field.strategy_image.draw_circle(aux.Point(0, 0), size_in_mms=1000)
                                            self.idGettingPass = doPassNearAllly(field, actions, idxThisR)
                                    else:
                                        """do pass"""
                                        # print("do pass")
                                        # field.strategy_image.draw_circle(aux.Point(0, 0), size_in_mms=1000)
                                        self.idGettingPass = doPassNearAllly(field, actions, idxThisR)
                                elif idxThisR == self.idGettingPass:
                                    """if this R getting pass"""
                                    status = "if this R getting pass"
                                    self.gettingPass(field, actions)
                                elif self.idGettingPass != None:
                                    """if we kick ball for pass, but ally dont yet catch him"""
                                    status = "if we kick ball for pass, but ally dont yet catch him"
                                    if ballPos.x * field.polarity > 0:
                                        """if ball on our part of field"""
                                        status += "if ball on our part of field"
                                        openForPass(field, idxThisR, actions)
                                    else:
                                        """if ball not on our part of field"""
                                        status += "if ball not on our part of field"
                                        actions[idxThisR] = Actions.GoToPoint(
                                            thisRPos, (field.allies[idxOtherAttacker].get_pos() - thisR.get_pos()).arg()
                                        )
                            if oldIdDoPass != self.idDoPass or oldIdGettingPass != self.idGettingPass:
                                print(self.idDoPass, self.idGettingPass)
                                # field.strategy_image.send_telemetry("ids:", "self.idDoPass" + str(self.idDoPass) + "self.idGettingPass:" + str(self.idGettingPass))

                    case whatWeDoStates.SimpleTest:

                        openForPass(field, 1, actions)
                        # canRDoScoreAndInWhatPoint(field)
                        # actions[0] = Actions.BallGrab(0)
                        # actions[0] = Actions.Kick(field.enemy_goal.center, is_upper=True)
                        # findPointForScore(field, field.ball.get_pos())
                        # if field.is_ball_in(field.allies[0]):
                        #     doPassNearAllly(field, actions, 0)
                        # else:
                        #     actions[0] = Actions.BallGrab((field.enemy_goal.center-field.allies[0].get_pos()).arg())

                    case whatWeDoStates.TestRotateWithBall:
                        thisR = field.allies[self.idFirstAttacker]
                        if self.maxVelBall < field.ball.get_vel().mag() and field.ball.get_vel().mag()<10000:
                            self.maxVelBall = field.ball.get_vel().mag()
                        if field.is_ball_in(thisR):
                            actions[self.idFirstAttacker] = Actions.Kick(field.enemy_goal.center)
                        else:
                            actions[self.idFirstAttacker] = Actions.BallGrab(math.pi/2)
                    case whatWeDoStates.NewIsBallInTest:
                        thisR = field.allies[self.idFirstAttacker]
                        dist2Ball = (thisR.get_pos() - field.ball.get_pos()).mag()
                        angle2Ball = abs(aux.wind_down_angle((field.ball.get_pos() - thisR.get_pos()).arg() - thisR.get_angle()))
                        print(round(dist2Ball), const.BALL_GRABBED_DIST, round(angle2Ball/math.pi*180, 2), round(const.BALL_GRABBED_ANGLE/math.pi*180))
                        print(int(dist2Ball<const.BALL_GRABBED_DIST), int(angle2Ball<const.BALL_GRABBED_ANGLE), int(dist2Ball<const.BALL_GRABBED_DIST and angle2Ball<const.BALL_GRABBED_ANGLE))

            else:
                """code for yellow"""
                if self.whatWeDoAtThisRun == whatWeDoStates.BothPlay:
                    if field.allies[const.GK].is_used():
                        self.GKLastState = GK(field, actions, self.GKLastState)
                    self.attacker(field, actions, self.idFirstAttacker, self.idSecondAttacker)
                    self.attacker(field, actions, self.idSecondAttacker, self.idFirstAttacker)
        else:
            print("WE HAVENT ROBOTS")

    def updateTimerAndIdWeTryDoPass(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        ballPos = field.ball.get_pos()

        if self.idDoPass is None:
            """if we dont try do or get pass"""
            self.TimeWeTryDoPass = None
        
        elif self.idGettingPass is None and self.TimeWeTryDoPass is None:
            """if we try do pass, but do not done this yet"""
            self.TimeWeTryDoPass = time()

        elif aux.dist(ballPos, field.allies[self.idDoPass].get_pos()) > 100:
            self.TimeWeTryDoPass = None

        
        
        if self.TimeWeTryDoPass is not None and time()-self.TimeWeTryDoPass > self.constForTimerWeTryDoPass:
            """if we try do pass too long"""
            idOtherR =  None
            for r in field.active_allies(False):
                if r.r_id != self.idDoPass:
                    idOtherR = r.r_id
            if idOtherR is not None and self.idDoPass is not None:
                if time()-self.TimeWeTryDoPass > self.constForTimerWeTryDoPass*2:
                    actions[self.idDoPass] = Actions.Kick(nearest2BallEnemy(field).get_pos())#TODO check and add some logic to avoid self-goal
                else:
                    actions[self.idDoPass] = Actions.Kick(field.allies[idOtherR].get_pos(), is_upper=True)
                if isBallKickedToR(field, idOtherR, self.idDoPass):
                    self.TimeWeTryDoPass = None

    def updateTimerWeHoldBall(self, field: fld.Field) -> None:
        if self.TimerWeHoldBall is None:
            if any(field.is_ball_in(r) for r in field.active_allies(True)):
                self.TimerWeHoldBall = time()
        elif all(not field.is_ball_in(r) for r in field.active_allies(True)):
            self.TimerWeHoldBall = None

    def gettingPass(self, field: fld.Field, actions: list[Optional[Action]], test: bool = False) -> None:
        thisRID: int = self.idGettingPass  # type:ignore
        thisR = field.allies[thisRID]
        thisRPos = thisR.get_pos()
        oldBallPos = field.ball_start_point
        ballPos = field.ball.get_pos()

        if self.idDoPass != None:
            """r not yet kick ball"""
            if actions[thisRID] == None:
                openForPass(field, thisRID, actions)
        elif not self.myIsBallInClass.myIsBallIn(thisR):
            if test: field.strategy_image.send_telemetry("status pass", "getting pass")
            """if ball already kicked"""
            if not aux.is_point_on_line(thisRPos, oldBallPos, ballPos, "R"):
                interseptBallPoint = aux.closest_point_on_line(oldBallPos, ballPos, thisRPos, "R")
                if interseptBallPoint != ballPos:
                    if test: field.strategy_image.send_telemetry("status pass", "Intersept")
                    """ intersept ball"""
                    actions[thisRID] = Actions.GoToPointIgnore(interseptBallPoint, (ballPos - interseptBallPoint).arg())
                else:
                    if test: field.strategy_image.send_telemetry("status pass", "Grab ball")
                    """grab ball if it maybe in hull and we cant intersept him"""
                    actions[thisRID] = Actions.BallGrab((ballPos - thisRPos).arg())
            else:
                actions[thisRID] = Actions.BallGrab((field.ball.get_pos()-field.allies[thisRID].get_pos()).arg())
        elif self.myIsBallInClass.myIsBallIn(field.allies[thisRID]):
            """get pass"""
            if test: field.strategy_image.send_telemetry("status pass", "get pass")
            self.idGettingPass = None

        actionThisR = actions[thisRID]
        if actionThisR is not None:
            actions[thisRID] = actionThisR.compose(DribblerActions.SetDribblerSpeed(15))

    def attacker(
        self, field: fld.Field, actions: list[Optional[Action]], idxThisR: int, idxOtherAttacker: int
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

        # field.allies[idxThisR].set_dribbler_speed(0)

        """ ↓ ↓ ↓ logic for not full team ↓ ↓ ↓ """

        if not field.allies[idxOtherAttacker].is_used():
            """if this attacker alone on field"""
            status = "No 1 r"
            nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
            if ballPos.x * field.polarity > 0:
                """if ball on our part of field"""
                if not self.myIsBallInClass.myIsBallIn(thisR):
                    mostLikelyPointForScore = aux.closest_point_on_line(field.ally_goal.up, field.ally_goal.down, ballPos)
                    pointForR = aux.closest_point_on_line(ballPos, mostLikelyPointForScore, thisR.get_pos())
                    if not aux.is_point_on_line(thisR.get_pos(), ballPos, mostLikelyPointForScore, "S"):
                        """if this r not block maybe score, block"""
                        actions[idxThisR] = Actions.GoToPoint(pointForR, (ballPos - thisR.get_pos()).arg()).compose(DribblerActions.SetDribblerSpeed(15))
                    else:
                        """if this r block maybe score, try grab ball"""
                        actions[idxThisR] = Actions.BallGrab((nearestEnemyR.get_pos() - ballPos).arg())
                else:
                    """try replace ball from our part of field"""
                    # TODO need test in real
                    actions[idxThisR] = Actions.Kick(field.enemy_goal.center, is_upper=True)
            else:
                """if ball on other part of field"""
                if field.is_ball_in(thisR):
                    pointForScore = findPointForScore(field, ballPos)
                    if pointForScore != None:
                        """if this r can do score, he do"""
                        actions[idxThisR] = Actions.Kick(pointForScore)
                    else:
                        newPointForScore = findPointForScore(field, ballPos, k=1)
                        if newPointForScore != None:
                            """if this r can do score, he try do another score"""
                            actions[idxThisR] = Actions.Kick(newPointForScore)
                        else:
                            """if this r cant do score, he kick to GK or do upper"""
                            if len(enemysRsWithoutGK) != 0:
                                actions[idxThisR] = Actions.Kick(
                                    fld.find_nearest_robot(thisRPos, enemysRsWithoutGK).get_pos(), is_upper=True
                                )
                            else:
                                actions[idxThisR] = Actions.Kick(field.enemies[const.ENEMY_GK].get_pos())
                else:
                    actions[idxThisR] = Actions.BallGrab((nearestEnemyR.get_pos() - ballPos).arg())
                    
            """ ↑ ↑ ↑ logic for not full team ↑ ↑ ↑ """

            """ ↓ ↓ ↓ logic for pass  ↓ ↓ ↓""" 

        elif self.idDoPass == idxThisR and not self.myIsBallInClass.myIsBallIn(thisR):
            """if we not yet catch ball"""
            if self.idGettingPass is None:
                actions[idxThisR] = Actions.BallGrab((-thisRPos+otherAttackerR.get_pos()).arg())
            elif isBallKickedToR(field, idxOtherAttacker, self.idDoPass):
                self.idDoPass = None
        elif self.idDoPass == idxThisR and self.myIsBallInClass.myIsBallIn(thisR):
            """if this R do pass"""
            status = "if this R do pass"
            if self.idGettingPass is not None and self.idDoPass is not None:
                if isBallKickedToR(field, self.idGettingPass, self.idDoPass):
                    """pass done"""
                    self.idDoPass = None
                else:
                    """pass in process"""
                    # field.strategy_image.draw_circle(aux.Point(0, 0), size_in_mms=1000)
                    if self.idGettingPass is None:
                        pointForScore = canRDoScoreAndInWhatPoint(field)
                        if pointForScore != None:
                            """if r can do score he must break pass"""
                            actions[idxThisR] = Actions.Kick(pointForScore)
                        else:
                            self.idGettingPass = doPassNearAllly(field, actions, idxThisR)

                    else:
                        self.idGettingPass = doPassNearAllly(field, actions, idxThisR)
            else:
                """pass in process"""
                # field.strategy_image.draw_circle(aux.Point(0, 0), size_in_mms=1000)
                self.idGettingPass = doPassNearAllly(field, actions, idxThisR)
        elif idxThisR == self.idGettingPass:
            """if this R getting pass"""
            status = "if this R getting pass"
            self.gettingPass(field, actions)
        elif self.idGettingPass != None:
            """if we kick ball for pass, but ally dont yet catch him"""
            status = "if we kick ball for pass, but ally dont yet catch him"
            if ballPos.x * field.polarity > 0:
                """if ball on our part of field"""
                status += "if ball on our part of field"
                openForPass(field, idxThisR, actions)
            else:
                """if ball not on our part of field"""
                status += "if ball not on our part of field"
                actions[idxThisR] = Actions.GoToPoint(
                    thisRPos, (field.allies[idxOtherAttacker].get_pos() - thisR.get_pos()).arg()
                )
            """ ↑ ↑ ↑ logic for pass  ↑ ↑ ↑ """ 
            
            """ ↓ ↓ ↓ genegal logic  ↓ ↓ ↓""" 

        elif actions[idxThisR] == None:
            """if we dont send command on this robot"""
            allR = enemies.copy() + allies.copy()
            nearestRToBall = fld.find_nearest_robot(field.ball.get_pos(), allR)
            field.strategy_image.draw_circle(nearestRToBall.get_pos(), (200, 0, 255), 50)
            if nearestRToBall == thisR:
                """if nearest to ball bot this"""
                status = "if nearest to ball bot this"
                if self.myIsBallInClass.myIsBallIn(thisR):
                    """if this robot have ball"""
                    status += "if this robot have ball"
                    pointForScore = canRDoScoreAndInWhatPoint(field)
                    if pointForScore != None:
                        """try do score if r can"""
                        status += "try do score if r can"
                        actions[idxThisR] = Actions.Kick(pointForScore)
                    else:
                        """if this r cant do score"""
                        status += "if this r cant do score"
                        self.idDoPass = idxThisR
                        # self.TimeWeTryDoPass = time()
                else:
                    if self.idGettingPass == None:
                        """if this r is nearest to ball, but dont grab him, grab ball"""
                        status += "if this r is nearest to ball, but dont grab him, grab ball"
                        actions[idxThisR] = Actions.BallGrab((-field.ball.get_pos() + field.enemy_goal.center).arg())
                    else:
                        """do do pass and wait for result""" #TODO check: we enter there?
                        status += "do do pass and wait for result"
                        actions[idxThisR] = Actions.GoToPoint(
                            thisRPos, (field.allies[idxOtherAttacker].get_pos() - thisR.get_pos()).arg()
                        )
            elif nearestRToBall == field.allies[idxOtherAttacker]:
                """if other attacker have ball"""
                status = "if other attacker have ball"
                if ballPos.x * field.polarity > 0 and aux.dist(nearest2BallEnemy(field).get_pos(), ballPos) < 200:
                    """defend on our part of field"""
                    # """code from bottom, copyed"""
                    mostLikelyPointForScore1 = findPointForScore(field, ballPos, reverse=True)
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
            elif nearestRToBall == field.enemies[const.GK]:
                """if nearest r to ball is enemy GK"""
                status = "if nearest r to ball is enemy GK"
                dist2BallFromThisR = aux.dist(ballPos, thisR.get_pos())
                dist2BallFromOtherR = aux.dist(ballPos, otherAttackerR.get_pos())
                enemyRsPos = field.active_enemies(False)
                if dist2BallFromOtherR < dist2BallFromThisR:
                    """if not this ally r nearest to ball"""
                    status += "if not this ally r nearest to ball and nearest r to ball is enemy GK"
                    enemyRPos = enemyRsPos[0]
                    pointGo = aux.point_on_line(ballPos, enemyRPos.get_pos(), 300)
                    actions[idxThisR] = Actions.GoToPoint(pointGo, (thisRPos - enemyRPos.get_pos()).arg()).compose(DribblerActions.SetDribblerSpeed(15))
                else:
                    """if this ally r nearest to ball"""
                    status += "if this ally r nearest to ball"
                    actions[idxThisR] = Actions.BallGrab((ballPos - field.enemy_goal.center).arg())
            elif ballPos.x * field.polarity > 0:
                """if ball on our part of field"""
                status = "if ball on our part of field"
                dist2BallFromThisR = aux.dist(ballPos, thisR.get_pos())
                dist2BallFromOtherR = aux.dist(ballPos, otherAttackerR.get_pos())
                if dist2BallFromThisR < dist2BallFromOtherR:
                    """if this attacker nearest to ball"""
                    status += "if this attacker nearest to ball"
                    mostLikelyPointForScore1 = findPointForScore(field, ballPos, reverse=True)
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
                        enemyRPos = enemyRsPos[1]
                        pointGo = aux.point_on_line(ballPos, enemyRPos.get_pos(), 400)
                        actions[idxThisR] = Actions.GoToPoint(pointGo, (thisRPos - enemyRPos.get_pos()).arg()).compose(DribblerActions.SetDribblerSpeed(15))
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
                    # actions[idxThisR] = Actions.BallGrab((nearestEnemyR.get_pos()-ballPos).arg())#GOOD TODO choose
                    actions[idxThisR] = Actions.BallGrab((-field.ball.get_pos() + field.enemy_goal.center).arg())  # work
                else:
                    """if nearest attacker for ball other, block maybe pass"""
                    status += "if nearest attacker for ball other, block maybe pass"
                    # TODO resolve problem with choose enemy, which we will block
                    pointGo = aux.point_on_line(ballPos, nearestEnemyR.get_pos(), 300)
                    actions[idxThisR] = Actions.GoToPoint(pointGo, (thisRPos - nearestEnemyR.get_pos()).arg()).compose(DribblerActions.SetDribblerSpeed(15))

        """ ↑ ↑ ↑ genegal logic ↑ ↑ ↑ """ 
        
        field.strategy_image.send_telemetry("statusAttacker" + str(idxThisR), status)