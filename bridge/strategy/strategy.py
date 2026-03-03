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
    isBallKickedToR
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
        match field.game_state:
            case GameStates.RUN: # GOOD
                self.run(field, actions)

            case GameStates.TIMEOUT:
                states.TIMEOUT(field, actions, self.we_active, self.idFirstAttacker, self.idSecondAttacker)

            case GameStates.HALT:  # GOOD
                return [None] * const.TEAM_ROBOTS_MAX_COUNT
            
            case GameStates.PREPARE_PENALTY:
                states.PREPARE_PENALTY(field, actions, self.we_active, self.idFirstAttacker, self.idSecondAttacker)

            case GameStates.PENALTY:
                states.PENALTY(field, actions, self.we_active, self.idFirstAttacker, self.idSecondAttacker)  # one r(our or not) kick ball from center of field, GK other team defend goal

            case GameStates.PREPARE_KICKOFF:
                states.PREPARE_KICKOFF(field, actions, self.we_active, self.idFirstAttacker, self.idSecondAttacker)  # our Rs on our part of field
                
            case GameStates.KICKOFF:# GOOD
                states.KICKOFF(field, actions, self.we_active, self.idFirstAttacker, self.idSecondAttacker)  # our Rs on our part of field

            case GameStates.FREE_KICK:
                self.run(field, actions)

            case GameStates.STOP:
                # The router will automatically prevent robots from getting too close to the ball
                self.run(field, actions)

        return actions

    def run(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        # TODO fix problem with that robots comes so close to each other,when they try take ball
        # print(field.active_allies(False))
        if len(field.active_allies(True)) != 0:  # if our Rs on field
            if field.ally_color == const.COLOR:
                """code for blue"""
                # print(field.game_state)#for real
                self.myIsBallInClass.updateTimerWeHoldBall(field)
                if self.whatWeDoAtThisRun == whatWeDoStates.Play or self.whatWeDoAtThisRun == whatWeDoStates.BothPlay:
                    # print(self.idDoPass)
                    # print(field.active_allies(False))
                    self.attacker(field, actions, self.idFirstAttacker, self.idSecondAttacker)
                    self.attacker(field, actions, self.idSecondAttacker, self.idFirstAttacker)
                    if field.allies[const.GK].is_used():
                        self.GKLastState = GK(field, actions, self.GKLastState)
                    # print("blue")
                    field.strategy_image.draw_circle(field.ally_goal.center, (0, 0, 255), 20)
                    # print(len(field.active_allies(True)), len(field.active_enemies(True)))#for real
                    # for r in field.active_allies(True):
                    #     field.strategy_image.draw_circle(r.get_pos(), (0, 255, 0), 100)
                    # for r in field.active_enemies(True):
                    #     field.strategy_image.draw_circle(r.get_pos(), (255, 255, 255), 100)
                match self.whatWeDoAtThisRun:

                    case whatWeDoStates.TestPass:
                        # for test pass
                        actions[const.GK] = Actions.GoToPoint(aux.Point(const.FIELD_DX, const.FIELD_DY), 0)

                        if field.ball.get_pos().y > 5000:
                            print("!!!!!!!!!!!!!!!!!!!!!!!!")
                        else:
                            # print(field.ball.get_vel().mag() > 100)
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
                                # print(nearestR.r_id, self.idDoPass)

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
                                    # elif self.idDoPass == idxThisR and self.TimerWeHoldBall is not None and self.TimerWeHoldBall > 0.3:#TODO check this const
                                    """if this R do pass"""
                                    status = "if this R do pass"
                                    # if self.TimeWeTryDoPass is not None and time() - self.TimeWeTryDoPass > 100:
                                    #     actions[idxThisR] = Actions.Kick(otherAttackerR.get_pos(), is_pass=True, is_upper=True)
                                    #     self.TimeWeTryDoPass = None
                                    #     self.idGettingPass = None
                                    #     self.idDoPass = None
                                    #     print("null")
                                    # elif field.is_ball_in(thisR):

                                    # elif self.idGettingPass == None:
                                    #     """grab ball"""
                                    #     # print("gab ball")
                                    #     actions[self.idDoPass] = Actions.BallGrab((field.ball.get_pos() - field.allies[self.idDoPass].get_pos()).arg() )
                                    if self.idGettingPass is not None and self.idDoPass is not None:
                                        if isBallKickedToR(field, self.idGettingPass, self.idDoPass):
                                            self.idDoPass = None
                                            # print("pass done")
                                            # field.strategy_image.send_telemetry("pass test state", "pass done")
                                        else:
                                            # field.strategy_image.draw_circle(aux.Point(0, 0), size_in_mms=1000)
                                            self.idGettingPass = doPassNearAllly(field, actions, idxThisR)
                                        #     """pass done"""
                                        # else:
                                        #     print("POINT IN PASS PROBLEM")
                                        #     field.strategy_image.send_telemetry("pass test state", "POINT IN PASS PROBLEM")
                                    else:
                                        """do pass"""
                                        # print("do pass")
                                        # field.strategy_image.draw_circle(aux.Point(0, 0), size_in_mms=1000)
                                        self.idGettingPass = doPassNearAllly(field, actions, idxThisR)
                                elif idxThisR == self.idGettingPass:
                                    """if this R getting pass"""
                                    status = "if this R getting pass"
                                    self.gettingPass(field, actions, True)
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
                                # else:
                                #     if idxThisR == 4:
                                #         actions[idxThisR] = Actions.GoToPoint(aux.RIGHT, 0)
                                #     else:
                                #         actions[idxThisR] = Actions.GoToPoint(aux.RIGHT*500, 0)
                            if oldIdDoPass != self.idDoPass or oldIdGettingPass != self.idGettingPass:
                                print(self.idDoPass, self.idGettingPass)
                                field.strategy_image.send_telemetry("ids:", "self.idDoPass" + str(self.idDoPass) + "self.idGettingPass:" + str(self.idGettingPass))

                    case whatWeDoStates.SimpleTest:
                        # if not self.myIsBallInClass.myIsBallIn():
                        #     print(1)
                        # actions[0] = Actions.BallGrab((-field.ball.get_pos() + field.enemy_goal.center).arg())  # work
                        # actions[2] = Actions.Kick(field.enemy_goal.center)
                        # field.allies[2].set_dribbler_speed(15)
                        # actions[2] = Actions.GoToPoint(aux.Point(0, 0), 0)
                        # actions[0] = Actions.GoToPoint(aux.Point(0, 0), 0).compose(DribblerActions.SetDribblerSpeed(15))
                        # print(field.is_ball_moves())
                        # print(field.ball.get_vel().mag() > 100)
                        # field.strategy_image.draw_circle(field.ball.get_pos(), (0, 0, 255), 200)
                        # if self.maxVelBall < field.ball.get_vel().mag() and field.ball.get_vel().mag()<10000:
                        #     self.maxVelBall = field.ball.get_vel().mag()
                        # print(self.maxVelBall)
                        # actions[self.idFirstAttacker] = Actions.Kick(field.enemy_goal.center)

                        if field.is_ball_in(field.allies[0]):
                            doPassNearAllly(field, actions, 0)
                        else:
                            actions[0] = Actions.BallGrab((field.enemy_goal.center-field.allies[0].get_pos()).arg())
                    case whatWeDoStates.TestRotateWithBall:
                        thisR = field.allies[self.idFirstAttacker]
                        if self.maxVelBall < field.ball.get_vel().mag() and field.ball.get_vel().mag()<10000:
                            self.maxVelBall = field.ball.get_vel().mag()
                        # print(self.maxVelBall)
                        if field.is_ball_in(thisR):
                            actions[self.idFirstAttacker] = Actions.Kick(field.enemy_goal.center)
                            # actions[self.idFirstAttacker] = Actions.Kick(aux.Point(0, 0))
                            # pointForScore = findPointForScore(field, thisR.get_pos())
                            # if pointForScore != None:
                            #     actions[self.idFirstAttacker] = Actions.Kick(pointForScore)
                            # else:
                            #     # goToNearestScorePoint(field, actions, self.idFirstAttacker, 0)
                            #     field.strategy_image.draw_line(field.enemy_goal.up, field.enemy_goal.down, (0, 0, 0), 30)
                        else:
                            # print(1)
                            actions[self.idFirstAttacker] = Actions.BallGrab(math.pi/2)
                            # field.strategy_image.draw_circle(field.ball.get_pos(), (0, 255, 0), 1000)
                            # print(field.ball.get_pos())
                            # field.strategy_image.draw_circle(aux.Point(0, 0), (0, 255, 0), 1000)
                        # field.strategy_image.send_telemetry("Curr Action", str(actions[self.idFirstAttacker]))
                        # print(actions[self.idFirstAttacker])
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
                    # print("yellow")
                # now = time()%8//4 # for change koef
                # match now:
                #     case 0:
                #         # pointF = field.ally_goal.center_down
                #         pointF = aux.Point(0, 200)
                #         field.strategy_image.draw_circle(aux.Point(0, 200), (200, 0, 0), 50)
                #         # print("1")
                #     case 1:
                #         # pointF = field.ally_goal.frw_down
                #         pointF = aux.Point(0, 0)
                #         field.strategy_image.draw_circle(aux.Point(0, 0), (200, 0, 0), 50)
                #         # print(2)
                # actions[const.GK] = Actions.GoToPointIgnore(aux.Point(pointF.x+100, pointF.y), 0)
        else:
            print("WE HAVENT ROBOTS")

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
        # field.allies[thisRID].set_dribbler_speed(15)

        if self.idDoPass != None:
            """r not yet kick ball"""
            if actions[thisRID] == None:
                # if test:
                #     # pointForWaitingForPass = aux.nearest_point_on_circle(thisR.get_pos(), field.allies[self.idDoPass].get_pos(), 500)
                openForPass(field, thisRID, actions)
                # else:
                #     pointForWaitingForPass = thisR.get_pos()
                #     actions[thisRID] = Actions.GoToPoint(pointForWaitingForPass, (field.allies[self.idDoPass].get_pos() - thisR.get_pos()).arg())
        elif not self.myIsBallInClass.myIsBallIn(thisR):
            # field.strategy_image.send_telemetry("status pass", "getting pass")
            """if ball already kicked"""
            if not aux.is_point_on_line(thisRPos, oldBallPos, ballPos, "R"):
                interseptBallPoint = aux.closest_point_on_line(oldBallPos, ballPos, thisRPos, "R")
                if interseptBallPoint != ballPos:
                    # field.strategy_image.send_telemetry("GK State", "Intersept")
                    """ intersept ball"""
                    actions[thisRID] = Actions.GoToPointIgnore(interseptBallPoint, (ballPos - interseptBallPoint).arg())
                else:
                    # field.strategy_image.send_telemetry("GK State", "Grab ball")
                    """grab ball if it maybe in hull and we cant intersept him"""
                    actions[thisRID] = Actions.BallGrab((ballPos - thisRPos).arg())
            else:
                actions[thisRID] = Actions.BallGrab((field.ball.get_pos()-field.allies[thisRID].get_pos()).arg())
            # actions[thisRID] = Actions.BallGrab((-field.ball.get_pos() + field.enemy_goal.center).arg())  # work - r catch ball with that angle that he look straight at enemy goal
        elif field.is_ball_in(field.allies[thisRID]) and not field.is_ball_moves():
            """get pass"""
            # field.strategy_image.send_telemetry("status pass", "get pass")
            # actions[idxThisR] = Actions.Kick(findPointForScore(field))
            # self.idDoPass = thisRID
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
        # alliesWithoutGK = allies.copy()
        # alliesWithoutGK.remove(field.allies[const.GK])
        alliesRWithoutGK = field.active_allies(False)
        alliesWithoutGK = [r.get_pos() for r in alliesRWithoutGK]
        thisR: rbt.Robot = field.allies[idxThisR]
        thisRPos = thisR.get_pos()
        otherAttackerR = field.allies[idxOtherAttacker]
        ballPos = field.ball.get_pos()

        # field.allies[idxThisR].set_dribbler_speed(0)

        """ ↓ ↓ ↓ logic for for full team ↓ ↓ ↓ """

        if not field.allies[idxOtherAttacker].is_used():
            """if this attacker alone on field"""
            status = "No 1 r"
            nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
            if ballPos.x * field.polarity > 0:
                """if ball on our part of field"""
                if not field.is_ball_in(thisR):
                    mostLikelyPointForScore = aux.closest_point_on_line(field.ally_goal.up, field.ally_goal.down, ballPos)
                    pointForR = aux.closest_point_on_line(ballPos, mostLikelyPointForScore, thisR.get_pos())
                    if not aux.is_point_on_line(thisR.get_pos(), ballPos, mostLikelyPointForScore, "S"):
                        """if this r not block maybe score, block"""
                        actions[idxThisR] = Actions.GoToPoint(pointForR, (ballPos - thisR.get_pos()).arg()).compose(DribblerActions.SetDribblerSpeed(15))
                        # field.allies[idxThisR].set_dribbler_speed(15)
                    else:
                        """if this r block maybe score, try grab ball"""
                        # nearestEnemyR = fld.find_nearest_robot(field.ball.get_pos(), enemys)
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

        elif self.idDoPass == idxThisR and not field.is_ball_in(thisR):
            """if we not yet catch ball"""
            if self.idGettingPass is None:
                actions[idxThisR] = Actions.BallGrab((-otherAttackerR.get_pos()+otherAttackerR.get_pos()).arg())
            elif isBallKickedToR(field, idxOtherAttacker, self.idDoPass):
                self.idDoPass = None
        elif self.idDoPass == idxThisR and self.myIsBallInClass.myIsBallIn(thisR):
            """if this R do pass"""
            status = "if this R do pass"
            # if self.TimeWeTryDoPass is not None and time() - self.TimeWeTryDoPass > 100:
            #     actions[idxThisR] = Actions.Kick(otherAttackerR.get_pos(), is_pass=True, is_upper=True)
            #     self.TimeWeTryDoPass = None
            #     self.idGettingPass = None
            #     self.idDoPass = None
            #     print("null")
            # elif field.is_ball_in(thisR):

            # elif self.idGettingPass == None:
            #     """grab ball"""
            #     # print("gab ball")
            #     actions[self.idDoPass] = Actions.BallGrab((field.ball.get_pos() - field.allies[self.idDoPass].get_pos()).arg() )
            if self.idGettingPass is not None and self.idDoPass is not None:
                if isBallKickedToR(field, self.idGettingPass, self.idDoPass):
                    self.idDoPass = None
                    # print("pass done")
                    # field.strategy_image.send_telemetry("pass test state", "pass done")
                else:
                    # field.strategy_image.draw_circle(aux.Point(0, 0), size_in_mms=1000)
                    self.idGettingPass = doPassNearAllly(field, actions, idxThisR)
                #     """pass done"""
                # else:
                #     print("POINT IN PASS PROBLEM")
                #     field.strategy_image.send_telemetry("pass test state", "POINT IN PASS PROBLEM")
            else:
                """do pass"""
                # print("do pass")
                # field.strategy_image.draw_circle(aux.Point(0, 0), size_in_mms=1000)
                self.idGettingPass = doPassNearAllly(field, actions, idxThisR)
        elif idxThisR == self.idGettingPass:
            """if this R getting pass"""
            status = "if this R getting pass"
            self.gettingPass(field, actions, True)
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
            # status = "if we dont send command on this robot"
            allR = enemies.copy() + allies.copy()
            nearestRToBall = fld.find_nearest_robot(field.ball.get_pos(), allR)
            field.strategy_image.draw_circle(nearestRToBall.get_pos(), (200, 0, 255), 50)
            # print(nearestRToBall.r_id)
            if nearestRToBall == thisR:
                """if nearest to ball bot this"""
                status = "if nearest to ball bot this"
                if field.is_ball_in(thisR):
                    # field.strategy_image.draw_circle(thisR.get_pos(), (255, 255, 255), 50)
                    """if this robot have ball"""
                    status += "if this robot have ball"
                    pointForScore = findPointForScore(field)
                    if pointForScore != None:
                        """try do score if r can"""
                        status += "try do score if r can"
                        actions[idxThisR] = Actions.Kick(pointForScore)
                    else:
                        """if this r cant do score"""
                        status += "if this r cant do score"
                        # self.idGettingPass = doPassNearAllly(field, actions, idxThisR)
                        self.idDoPass = idxThisR
                        self.TimeWeTryDoPass = time()
                        # if pointForScoreForOtherAttacker != None:
                        #     """if other attacker can do score, pass other attacker"""
                        #     status = "if other attacker can do score, pass other attacker"
                        #     self.idGettingPass = doPassNearAllly(field, actions, idxThisR)
                        # else:
                        #     """if both attackers cant do score try do score: change position"""
                        #     status = "if both attackers cant do score try do score: change position"
                        #     goToNearestScorePoint(field, actions, 0, None)
                else:
                    if self.idGettingPass == None:
                        """if this r is nearest to ball, but dont grab him, grab ball"""
                        status += "if this r is nearest to ball, but dont grab him, grab ball"
                        # actions[idxThisR] = Actions.BallGrab((field.ball.get_pos() - thisR.get_pos()).arg())
                        actions[idxThisR] = Actions.BallGrab((-field.ball.get_pos() + field.enemy_goal.center).arg())
                    else:
                        """do do pass and wait for result"""
                        status += "do do pass and wait for result"
                        actions[idxThisR] = Actions.GoToPoint(
                            thisRPos, (field.allies[idxOtherAttacker].get_pos() - thisR.get_pos()).arg()
                        )
            elif nearestRToBall == field.allies[idxOtherAttacker]:
                """if other attacker have ball"""
                status = "if other attacker have ball"
                # goToNearestScorePoint(field, actions, idxThisR, idxOtherAttacker) #BAD
                openForPass(field, idxThisR, actions)
            elif nearestRToBall == field.allies[const.GK]:
                """if GK have ball"""
                status = "if GK have ball"
                # print(alliesWithoutGK, field.allies)
                if len(alliesWithoutGK) != 0:
                    nearestRToGK = aux.find_nearest_point(field.ball.get_pos(), alliesWithoutGK)
                    if nearestRToGK == field.allies[idxThisR]:
                        """if this R nearest to ally GK"""
                        status += "if this R nearest to ally GK"
                        openForPass(field, idxThisR, actions)
                    else:
                        """if not this r nearest to ally GK"""
                        status += "if not this r nearest to ally GK"
                        # actions[idxThisR] = Actions.GoToPoint(aux.Point(0, 0), 0)
            elif nearestRToBall == field.enemies[const.GK]:
                """if nearest r to ball is enemy GK"""
                status = "if nearest r to ball is enemy GK"
                # status = "if nearest r to ball is enemy GK"
                dist2BallFromThisR = aux.dist(ballPos, thisR.get_pos())
                dist2BallFromOtherR = aux.dist(ballPos, otherAttackerR.get_pos())
                enemyRsPos = field.active_enemies(False)
                if dist2BallFromOtherR < dist2BallFromThisR:
                    """if not this ally r nearest to ball"""
                    status += "if not this ally r nearest to ball and nearest r to ball is enemy GK"
                    # enemyRsPos.remove(fld.find_nearest_robot(ballPos, enemyRsPos))
                    enemyRPos = enemyRsPos[0]
                    pointGo = aux.point_on_line(ballPos, enemyRPos.get_pos(), 300)
                    actions[idxThisR] = Actions.GoToPoint(pointGo, (thisRPos - enemyRPos.get_pos()).arg()).compose(DribblerActions.SetDribblerSpeed(15))
                    # field.allies[idxThisR].set_dribbler_speed(15)
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
                    # mostLikelyPointForScore = aux.closest_point_on_line(field.ally_goal.up, field.ally_goal.down, ballPos)
                    if mostLikelyPointForScore1 != None:
                        pointForR = aux.closest_point_on_line(ballPos, mostLikelyPointForScore1, thisR.get_pos())
                        if (
                            not aux.is_point_on_line(thisR.get_pos(), ballPos, mostLikelyPointForScore1, "S")
                            or aux.dist2line(ballPos, mostLikelyPointForScore1, thisR.get_pos()) < 200
                        ):
                            """if this r not block maybe score, block"""
                            status += "if this r not block maybe score, block"
                            actions[idxThisR] = Actions.GoToPoint(pointForR, (ballPos - thisR.get_pos()).arg()).compose(DribblerActions.SetDribblerSpeed(15))
                            # field.allies[idxThisR].set_dribbler_speed(15)
                        else:
                            """if this r block maybe score, try grab ball"""
                            status += "if this r block maybe score, try grab ball"
                            nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
                            actions[idxThisR] = Actions.BallGrab((nearestEnemyR.get_pos() - ballPos).arg())
                    else:
                        """if enemy r cant do score, grab ball"""
                        status += "if enemy r cant do score, grab ball"
                        nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
                        actions[idxThisR] = Actions.BallGrab((nearestEnemyR.get_pos() - ballPos).arg())
                else:
                    """if nearest attacker for ball other, block maybe pass"""
                    status += "if nearest attacker for ball other, block maybe pass"
                    # enemyRPos = field.allies[3].get_pos() # HARD CODE
                    # enemyRsPos = field.active_enemies(False).copy()
                    # enemyRsPos.remove(fld.find_nearest_robot(ballPos, enemyRsPos))
                    # enemyRPos = enemyRsPos[0]
                    if len(enemies) > 1:
                        enemyRsPos = fld.find_nearest_robots(ballPos, field.active_enemies(True))
                        enemyRPos = enemyRsPos[1]
                        # pointGo = aux.closest_point_on_line(enemyRPos, ballPos, rPos, "R")
                        pointGo = aux.point_on_line(ballPos, enemyRPos.get_pos(), 400)
                        actions[idxThisR] = Actions.GoToPoint(pointGo, (thisRPos - enemyRPos.get_pos()).arg()).compose(DribblerActions.SetDribblerSpeed(15))
                        # field.allies[idxThisR].set_dribbler_speed(15)
                    else:
                        openForPass(field, idxThisR, actions)
            else:
                """if ball not on our part of field"""
                status = "if ball not on our part of field"
                # enemyRsPos = field.active_enemies(False).copy()
                # enemyRsPos.remove(fld.find_nearest_robot(ballPos, enemyRsPos))
                # enemyRPos = enemyRsPos[0]
                # enemies = field.active_enemies(True)
                nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
                dist2BallFromThisR = aux.dist(ballPos, thisR.get_pos())
                dist2BallFromOtherR = aux.dist(ballPos, otherAttackerR.get_pos())
                if dist2BallFromThisR < dist2BallFromOtherR:
                    """if this attacker nearest to ball, take ball"""
                    status += "if this attacker nearest to ball, take ball"
                    # actions[idxThisR] = Actions.BallGrab((nearestEnemyR.get_pos()-ballPos).arg())#GOOD
                    actions[idxThisR] = Actions.BallGrab((-field.ball.get_pos() + field.enemy_goal.center).arg())  # work
                else:
                    """if nearest attacker for ball other, block maybe pass"""
                    status += "if nearest attacker for ball other, block maybe pass"
                    # enemyRPos = field.allies[3].get_pos() # HARD CODE
                    # pointGo = aux.closest_point_on_line(enemyRPos, ballPos, rPos, "R")
                    # TODO resolve problem with choose enemy, which we will block
                    pointGo = aux.point_on_line(ballPos, nearestEnemyR.get_pos(), 300)
                    actions[idxThisR] = Actions.GoToPoint(pointGo, (thisRPos - nearestEnemyR.get_pos()).arg()).compose(DribblerActions.SetDribblerSpeed(15))
                    # field.allies[idxThisR].set_dribbler_speed(15)
        # print(status, idxThisR)

        """ ↑ ↑ ↑ genegal logic ↑ ↑ ↑ """ 
        
        field.strategy_image.send_telemetry("statusAttacker" + str(idxThisR), status)
