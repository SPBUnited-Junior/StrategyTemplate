"""High-level strategy code"""

# !v DEBUG ONLY
import math  # type: ignore
from bridge.strategy.myFunc import *
import bridge.strategy.states as states
from time import time  # type: ignore
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions  # type: ignore




class Strategy:
    """Main class of strategy"""

    def __init__(
        self
    ) -> None:
        self.we_active: bool = False
        self.state: int = 0
        self.idGettingPass: Optional[int] = None
        # self.idGettingPass: SupportsIndex = None
        self.idDoPass: Optional[int] = None
        self.GKLastState: Optional[str] = None
        self.idFirstAttacker: int = 7
        self.idSecondAttacker: int = 6
        # self.TimeWeTryDoPass: Optional[float] = None
        self.whatWeDoAtThisRun: whatWeDoStates = whatWeDoStates.TestRotateWithBall

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

        if field.ally_color == const.COLOR:
            text = str(field.game_state) + "  we_active:" + str(self.we_active)
            field.strategy_image.print(aux.Point(600, 780), text, need_to_scale=False)
        #TODO make game states
        match field.game_state:
            case GameStates.RUN:
                self.run(field, actions)
            case GameStates.TIMEOUT:
                states.TIMEOUT(field, actions, self.we_active)
            case GameStates.HALT:#GOOD
                return [None] * const.TEAM_ROBOTS_MAX_COUNT
            case GameStates.PREPARE_PENALTY:
                states.PREPARE_PENALTY(field, actions, self.we_active)
            case GameStates.PENALTY:
                states.PENALTY(field, actions, self.we_active)# one r(our or not) kick ball from center of field, GK other team defend goal
            case GameStates.PREPARE_KICKOFF:
                states.PREPARE_KICKOFF(field, actions, self.we_active)#our Rs on our part of field
            case GameStates.KICKOFF:
                states.KICKOFF(field, actions, self.we_active)#our Rs on our part of field
            case GameStates.FREE_KICK: 
                self.run(field, actions)
            case GameStates.STOP:
                # The router will automatically prevent robots from getting too close to the ball
                self.run(field, actions)

        return actions

    def run(self, field: fld.Field, actions: list[Optional[Action]]) -> None:#TODO fix rotate with ball, here's 2 options: rotate at arc, or slow rotate with ball
        #TODO fix problem with that robots comes so close to each other,when they try take ball
        # print(field.active_allies())
        if len(field.active_allies(True)) != 0:#if our Rs on field
            if field.ally_color == const.Color.BLUE:
                """code for blue"""
                # print(field.game_state)#for real
                if self.whatWeDoAtThisRun == whatWeDoStates.Play or self.whatWeDoAtThisRun == whatWeDoStates.BothPlay:
                    # print(self.idDoPass)
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
                        nearestRToBall = fld.find_nearest_robot(field.ball.get_pos(), field.active_allies())
                        otherR = field.allies[2*(nearestRToBall.r_id != 2)]#HARD CODE
                        ballPos = field.ball.get_pos()
                        # actions[otherR.r_id] = Actions.GoToPoint(aux.Point(ballPos.x, ballPos.y *-1), (ballPos- field.allies[otherR.r_id].get_pos()).arg())
                        if self.idDoPass == None and self.idGettingPass == None:
                            self.idDoPass = nearestRToBall.r_id
                        if self.idDoPass != None:
                            self.state = self.idDoPass
                            if field.is_ball_in(field.allies[self.idDoPass]):
                                """do pass"""
                                self.idGettingPass = doPassNearAllly(field, actions, nearestRToBall.r_id)
                            elif self.idGettingPass == None:
                                """grab ball"""
                                actions[self.idDoPass] = Actions.BallGrab((field.ball.get_pos() - field.allies[self.idDoPass].get_pos()).arg())  
                            else:
                                self.idDoPass = None
                                """pass done"""
                        else:
                            actions[self.state] = Actions.GoToPoint(field.allies[self.state].get_pos(), 0)
                        if self.idGettingPass != None:
                            self.gettingPass(field, actions)
                        print(self.idDoPass, self.idGettingPass)

                    case whatWeDoStates.SimpleTest:
                        actions[0] = Actions.BallGrab((-field.ball.get_pos() + field.enemy_goal.center).arg())#work

                    case whatWeDoStates.TestRotateWithBall:
                        thisR = field.allies[self.idFirstAttacker]
                        if field.is_ball_in(thisR):
                            pointForScore = findPointForScore(field, thisR.get_pos())
                            if pointForScore != None:
                                actions[self.idFirstAttacker] = Actions.Kick(pointForScore)
                            else:
                                # goToNearestScorePoint(field, actions, self.idFirstAttacker, 0)
                                field.strategy_image.draw_line(field.enemy_goal.up, field.enemy_goal.down, (0, 0, 0), 30)
                        else:
                            actions[self.idFirstAttacker] = Actions.BallGrab(0)
                        # field.strategy_image.send_telemetry("Curr Action", str(actions[self.idFirstAttacker]))
                        # print(actions[self.idFirstAttacker])

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

    def gettingPass(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        idxPass: int = self.idGettingPass #type:ignore
        thisR = field.allies[idxPass]
        if self.idDoPass != None:
            """pass not yet done"""
            if actions[idxPass] == None:
                actions[idxPass] = Actions.GoToPoint(thisR.get_pos(), (field.allies[self.idDoPass].get_pos()-thisR.get_pos()).arg())
        elif not field.is_ball_in(field.allies[idxPass]) and field.is_ball_moves():
            # field.strategy_image.send_telemetry("status pass", "getting pass")
            """getting pass"""
            # actions[idxPass] = Actions.BallGrab((field.ball.get_pos()-field.allies[idxPass].get_pos()).arg())
            # actions[idxPass] = Actions.BallGrab((field.ball.get_pos()-field.allies[idxPass].get_pos()).arg())
            actions[idxPass] = Actions.BallGrab((-field.ball.get_pos() + field.enemy_goal.center).arg())#work
        else:
            """get pass"""
            # field.strategy_image.send_telemetry("status pass", "get pass")
            # actions[idxThisR] = Actions.Kick(findPointForScore(field))
            # self.idDoPass = idxPass
            self.idGettingPass = None

    def attacker(self, field: fld.Field, actions: list[Optional[Action]], idxThisR: int, idxOtherAttacker: int) -> None:#TODO: solve problem with situation, when ball between 2 robots
        status = "Nothing"
        enemies = field.active_enemies(True)
        enemysRsWithoutGK = field.active_enemies()
        allies = field.active_allies(True)
        # alliesWithoutGK = allies.copy()
        # alliesWithoutGK.remove(field.allies[const.GK])
        alliesRWithoutGK = field.active_allies()
        alliesWithoutGK = [r.get_pos() for r in alliesRWithoutGK]
        thisR: rbt.Robot = field.allies[idxThisR]
        thisRPos = thisR.get_pos()
        otherAttackerR = field.allies[idxOtherAttacker]
        ballPos = field.ball.get_pos()

        field.allies[idxThisR].set_dribbler_speed(0)

        if not field.allies[idxOtherAttacker].is_used():
            """if this attacker alone on field"""
            status = "No 1 r"
            nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
            if ballPos.x*field.polarity > 0:
                """if ball on our part of field"""
                if not field.is_ball_in(thisR):
                    mostLikelyPointForScore = aux.closest_point_on_line(field.ally_goal.up, field.ally_goal.down, ballPos)
                    pointForR = aux.closest_point_on_line(ballPos, mostLikelyPointForScore, thisR.get_pos())
                    if not aux.is_point_on_line(thisR.get_pos(), ballPos, mostLikelyPointForScore, "S"):
                        """if this r not block maybe score, block"""
                        actions[idxThisR] = Actions.GoToPoint(pointForR, (ballPos-thisR.get_pos()).arg())
                        field.allies[idxThisR].set_dribbler_speed(15)
                    else:
                        """if this r block maybe score, try grab ball"""
                        # nearestEnemyR = fld.find_nearest_robot(field.ball.get_pos(), enemys)
                        actions[idxThisR] = Actions.BallGrab((nearestEnemyR.get_pos()-ballPos).arg())
                else:
                    """try replace ball from our part of field"""
                    #TODO need test in real
                    actions[idxThisR] = Actions.Kick(field.enemy_goal.center, is_upper=True)
            else:
                """if ball on other part of field"""
                if field.is_ball_in(thisR):
                    pointForScore = findPointForScore(field, ballPos)
                    if pointForScore != None:
                        """if this r can do score, he do"""
                        actions[idxThisR] = Actions.Kick(pointForScore)
                    else:
                        newPointForScore = findPointForScore(field, ballPos, k = 1)
                        if newPointForScore != None:
                            """if this r can do score, he try do another score"""
                            actions[idxThisR] = Actions.Kick(newPointForScore)
                        else:
                            """if this r cant do score, he kick to GK or do upper"""
                            if len(enemysRsWithoutGK) != 0:
                                actions[idxThisR] = Actions.Kick(fld.find_nearest_robot(thisRPos, enemysRsWithoutGK).get_pos(), is_upper=True)
                            else:
                                actions[idxThisR] = Actions.Kick(field.enemies[const.ENEMY_GK].get_pos())
                else:
                    actions[idxThisR] = Actions.BallGrab((nearestEnemyR.get_pos()-ballPos).arg())
        # elif len(enemies) == 0:
        elif self.idDoPass == idxThisR and aux.dist(ballPos, thisRPos) < 200:
            """if this R do pass"""
            status = "if this R do pass"
            if field.is_ball_in(field.allies[self.idDoPass]):
                """do pass"""
                # print("do pass")
                self.idGettingPass = doPassNearAllly(field, actions, idxThisR)
            elif self.idGettingPass == None:
                """grab ball"""
                # print("gab ball")
                actions[self.idDoPass] = Actions.BallGrab((field.ball.get_pos() - field.allies[self.idDoPass].get_pos()).arg())  
            else:
                self.idDoPass = None
                # print("pass done")
                """pass done"""
        elif self.idDoPass == idxThisR and aux.dist(ballPos, thisRPos) >= 200:
            self.idDoPass = None
        elif idxThisR == self.idGettingPass:
            """if this R getting pass"""
            status = "if this R getting pass"
            self.gettingPass(field, actions)
        elif self.idGettingPass != None:
            """if we kick ball for pass, but ally dont yet catch him"""
            status = "if we kick ball for pass, but ally dont yet catch him"
            if ballPos.x*field.polarity > 0:
                """if ball on our part of field"""
                status += "if ball on our part of field"
                openForPass(field, idxThisR, actions)
            else:
                """if ball not on our part of field"""
                status += "if ball not on our part of field"
                actions[idxThisR] = Actions.GoToPoint(thisRPos, (field.allies[idxOtherAttacker].get_pos()-thisR.get_pos()).arg())
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
                        actions[idxThisR] = Actions.GoToPoint(thisRPos, (field.allies[idxOtherAttacker].get_pos()-thisR.get_pos()).arg())
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
                enemyRsPos = field.active_enemies()
                if dist2BallFromOtherR < dist2BallFromThisR:
                    """if not this ally r nearest to ball"""
                    status += "if not this ally r nearest to ball and nearest r to ball is enemy GK"
                    # enemyRsPos.remove(fld.find_nearest_robot(ballPos, enemyRsPos))
                    enemyRPos = enemyRsPos[0]
                    pointGo = aux.point_on_line(ballPos, enemyRPos.get_pos(), 300)
                    actions[idxThisR] = Actions.GoToPoint(pointGo, (thisRPos-enemyRPos.get_pos()).arg())
                    field.allies[idxThisR].set_dribbler_speed(15)
                else:
                    """if this ally r nearest to ball"""
                    status += "if this ally r nearest to ball"
                    actions[idxThisR] = Actions.BallGrab((ballPos-field.enemy_goal.center).arg())
            elif ballPos.x*field.polarity > 0:
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
                        if not aux.is_point_on_line(thisR.get_pos(), ballPos, mostLikelyPointForScore1, "S") or aux.dist2line(ballPos, mostLikelyPointForScore1, thisR.get_pos())<200:
                            """if this r not block maybe score, block"""
                            status += "if this r not block maybe score, block"
                            actions[idxThisR] = Actions.GoToPoint(pointForR, (ballPos-thisR.get_pos()).arg())
                            field.allies[idxThisR].set_dribbler_speed(15)
                        else:
                            """if this r block maybe score, try grab ball"""
                            status += "if this r block maybe score, try grab ball"
                            nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
                            actions[idxThisR] = Actions.BallGrab((nearestEnemyR.get_pos()-ballPos).arg())
                    else:
                        """if enemy r cant do score, grab ball"""
                        status += "if enemy r cant do score, grab ball"
                        nearestEnemyR = fld.find_nearest_robot(ballPos, enemies)
                        actions[idxThisR] = Actions.BallGrab((nearestEnemyR.get_pos()-ballPos).arg())
                else:   
                    """if nearest attacker for ball other, block maybe pass"""
                    status += "if nearest attacker for ball other, block maybe pass"
                    # enemyRPos = field.allies[3].get_pos() # HARD CODE
                    # enemyRsPos = field.active_enemies().copy()
                    # enemyRsPos.remove(fld.find_nearest_robot(ballPos, enemyRsPos))
                    # enemyRPos = enemyRsPos[0]
                    if len(enemies) > 1:
                        enemyRsPos = fld.find_nearest_robots(ballPos, field.active_enemies(True))
                        enemyRPos = enemyRsPos[1]
                        # pointGo = aux.closest_point_on_line(enemyRPos, ballPos, rPos, "R")
                        pointGo = aux.point_on_line(ballPos, enemyRPos.get_pos(), 400)
                        actions[idxThisR] = Actions.GoToPoint(pointGo, (thisRPos-enemyRPos.get_pos()).arg())
                        field.allies[idxThisR].set_dribbler_speed(15)
                    else:
                        openForPass(field, idxThisR, actions)
            else:
                """if ball not on our part of field"""
                status = "if ball not on our part of field"
                # enemyRsPos = field.active_enemies().copy()
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
                    actions[idxThisR] = Actions.BallGrab((-field.ball.get_pos() + field.enemy_goal.center).arg())#work
                else:
                    """if nearest attacker for ball other, block maybe pass"""
                    status += "if nearest attacker for ball other, block maybe pass"
                    # enemyRPos = field.allies[3].get_pos() # HARD CODE
                    # pointGo = aux.closest_point_on_line(enemyRPos, ballPos, rPos, "R")
                    #TODO resolve problem with choose enemy, which we will block
                    pointGo = aux.point_on_line(ballPos, nearestEnemyR.get_pos(), 300)
                    actions[idxThisR] = Actions.GoToPoint(pointGo, (thisRPos-nearestEnemyR.get_pos()).arg())
                    field.allies[idxThisR].set_dribbler_speed(15)
        # print(status, idxThisR)
        field.strategy_image.send_telemetry("statusAttacker"+str(idxThisR), status)