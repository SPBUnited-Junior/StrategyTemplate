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
from bridge.strategy.ClassWithMyStaticVariables import ClassWithMyStaticVariables  # type: ignore
import bridge.strategy.myConst as myConst
from bridge.strategy.myConst import whatWeDoStates
from bridge.strategy.myLogicFunc import (
    updatePointAndAngleFromWhatBallKicked, 
    updateTimerAndIdWeTryDoPass,
    attacker
)
from bridge.strategy.myFunc import (
    GK
)

from bridge.strategy.testStates import (
    testPass,
    simpleTest,
    testGK,
    testRotateWithBall,
    newIsBallInTest
)

from typing import Callable

testStates: dict[myConst.whatWeDoStates, Callable[[ClassWithMyStaticVariables, fld.Field, list[Optional[Action]]], None]] = {
    myConst.whatWeDoStates.TestPass: testPass,
    myConst.whatWeDoStates.SimpleTest: simpleTest,
    myConst.whatWeDoStates.TestGK: testGK,
    myConst.whatWeDoStates.TestRotateWithBall: testRotateWithBall,
    myConst.whatWeDoStates.NewIsBallInTest: newIsBallInTest,
}

class Strategy:
    """Main class of strategy"""

    def __init__(self) -> None:
        self.staticVariables = ClassWithMyStaticVariables()

    def process(self, field: fld.Field) -> list[Optional[Action]]:
        """Game State Management"""
        if field.game_state not in [GameStates.KICKOFF, GameStates.PENALTY]:
            if field.active_team in [const.Color.ALL, field.ally_color]:
                self.staticVariables.we_active = True
            else:
                self.staticVariables.we_active = False

        actions: list[Optional[Action]] = []
        for _ in range(const.TEAM_ROBOTS_MAX_COUNT):
            actions.append(None)

        # TODO make game states
        print(field.game_state)#for real
        match field.game_state:
            case GameStates.RUN: # GOOD
                self.run(field, actions)

            case GameStates.TIMEOUT:
                states.TIMEOUT(field, actions, self.staticVariables.we_active, self.staticVariables.idFirstAttacker, self.staticVariables.idSecondAttacker)

            case GameStates.HALT:
                return [Actions.Stop()] * const.TEAM_ROBOTS_MAX_COUNT
            
            case GameStates.PREPARE_PENALTY:
                states.PREPARE_PENALTY(field, actions, self.staticVariables.we_active, self.staticVariables.idFirstAttacker, self.staticVariables.idSecondAttacker)

            case GameStates.PENALTY:
                updatePointAndAngleFromWhatBallKicked(self.staticVariables, field)
                self.staticVariables.GKLastState = states.PENALTY(field, actions, self.staticVariables.we_active, self.staticVariables.idFirstAttacker, self.staticVariables.idSecondAttacker, self.staticVariables.GKLastState, self.staticVariables.PointFromBallKicked, self.staticVariables.AngleWithWhatBallKicked)  # one r(our or not) kick ball from center of field, GK other team defend goal

            case GameStates.PREPARE_KICKOFF:
                self.staticVariables.GKLastState = states.PREPARE_KICKOFF(field, actions, self.staticVariables.we_active, self.staticVariables.idFirstAttacker, self.staticVariables.idSecondAttacker, self.staticVariables.GKLastState)  # our Rs on our part of field
                
            case GameStates.KICKOFF:
                self.staticVariables.GKLastState = states.KICKOFF(field, actions, self.staticVariables.we_active, self.staticVariables.idFirstAttacker, self.staticVariables.idSecondAttacker, self.staticVariables.GKLastState)  # our Rs on our part of field

            case GameStates.FREE_KICK:
                self.run(field, actions)

            case GameStates.STOP:
                # The router will automatically prevent robots from getting too close to the ball
                # self.staticVariables.run(field, actions)
                self.staticVariables.GKLastState = states.STOP(field, actions, self.staticVariables.GKLastState)

        return actions
 
    def run(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        # TODO fix problem with that robots comes so close to each other,when they try take ball
        if len(field.active_allies(True)) != 0:  # if our Rs on field
            if field.ally_color == const.COLOR:
                """code for blue"""
                self.staticVariables.myIsBallInClass.updateTimerWeHoldBall(field)
                updateTimerAndIdWeTryDoPass(self.staticVariables, field, actions)
                updatePointAndAngleFromWhatBallKicked(self.staticVariables, field)
                if self.staticVariables.TimeWeTryDoPass is not None:
                    field.strategy_image.send_telemetry("timerPass", str(time()-self.staticVariables.TimeWeTryDoPass))
                else:
                    field.strategy_image.send_telemetry("timerPass", str(None))
                if self.staticVariables.whatWeDoAtThisRun == whatWeDoStates.Play or self.staticVariables.whatWeDoAtThisRun == whatWeDoStates.BothPlay:
                    # print(self.staticVariables.idDoPass, self.staticVariables.idGettingPass)
                    field.strategy_image.send_telemetry("ids", str(self.staticVariables.idDoPass)+" "+str(self.staticVariables.idGettingPass))
                    attacker(self.staticVariables, field, actions, self.staticVariables.idFirstAttacker, self.staticVariables.idSecondAttacker)
                    attacker(self.staticVariables, field, actions, self.staticVariables.idSecondAttacker, self.staticVariables.idFirstAttacker)
                    if field.allies[const.GK].is_used():
                        self.staticVariables.GKLastState = GK(field, actions, self.staticVariables.GKLastState)
                    field.strategy_image.draw_circle(field.ally_goal.center, (0, 0, 255), 20)
                    # print(len(field.active_allies(True)), len(field.active_enemies(True)))#for real
                else:
                    testStates[self.staticVariables.whatWeDoAtThisRun](ClassWithMyStaticVariables(), field, actions)
            else:
                """code for yellow"""
                if self.staticVariables.whatWeDoAtThisRun == whatWeDoStates.BothPlay:
                    if field.allies[const.GK].is_used():
                        self.staticVariables.GKLastState = GK(field, actions, self.staticVariables.GKLastState)
                    attacker(self.staticVariables, field, actions, self.staticVariables.idFirstAttacker, self.staticVariables.idSecondAttacker)
                    attacker(self.staticVariables, field, actions, self.staticVariables.idSecondAttacker, self.staticVariables.idFirstAttacker)
                else:
                    testStates[self.staticVariables.whatWeDoAtThisRun](ClassWithMyStaticVariables(), field, actions)

        else:
            print("WE HAVENT ROBOTS")