import math  # type: ignore
from time import time  # type: ignore
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.router.base_actions import Action, Actions, KickActions  # type: ignore
from bridge.strategy.myFunc import findPointForScore
from bridge.strategy.myConst import minDistForScorePenalty, timerForRotate, idFirstAttacker, idSecondAttacker, timerForHoldBallForMyIsBallIn
from bridge.strategy.myLogicFunc import buildWallInFrontOfBall, GK
from bridge.strategy.ClassWithMyStaticVariables import GKStates




def TIMEOUT(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int) -> None:
    if field.ally_goal.center_up.y > field.ally_goal.center_down.y:
        actions[const.GK] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(-300, 150), 0)
        actions[idFirstAttacker] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(-300, 400), 0)
        actions[idSecondAttacker] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(-300, 700), 0)
    else:
        actions[const.GK] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(300, 100), 0)
        actions[idFirstAttacker] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(300, 400), 0)
        actions[idSecondAttacker] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(300, 700), 0)


def PREPARE_PENALTY(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int) -> None:
        point_first = aux.Point(const.FIELD_DX / 2 * -field.polarity, const.FIELD_DY / 2)
        point_second = aux.Point(const.FIELD_DX / 2 * -field.polarity, -const.FIELD_DY / 2)
        if we_active:
            actions[const.GK] = Actions.GoToPoint(field.ally_goal.frw, 0)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(aux.Point(200 * field.polarity), (field.ball.get_pos() - field.allies[idSecondAttacker].get_pos()).arg())
                actions[idFirstAttacker] = Actions.GoToPoint(aux.Point(const.FIELD_DX / 2 * field.polarity, const.FIELD_DY / 2), 0)
            elif field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(aux.Point(200 * field.polarity), (field.ball.get_pos() - field.allies[idSecondAttacker].get_pos()).arg())
            else:
                actions[const.GK] = Actions.GoToPoint(aux.Point(200 * field.polarity), (field.ball.get_pos() - field.allies[const.GK].get_pos()).arg())
        else:
            actions[idFirstAttacker] = Actions.GoToPoint(point_first, 0)
            actions[idSecondAttacker] = Actions.GoToPoint(point_second, 0)
            actions[const.GK] = Actions.GoToPoint(field.ally_goal.center, 0)


def PENALTY(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int, GKLastState: GKStates, pointFromBallKicked: Optional[aux.Point], angleFromBallKicked: float) -> GKStates:
    GKNewState: GKStates = GKStates.BlockMaybeKick
    gkId = field.gk_id
    ballPos = field.ball.get_pos()
    point_first = aux.Point(const.FIELD_DX / 2 * -field.polarity, const.FIELD_DY / 2)
    point_second = aux.Point(const.FIELD_DX / 2 * -field.polarity, -const.FIELD_DY / 2)
    if we_active:
        actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
        nearestGoalPoint = aux.closest_point_on_line(field.enemy_goal.center_up, field.enemy_goal.center_down, ballPos)
        point_for_score: Optional[aux.Point] = findPointForScore(field, ballPos, reverse=True, draw=True)
        if aux.dist(ballPos, nearestGoalPoint) > minDistForScorePenalty:
            field.strategy_image.draw_line(ballPos, field.enemy_goal.center)
            if field.allies[idSecondAttacker].is_used():
                # actions[idSecondAttacker] = Actions.Kick(voltage=2, target_pos=ballPos+nearestGoalPoint)
                actions[idSecondAttacker] = Actions.DelayedSlowKick(voltage=2, target_pos=nearestGoalPoint, timerForHoldBallForMyIsBallIn=0)
                # actions[idSecondAttacker] = Actions.DelayedSlowKick(voltage=2, target_pos=ballPos+field.ally_goal.eye_forw*200, timerForHoldBallForMyIsBallIn=0)
                # actions[idSecondAttacker] = Actions.DelayedSlowKick(voltage=2, target_pos=ballPos+nearestGoalPoint, timerForHoldBallForMyIsBallIn=timerForHoldBallForMyIsBallIn/4)
            elif field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.DelayedSlowKick(voltage=2, target_pos=nearestGoalPoint, timerForHoldBallForMyIsBallIn=0)
            else:
                actions[const.GK] = Actions.DelayedSlowKick(voltage=2, target_pos=nearestGoalPoint, timerForHoldBallForMyIsBallIn=0)
        elif point_for_score is not None:
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.DelayedSlowKick(point_for_score, timerForHoldBallForMyIsBallIn=0)
            elif field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.DelayedSlowKick(point_for_score, timerForHoldBallForMyIsBallIn=0)
            else:
                actions[const.GK] = Actions.DelayedSlowKick(point_for_score, timerForHoldBallForMyIsBallIn=0)

        else:
            new_point_for_score: Optional[aux.Point] = findPointForScore(field, ballPos, OtherK=1)
            if new_point_for_score is not None:
                if field.allies[idSecondAttacker].is_used():
                    actions[idSecondAttacker] = Actions.DelayedSlowKick(new_point_for_score, timerForHoldBallForMyIsBallIn=0)
                elif field.allies[idFirstAttacker].is_used():
                    actions[idFirstAttacker] = Actions.DelayedSlowKick(new_point_for_score, timerForHoldBallForMyIsBallIn=0)
                else:
                    actions[const.GK] = Actions.DelayedSlowKick(new_point_for_score, timerForHoldBallForMyIsBallIn=0)
            else:
                if field.allies[idSecondAttacker].is_used():
                    actions[idSecondAttacker] = Actions.DelayedSlowKick(field.enemy_goal.center, timerForHoldBallForMyIsBallIn=0)
                elif field.allies[idFirstAttacker].is_used():
                    actions[idFirstAttacker] = Actions.DelayedSlowKick(field.enemy_goal.center, timerForHoldBallForMyIsBallIn=0)
                else:
                    actions[const.GK] = Actions.DelayedSlowKick(field.enemy_goal.center, timerForHoldBallForMyIsBallIn=0)
    else:
        GKNewState = GK(field, actions, GKLastState, pointFromBallKicked, angleFromBallKicked)
        actions[idFirstAttacker] = Actions.GoToPoint(point_first, 0)
        actions[idSecondAttacker] = Actions.GoToPoint(point_second, 0)
    return GKNewState


def PREPARE_KICKOFF(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int, GKLastState: GKStates) -> GKStates:
    GKNewState: GKStates = GKStates.BlockMaybeKick
    actions[const.GK] = Actions.GoToPoint(field.ally_goal.frw, 0)
    if we_active:
        if field.allies[idFirstAttacker].is_used():
            actions[idFirstAttacker] = Actions.GoToPoint(aux.Point(200 * field.polarity, 0), (field.ball.get_pos() - field.allies[idSecondAttacker].get_pos()).arg())
            actions[idSecondAttacker] = Actions.GoToPoint(aux.Point(150 * field.polarity, const.GOAL_DY), (field.ball.get_pos() - field.allies[idFirstAttacker].get_pos()).arg())
        elif field.allies[idSecondAttacker].is_used():
            actions[idSecondAttacker] = Actions.GoToPoint(aux.Point(200 * field.polarity, 0), (field.ball.get_pos() - field.allies[idSecondAttacker].get_pos()).arg())
    else:
        buildWallInFrontOfBall(field, actions)
        GKNewState = GK(field, actions, GKLastState)
    return GKNewState


def KICKOFF(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int, GKLastState: GKStates) -> GKStates:
    GKNewState: GKStates = GKStates.BlockMaybeKick
    if we_active:
        if field.allies[idSecondAttacker].is_used() and field.allies[idFirstAttacker].is_used():
            actions[idFirstAttacker] = Actions.DelayedSlowKick(field.allies[idSecondAttacker].get_pos(), is_pass=True)
        else:
            actions[const.GK] = Actions.DelayedSlowKick(field.enemy_goal.center)
    else:
        buildWallInFrontOfBall(field, actions)
        GKNewState = GK(field, actions, GKLastState)
    return GKNewState

def STOP(field: fld.Field, actions: list[Optional[Action]], GKLastState: GKStates) -> GKStates:
    buildWallInFrontOfBall(field, actions)
    GKNewState = GK(field, actions, GKLastState)
    return GKNewState

