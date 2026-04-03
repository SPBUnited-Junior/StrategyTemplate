import math  # type: ignore
from time import time  # type: ignore
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.router.base_actions import Action, Actions, KickActions  # type: ignore
from bridge.strategy.myFunc import findPointForScore, GK
from bridge.strategy.myConst import minDistForScorePenalty, angleBetweenRsInWall, timerForRotate



def TIMEOUT(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int) -> None:
    if len(field.active_allies(True)) > 0:
        if field.ally_goal.center_up.y > field.ally_goal.center_down.y:
            actions[const.GK] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(-300, 150), 0)
            if field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(-300, 400), 0)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(-300, 700), 0)
        else:
            actions[const.GK] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(300, 100), 0)
            if field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(300, 400), 0)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(300, 700), 0)


def PREPARE_PENALTY(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int) -> None:
    if len(field.active_allies(True)) > 0:
        gkId = field.gk_id

        if not field.allies[idSecondAttacker].is_used() and field.allies[idFirstAttacker].is_used():
            idSecondAttacker = idFirstAttacker
        elif not field.allies[idFirstAttacker].is_used() and not field.allies[idSecondAttacker].is_used():
            idSecondAttacker = gkId

        if not field.allies[idFirstAttacker].is_used() and field.allies[idSecondAttacker].is_used():
            idFirstAttacker = idSecondAttacker
        elif not field.allies[idFirstAttacker].is_used() and not field.allies[idSecondAttacker].is_used():
            idFirstAttacker = gkId

        if field.allies[gkId].is_used():
            field.allies[gkId].get_pos()
            field.enemies[gkId]
        if field.allies[idFirstAttacker].is_used():
            field.allies[idFirstAttacker].get_pos()
            field.allies[idFirstAttacker]

        if field.allies[idSecondAttacker].is_used():
            atacker_second_pos = field.allies[idSecondAttacker].get_pos()
            field.allies[idSecondAttacker]

        point_first = aux.Point(const.FIELD_DX / 2 * -field.polarity, const.FIELD_DY / 2)
        point_second = aux.Point(const.FIELD_DX / 2 * -field.polarity, -const.FIELD_DY / 2)
        if we_active:
            if field.allies[gkId].is_used():
                actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
                # pass
            if field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(aux.Point(const.FIELD_DX / 2 * field.polarity, const.FIELD_DY / 2), 0)
                # pass
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(
                    aux.Point(200 * field.polarity), (field.ball.get_pos() - atacker_second_pos).arg()
                )
                # pass
        else:
            # code for GK, its for time:
            if field.allies[gkId].is_used():
                actions[gkId] = Actions.GoToPoint(field.ally_goal.center, 0)
            # ////
            if field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(point_first, 0)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(point_second, 0)


def PENALTY(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int, GKLastState: Optional[str]) -> str | None:
    GKNewState = None
    gkId = field.gk_id
    ballPos = field.ball.get_pos()

    point_first = aux.Point(const.FIELD_DX / 2 * -field.polarity, const.FIELD_DY / 2)
    point_second = aux.Point(const.FIELD_DX / 2 * -field.polarity, -const.FIELD_DY / 2)
    if we_active:
        actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
        # if field.allies[idFirstAttacker].is_used():
        #     actions[idFirstAttacker] = Actions.GoToPoint(-point_first, 0)
        point_for_score: Optional[aux.Point] = findPointForScore(field, ballPos, reverse=True, draw=True)
        # print(aux.dist(ballPos, field.enemy_goal.center))
        if aux.dist(ballPos, field.enemy_goal.center) > minDistForScorePenalty:
            field.strategy_image.draw_line(ballPos, field.enemy_goal.center)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.DelayedSlowKick(field.enemy_goal.center, is_pass=True, is_upper=True)
            elif field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.DelayedSlowKick(field.enemy_goal.center, is_pass=True, is_upper=True)
        elif point_for_score is not None:
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.DelayedSlowKick(point_for_score, timer_for_rotate=timerForRotate/2)
            elif field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.DelayedSlowKick(point_for_score, timer_for_rotate=timerForRotate/2)
        else:
            new_point_for_score: Optional[aux.Point] = findPointForScore(field, ballPos, OtherK=1)
            if new_point_for_score is not None:
                if field.allies[idSecondAttacker].is_used():
                    actions[idSecondAttacker] = Actions.DelayedSlowKick(new_point_for_score, timer_for_rotate=timerForRotate/2)
                elif field.allies[idFirstAttacker].is_used():
                    actions[idFirstAttacker] = Actions.DelayedSlowKick(new_point_for_score, timer_for_rotate=timerForRotate/2)
            else:
                if field.allies[idSecondAttacker].is_used():
                    actions[idSecondAttacker] = Actions.DelayedSlowKick(field.enemy_goal.center, timer_for_rotate=timerForRotate/2)
                elif field.allies[idFirstAttacker].is_used():
                    actions[idFirstAttacker] = Actions.DelayedSlowKick(field.enemy_goal.center, timer_for_rotate=timerForRotate/2)

    else:
        GKNewState = GK(field, actions, GKLastState)
        
        if field.allies[idFirstAttacker].is_used():
            actions[idFirstAttacker] = Actions.GoToPoint(point_first, 0)
        if field.allies[idSecondAttacker].is_used():
            actions[idSecondAttacker] = Actions.GoToPoint(point_second, 0)
    return GKNewState


def PREPARE_KICKOFF(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int, GKLastState: Optional[str]) -> str | None:
    GKNewState = None
    if len(field.active_allies(True)) > 0:
        gkId = const.GK
        actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
        if not field.allies[idSecondAttacker].is_used() and field.allies[idFirstAttacker].is_used():
            idSecondAttacker = idFirstAttacker
        elif not field.allies[idFirstAttacker].is_used() and not field.allies[idSecondAttacker].is_used():
            idSecondAttacker = gkId

        if not field.allies[idFirstAttacker].is_used() and field.allies[idSecondAttacker].is_used():
            idFirstAttacker = idSecondAttacker
        elif not field.allies[idFirstAttacker].is_used() and not field.allies[idSecondAttacker].is_used():
            idFirstAttacker = gkId
        # print(idFirstAttacker, idSecondAttacker, gkId)
        if we_active:
            if field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(aux.Point(150 * field.polarity, const.GOAL_DY), (field.ball.get_pos() - field.allies[idFirstAttacker].get_pos()).arg())
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(aux.Point(200 * field.polarity, 0), (field.ball.get_pos() - field.allies[idSecondAttacker].get_pos()).arg())
        else:
            # y = 130
            # x = 300
            # if field.allies[idFirstAttacker].is_used():
            #     actions[idFirstAttacker] = Actions.GoToPoint(
            #         aux.Point(x, y), (field.ball.get_pos() - field.allies[idFirstAttacker].get_pos()).arg()
            #     )
            # actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            # if field.allies[idSecondAttacker].is_used():
            #     actions[idSecondAttacker] = Actions.GoToPoint(
            #         aux.Point(x, -y), (field.ball.get_pos() - field.allies[idSecondAttacker].get_pos()).arg()
            #     )
            vectFromBallToCenter = field.ally_goal.center-field.ball.get_pos()
            if field.allies[idFirstAttacker].is_used() and field.allies[idSecondAttacker].is_used():
                point1 = field.ball.get_pos()+aux.rotate(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50), angleBetweenRsInWall)
                point2 = field.ball.get_pos()+aux.rotate(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50), -angleBetweenRsInWall)
                actions[idFirstAttacker] = Actions.GoToPoint(point1, 0)#TODO fix angle
                actions[idSecondAttacker] = Actions.GoToPoint(point2, 0)#TODO fix angle

                # field.strategy_image.draw_circle(point1, size_in_mms=100)
                # field.strategy_image.draw_circle(point2, size_in_mms=100)
            elif field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(field.ball.get_pos()+(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50)), 0)#TODO fix angle
            else:
                actions[idSecondAttacker] = Actions.GoToPoint(field.ball.get_pos()+(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50)), 0)#TODO fix angle


            # actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            GKNewState = GK(field, actions, GKLastState)

            # points_first = aux.get_tangent_points(field.allies[idFirstAttacker].get_pos(), field.allies[gkId].get_pos() + aux.Point(0, 100), 100.0)
            # if points_first[0].y < points_first[1].y:
            #     point_first = points_first[0]
            # else:
            #     point_first = points_first[1]

            # points_first = aux.get_tangent_points(field.allies[idSecondAttacker].get_pos(), field.allies[gkId].get_pos() + aux.Point(0, -100), 100.0)
            # if points_first[0].y < points_first[1].y:
            #     point_second = points_first[1]
            # else:
            #     point_second = points_first[0]

            # field.strategy_image.draw_line(field.ball.get_pos(), field.allies[gkId].get_pos() + aux.Point(0, -100) - (field.ball.get_pos() - field.allies[gkId].get_pos() + aux.Point(0, -100)) * 100)
            # field.strategy_image.draw_line(field.ball.get_pos(), field.allies[gkId].get_pos() + aux.Point(0, 100) - (field.ball.get_pos() - field.allies[gkId].get_pos() + aux.Point(0, 100)) * 100)
            # field.strategy_image.draw_line(field.ball.get_pos(), point_first + (field.allies[idFirstAttacker].get_pos() - field.ball.get_pos() + aux.Point(0, -100)) * 100, (0, 0, 0))
            # field.strategy_image.draw_line(field.ball.get_pos(), point_second + (field.allies[idSecondAttacker].get_pos() - field.ball.get_pos() + aux.Point(0, 100)) * 100, (0, 0, 0))
    return GKNewState


def KICKOFF(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int, GKLastState: Optional[str]) -> str | None:
    GKNewState = None
    if len(field.active_allies(True)) > 0:
        gkId = field.gk_id

        if not field.allies[idSecondAttacker].is_used() and field.allies[idFirstAttacker].is_used():
            idSecondAttacker = idFirstAttacker
        elif not field.allies[idFirstAttacker].is_used() and not field.allies[idSecondAttacker].is_used():
            idSecondAttacker = gkId

        if not field.allies[idFirstAttacker].is_used() and field.allies[idSecondAttacker].is_used():
            idFirstAttacker = idSecondAttacker
        elif not field.allies[idFirstAttacker].is_used() and not field.allies[idSecondAttacker].is_used():
            idFirstAttacker = gkId
        if we_active:
            if field.allies[idSecondAttacker].is_used():
                point_to_kick = findPointForScore(field, field.allies[idSecondAttacker].get_pos())
            if point_to_kick is not None:
                if field.allies[idSecondAttacker].is_used():
                    actions[idSecondAttacker] = Actions.DelayedSlowKick(point_to_kick)
            else:
                if field.allies[idSecondAttacker].is_used() and field.allies[idFirstAttacker].is_used() and idSecondAttacker != idFirstAttacker:
                    actions[idSecondAttacker] = Actions.DelayedSlowKick(field.allies[idFirstAttacker].get_pos(), is_pass=True)
                elif field.allies[idSecondAttacker].is_used() and field.allies[idFirstAttacker].is_used() and idSecondAttacker == idFirstAttacker:
                    actions[idSecondAttacker] = Actions.DelayedSlowKick(field.allies[gkId].get_pos(), is_pass=True)
                elif field.allies[idFirstAttacker].is_used():
                    actions[idFirstAttacker] = Actions.DelayedSlowKick(field.allies[gkId].get_pos(), is_pass=True)
                else:
                    actions[gkId] = Actions.DelayedSlowKick(field.enemy_goal.frw)
        else:
            vectFromBallToCenter = field.ally_goal.center-field.ball.get_pos()
            if field.allies[idFirstAttacker].is_used() and field.allies[idSecondAttacker].is_used():
                point1 = field.ball.get_pos()+aux.rotate(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50), angleBetweenRsInWall)
                point2 = field.ball.get_pos()+aux.rotate(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50), -angleBetweenRsInWall)
                actions[idFirstAttacker] = Actions.GoToPoint(point1, 0)#TODO fix angle
                actions[idSecondAttacker] = Actions.GoToPoint(point2, 0)#TODO fix angle

                # field.strategy_image.draw_circle(point1, size_in_mms=100)
                # field.strategy_image.draw_circle(point2, size_in_mms=100)
            elif field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(field.ball.get_pos()+(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50)), 0)#TODO fix angle
            else:
                actions[idSecondAttacker] = Actions.GoToPoint(field.ball.get_pos()+(vectFromBallToCenter.unity()*(const.KEEP_BALL_DIST+50)), 0)#TODO fix angle
            # y = 130
            # x = math.sqrt(600 * 600 - y * y) * -const.POLARITY
            # if idFirstAttacker != idSecondAttacker and field.allies[idFirstAttacker].is_used():
            #     actions[idFirstAttacker] = Actions.GoToPoint(
            #         aux.Point(x, y), (field.ball.get_pos() - field.allies[idFirstAttacker].get_pos()).arg()
            #     )
            # if field.allies[idSecondAttacker].is_used():
            #     actions[idSecondAttacker] = Actions.GoToPoint(
            #         aux.Point(x, -y), (field.ball.get_pos() - field.allies[idSecondAttacker].get_pos()).arg()
            #     )
            # actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            GKNewState = GK(field, actions, GKLastState)
    return GKNewState