import math  # type: ignore
from time import time  # type: ignore
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.router.base_actions import Action, Actions, KickActions  # type: ignore
from bridge.strategy.myFunc import findPointForScore


def TIMEOUT(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int) -> None:
    if len(field.active_allies(True)) > 0:
        one = field.gk_id
        if field.ally_goal.center_up.y > field.ally_goal.center_down.y:
            actions[one] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(0, 100), 0)
            if field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(0, 250), 0)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(0, 400), 0)
        else:
            actions[one] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(0, 100), 0)
            if field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(0, 400), 0)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(0, 700), 0)


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
                    aux.Point(200 * -const.POLARITY), (field.ball.get_pos() - atacker_second_pos).arg()
                )
                # pass
        else:
            # code for GK, its for time:
            if field.allies[gkId].is_used():
                actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            # ////
            if field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(point_first, 0)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(point_second, 0)
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)


def PENALTY(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int) -> None:
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
            field.allies[idSecondAttacker].get_pos()
            field.allies[idSecondAttacker]
        ball = field.ball.get_pos()

        point_first = aux.Point(const.FIELD_DX / 2 * -field.polarity, const.FIELD_DY / 2)
        point_second = aux.Point(const.FIELD_DX / 2 * -field.polarity, -const.FIELD_DY / 2)
        if we_active:
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            if field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(-point_first, 0)
            point_for_score: Optional[aux.Point] = findPointForScore(field, ball)
            if point_for_score is not None and field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.Kick(point_for_score)

        else:
            # code for GK, its for time:
            actions[gkId] = Actions.GoToPoint(field.ally_goal.center, 0)
            # ////
            if field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(point_first, 0)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(point_second, 0)
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)


def PREPARE_KICKOFF(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int) -> None:
    if len(field.active_allies(True)) > 0:
        gkId = field.gk_id
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
                actions[idFirstAttacker] = Actions.GoToPoint(aux.Point(150 * const.POLARITY, const.GOAL_DX/3), 0)
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(
                    aux.Point(200 * const.POLARITY), (field.ball.get_pos() - field.allies[idSecondAttacker].get_pos()).arg()
                )
        else:
            y = 130
            x = 300
            if field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(
                    aux.Point(x, y), (field.ball.get_pos() - field.allies[idFirstAttacker].get_pos()).arg()
                )
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(
                    aux.Point(x, -y), (field.ball.get_pos() - field.allies[idSecondAttacker].get_pos()).arg()
                )
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)

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


def KICKOFF(field: fld.Field, actions: list[Optional[Action]], we_active: bool, idFirstAttacker: int, idSecondAttacker: int) -> None:
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
                    actions[idSecondAttacker] = Actions.Kick(point_to_kick)
            else:
                if field.allies[idSecondAttacker].is_used() and field.allies[idFirstAttacker].is_used() and idSecondAttacker != idFirstAttacker:
                    actions[idSecondAttacker] = Actions.Kick(field.allies[idFirstAttacker].get_pos(), is_pass=True)
                elif field.allies[idSecondAttacker].is_used() and field.allies[idFirstAttacker].is_used() and idSecondAttacker == idFirstAttacker:
                    actions[idSecondAttacker] = Actions.Kick(field.allies[gkId].get_pos(), is_pass=True)
                elif field.allies[idFirstAttacker].is_used():
                    actions[idFirstAttacker] = Actions.Kick(field.allies[gkId].get_pos(), is_pass=True)
                else:
                    actions[gkId] = Actions.Kick(field.enemy_goal.frw)
        else:
            y = 130
            x = math.sqrt(600 * 600 - y * y) * -const.POLARITY
            if idFirstAttacker != idSecondAttacker and field.allies[idFirstAttacker].is_used():
                actions[idFirstAttacker] = Actions.GoToPoint(
                    aux.Point(x, y), (field.ball.get_pos() - field.allies[idFirstAttacker].get_pos()).arg()
                )
            if field.allies[idSecondAttacker].is_used():
                actions[idSecondAttacker] = Actions.GoToPoint(
                    aux.Point(x, -y), (field.ball.get_pos() - field.allies[idSecondAttacker].get_pos()).arg()
                )
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
