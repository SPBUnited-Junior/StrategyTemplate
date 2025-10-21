import math  # type: ignore
from time import time  # type: ignore
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.router.base_actions import Action, Actions, KickActions  # type: ignore
from bridge.strategy.myFunc import findPointForScore


def TIMEOUT(field: fld.Field, actions: list[Optional[Action]], we_active: bool) -> None:
    if len(field.active_allies(True)) > 0:
        one = field.gk_id
        two, three = GetIds(field, actions)
        if field.ally_goal.center_up.y > field.ally_goal.center_down.y:
            actions[one] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(0, 100), 0)
            if two is not None:
                actions[two] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(0, 250), 0)
            if three is not None:
                actions[three] = Actions.GoToPoint(field.ally_goal.center_up + aux.Point(0, 400), 0)
        else:
            actions[one] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(0, 100), 0)
            if two is not None:
                actions[two] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(0, 400), 0)
            if three is not None:
                actions[three] = Actions.GoToPoint(field.ally_goal.center_down + aux.Point(0, 700), 0)


def PREPARE_PENALTY(field: fld.Field, actions: list[Optional[Action]], we_active: bool) -> None:
    if len(field.active_allies(True)) > 0:
        gkId = field.gk_id
        saId, faId = GetIds(field, actions)

        if saId is None and faId is not None:
            saId = faId
        elif faId is None and saId is None:
            saId = gkId

        if faId is None and saId is not None:
            faId = saId
        elif faId is None and saId is None:
            faId = gkId

        if gkId is not None:
            field.allies[gkId].get_pos()
            field.enemies[gkId]
        if faId is not None:
            field.allies[faId].get_pos()
            field.allies[faId]

        if saId is not None:
            atacker_second_pos = field.allies[saId].get_pos()
            field.allies[saId]

        point_first = aux.Point(const.FIELD_DX / 2 * -field.polarity, const.FIELD_DY / 2)
        point_second = aux.Point(const.FIELD_DX / 2 * -field.polarity, -const.FIELD_DY / 2)
        if we_active:
            if gkId is not None:
                actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
                # pass
            if faId is not None:
                actions[faId] = Actions.GoToPoint(aux.Point(const.FIELD_DX / 2 * field.polarity, const.FIELD_DY / 2), 0)
                # pass
            if saId is not None:
                actions[saId] = Actions.GoToPoint(
                    aux.Point(200 * -const.POLARITY), (field.ball.get_pos() - atacker_second_pos).arg()
                )
                # pass
        else:
            # code for GK, its for time:
            if gkId is not None:
                actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            # ////
            if faId is not None:
                actions[faId] = Actions.GoToPoint(point_first, 0)
            if saId is not None:
                actions[saId] = Actions.GoToPoint(point_second, 0)
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)


def PENALTY(field: fld.Field, actions: list[Optional[Action]], we_active: bool) -> None:
    if len(field.active_allies(True)) > 0:
        gkId = field.gk_id
        saId, faId = GetIds(field, actions)

        if saId is None and faId is not None:
            saId = faId
        elif faId is None and saId is None:
            saId = gkId

        if faId is None and saId is not None:
            faId = saId
        elif faId is None and saId is None:
            faId = gkId

        if gkId is not None:
            field.allies[gkId].get_pos()
            field.enemies[gkId]
        if faId is not None:
            field.allies[faId].get_pos()
            field.allies[faId]

        if saId is not None:
            field.allies[saId].get_pos()
            field.allies[saId]
        ball = field.ball.get_pos()

        point_first = aux.Point(const.FIELD_DX / 2 * -field.polarity, const.FIELD_DY / 2)
        point_second = aux.Point(const.FIELD_DX / 2 * -field.polarity, -const.FIELD_DY / 2)
        if we_active:
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            if faId is not None:
                actions[faId] = Actions.GoToPoint(-point_first, 0)
            point_for_score: Optional[aux.Point] = findPointForScore(field, ball)
            if point_for_score is not None and saId is not None:
                actions[saId] = Actions.Kick(point_for_score)

        else:
            # code for GK, its for time:
            actions[gkId] = Actions.GoToPoint(field.ally_goal.center, 0)
            # ////
            if faId is not None:
                actions[faId] = Actions.GoToPoint(point_first, 0)
            if saId is not None:
                actions[saId] = Actions.GoToPoint(point_second, 0)
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)


def PREPARE_KICKOFF(field: fld.Field, actions: list[Optional[Action]], we_active: bool) -> None:
    if len(field.active_allies(True)) > 0:
        gkId = field.gk_id
        saId, faId = GetIds(field, actions)
        actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
        if saId is None and faId is not None:
            saId = faId
        elif faId is None and saId is None:
            saId = gkId

        if faId is None and saId is not None:
            faId = saId
        elif faId is None and saId is None:
            faId = gkId
        print(faId, saId, gkId)
        if we_active:
            if faId is not None:
                actions[faId] = Actions.GoToPoint(aux.Point(600 * -const.POLARITY), 0)
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            if saId is not None:
                actions[saId] = Actions.GoToPoint(
                    aux.Point(200 * -const.POLARITY), (field.ball.get_pos() - field.allies[saId].get_pos()).arg()
                )
        else:
            y = 130
            x = math.sqrt(600 * 600 - y * y) * -const.POLARITY
            if faId is not None:
                actions[faId] = Actions.GoToPoint(
                    aux.Point(x, y), (field.ball.get_pos() - field.allies[faId].get_pos()).arg()
                )
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)
            if saId is not None:
                actions[saId] = Actions.GoToPoint(
                    aux.Point(x, -y), (field.ball.get_pos() - field.allies[saId].get_pos()).arg()
                )
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)

            # points_first = aux.get_tangent_points(field.allies[faId].get_pos(), field.allies[gkId].get_pos() + aux.Point(0, 100), 100.0)
            # if points_first[0].y < points_first[1].y:
            #     point_first = points_first[0]
            # else:
            #     point_first = points_first[1]

            # points_first = aux.get_tangent_points(field.allies[saId].get_pos(), field.allies[gkId].get_pos() + aux.Point(0, -100), 100.0)
            # if points_first[0].y < points_first[1].y:
            #     point_second = points_first[1]
            # else:
            #     point_second = points_first[0]

            # field.strategy_image.draw_line(field.ball.get_pos(), field.allies[gkId].get_pos() + aux.Point(0, -100) - (field.ball.get_pos() - field.allies[gkId].get_pos() + aux.Point(0, -100)) * 100)
            # field.strategy_image.draw_line(field.ball.get_pos(), field.allies[gkId].get_pos() + aux.Point(0, 100) - (field.ball.get_pos() - field.allies[gkId].get_pos() + aux.Point(0, 100)) * 100)
            # field.strategy_image.draw_line(field.ball.get_pos(), point_first + (field.allies[faId].get_pos() - field.ball.get_pos() + aux.Point(0, -100)) * 100, (0, 0, 0))
            # field.strategy_image.draw_line(field.ball.get_pos(), point_second + (field.allies[saId].get_pos() - field.ball.get_pos() + aux.Point(0, 100)) * 100, (0, 0, 0))


def KICKOFF(field: fld.Field, actions: list[Optional[Action]], we_active: bool) -> None:
    if len(field.active_allies(True)) > 0:
        gkId = field.gk_id
        saId, faId = GetIds(field, actions)

        if saId is None and faId is not None:
            saId = faId
        elif faId is None and saId is None:
            saId = gkId

        if faId is None and saId is not None:
            faId = saId
        elif faId is None and saId is None:
            faId = gkId
        if we_active:
            if saId is not None:
                point_to_kick = findPointForScore(field, field.allies[saId].get_pos())
            if point_to_kick is not None:
                if saId is not None:
                    actions[saId] = Actions.Kick(point_to_kick)
            else:
                if saId is not None and faId is not None and saId != faId:
                    actions[saId] = Actions.Kick(field.allies[faId].get_pos(), is_pass=True)
                elif saId is not None and faId is not None and saId == faId:
                    actions[saId] = Actions.Kick(field.allies[gkId].get_pos(), is_pass=True)
                elif faId is not None:
                    actions[faId] = Actions.Kick(field.allies[gkId].get_pos(), is_pass=True)
                else:
                    actions[gkId] = Actions.Kick(field.enemy_goal.frw)
        else:
            y = 130
            x = math.sqrt(600 * 600 - y * y) * -const.POLARITY
            if faId != saId and faId is not None:
                actions[faId] = Actions.GoToPoint(
                    aux.Point(x, y), (field.ball.get_pos() - field.allies[faId].get_pos()).arg()
                )
            if saId is not None:
                actions[saId] = Actions.GoToPoint(
                    aux.Point(x, -y), (field.ball.get_pos() - field.allies[saId].get_pos()).arg()
                )
            actions[gkId] = Actions.GoToPoint(field.ally_goal.frw, 0)


def GetIds(field: fld.Field, actions: list[Optional[Action]]) -> tuple[Optional[int], Optional[int]]:
    """
    Возвращает id роботов

    id:
        1: вратарь
        2: первый атак.
        3: второй атак.
    """
    mass_idx: Optional[list]
    mass_idx = [robot.r_id for robot in field.active_allies()]
    gk = field.gk_id
    for bot in mass_idx:
        if mass_idx == gk:
            mass_idx.remove(bot)
            break
    need_none = 2 - len(mass_idx)
    for i in range(0, need_none):
        mass_idx.append(None)
    return mass_idx[0], mass_idx[1]
