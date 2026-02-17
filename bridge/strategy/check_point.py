import math  # type: ignore
from time import time  # type: ignore
from typing import Optional
from enum import Enum

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, get_pass_voltage  # type: ignore

def quality_point(
    field: fld.Field,
    point: aux.Point,
    kick_goal_point: None | aux.Point = None
) -> float:
    ball = field.ball.get_pos()

    if point_in_goal(field, point): 
        return 0
    if block_kick_goal(field, point, kick_goal_point):
        return 0
    if nearest_to_ball(field, point):
        return 0
    
    """
    коэффицент возможности блокировки пасса вражеским роботом 
    """
    block_weight: float = 3000
    for rbt in field.active_enemies(False):
        dist_to_point = (rbt.get_pos() - aux.closest_point_on_line(ball, point, rbt.get_pos(), "S")).mag()
        block_weight = min(block_weight, dist_to_point * 3)
    """
    коэффициент возможностьи ударить в ворота
    """
    kick_to_goal_weight: float = check_goal_point(field, point)[1] * 9

    """
    сумма весов
    """
    weight: float = block_weight + kick_to_goal_weight
    return weight

def point_in_goal(
    field: fld.Field,
    point: aux.Point,
) -> bool:
    """
    Проверка на то что точка находиться не в воротах
    """
    ball = field.ball.get_pos()

    if aux.is_point_inside_poly(ball, field.ally_goal.hull): return True
    if aux.is_point_inside_poly(ball, field.enemy_goal.hull): return True
    return False

def block_kick_goal(
    field: fld.Field,
    point: aux.Point,
    kick_goal_point: None | aux.Point
) -> bool:
    """
    Проверка на то что точка не стоит на траектории удара в ворота
    """
    ball = field.ball.get_pos()
    if kick_goal_point is None: return False
    return aux.dist(point, aux.closest_point_on_line(ball, kick_goal_point, point)) < const.MIN_DIST_FROM_KICK

def nearest_to_ball(
    field: fld.Field,
    point: aux.Point,
) -> bool:
    """
    Проверка на тол что точка не слишком близко к мячу
    """
    ball = field.ball.get_pos()
    return aux.dist(ball, point) < const.MIN_PASS_DIST

def check_goal_point(
    field: fld.Field,
    ball: aux.Point
) -> tuple[aux.Point | None, float]:
    """
    Строим косательные к вражеским роботам от ball
    строим отрезки в воротах которые защищены вражескими роботами
    Нахордим максимально большой по длинне, не защищенный отрезок в вражеских воротах
    и бъем в его центр
    """


    # Расчёт касательных к вражеским роботам
    result_cords = []
    for rbt in field.active_enemies(True):
        enemy = rbt.get_pos()
        tangent_points = aux.get_tangent_points(enemy, ball, const.ROBOT_R + aux.dist(ball, enemy) * 0.03)
        if len(tangent_points) >= 2:
            cords_peresch = []
            for count in range(2):
                result = aux.get_line_intersection(
                    ball,
                    tangent_points[count],
                    field.enemy_goal.center_up,
                    field.enemy_goal.center_down,
                    "RL",
                )
                if result is not None:
                    cords_peresch.append(result.y)
            if len(cords_peresch) > 1:
                result_cords.append(sorted(cords_peresch))

    # for cordes in result_cords:
    #     field.strategy_image.draw_line(ball, aux.Point(field.enemy_goal.up.x, cordes[0]), (255, 0, 0), 3)
    #     field.strategy_image.draw_line(ball, aux.Point(field.enemy_goal.up.x, cordes[1]), (255, 0, 0), 3)
    #     field.strategy_image.draw_line(
    #         aux.Point(field.enemy_goal.up.x, cordes[0]),
    #         aux.Point(field.enemy_goal.up.x, cordes[1]),
    #         (255, 0, 0),
    #         3,
    #     )
    
    result_cords = sorted(result_cords)
    maximum: float = 0

    if (const.POLARITY == -1 and const.COLOR == const.Color.BLUE) or (
        const.POLARITY == 1 and const.COLOR == const.Color.YELLOW
    ):
        field_up = field.enemy_goal.up
        field_down = field.enemy_goal.down
    else:
        field_down = field.enemy_goal.up
        field_up = field.enemy_goal.down

    mid = None
    left: float = min(result_cords[0][0], field_down.y) if result_cords else field_down.y #раньше был None
    right: float = field_up.y
    count = 0
    arg_attacker = (field.enemy_goal.center - ball).arg()
    while count < len(result_cords):
        if left is not None and (left > right and right >= field_up.y and left <= field_down.y and left - right > 200):
            maximum = left - right
            mid = aux.Point(field_up.x, (left + right) // 2)
            break
        right = min(max(result_cords[count][1], right), field_down.y)
        count += 1
        if count < len(result_cords):
            left = result_cords[count][0]
        if left is None or left > field_down.y:
            left = field_down.y

    if count != 0:
        count -= 1
    while count < len(result_cords) and right < field_down.y:
        if left > right:
            if left > field_down.y:
                left = field_down.y
            if left - right > maximum and left - right > 200 and right >= field_up.y:
                maximum = left - right
                mid = aux.Point(field_up.x, (left + right) // 2)
        right = max(result_cords[count][1], right)
        count += 1
        if count < len(result_cords):
            left = result_cords[count][0]
    if left is None:
        left = field_down.y
    if left <= field_down.y:
        left = field_down.y
        if left - right > maximum and left > right and left - right > 200 and right >= field_up.y:
            maximum = left - right
            mid = aux.Point(field_up.x, (left + right) // 2)

    return mid, maximum
