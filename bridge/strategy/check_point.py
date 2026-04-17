import math  # type: ignore
from time import time  # type: ignore
from typing import Optional
from enum import Enum

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore

def check_goal_point(
    field: fld.Field,
    ball: aux.Point,
    flag_goalkeeper: bool = True
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
        if (not flag_goalkeeper and rbt.r_id == const.ENEMY_GK): continue
        enemy = rbt.get_pos()
        tangent_points = aux.get_tangent_points(enemy, ball, const.ROBOT_R + aux.dist(ball, enemy) * 0.07)
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

    for cordes in result_cords:
        field.strategy_image.draw_line(ball, aux.Point(field.enemy_goal.up.x, cordes[0]), (255, 0, 0), 3)
        field.strategy_image.draw_line(ball, aux.Point(field.enemy_goal.up.x, cordes[1]), (255, 0, 0), 3)
        field.strategy_image.draw_line(
            aux.Point(field.enemy_goal.up.x, cordes[0]),
            aux.Point(field.enemy_goal.up.x, cordes[1]),
            (255, 0, 0),
            3,
        )
    
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

    #print(mid)
    return mid, maximum
