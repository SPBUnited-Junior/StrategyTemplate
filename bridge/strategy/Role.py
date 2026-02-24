import math  # type: ignore
from time import time  # type: ignore
from typing import Optional
from enum import Enum

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, get_pass_voltage  # type: ignore
from bridge.strategy.check_point import check_goal_point

class Role:
    """
    Роли которые раздаються роботам
    """
    class Attacker:

        def __init__(self) -> None: 
            pass

    class Goalkeper:

        def __init__(self) -> None: 
            pass
    
    class Block_Enemy_Pass:

        def __init__(self) -> None: 
            pass

    class Defer:

        def __init__(self) -> None: 
            pass

    class Pass:

        def __init__(self) -> None: 
            pass


def go_to_position(
    field: fld.Field, 
    actions: list[Optional[Action]], 
    robots: list[rbt.Robot], 
    list_pos: list[aux.Point], 
    idx: int = 0, 
    min_dist: float = 1e5, 
    max_dist: float = 0,
    used: list[int] = [True] * 15
) -> None:
    """
    Распределяет позиции по роботам,
    чтобы они максимально быстро приехали во все точки
    robots - массив роботов
    list_pos - массив позиций в которые должны приехать роботы
    """
    if idx == len(list_pos):
        min_dist = max_dist
        return

    for rbt in robots:
        dist: float = aux.dist(rbt.get_pos(), list_pos[idx])
        if not used[rbt.r_id] and dist < min_dist:
            max_dist = max(max_dist, dist)
            used[rbt.r_id] = True
            actions[rbt.r_id] = Actions.GoToPoint(list_pos[idx], (list_pos[idx] - rbt.get_pos()).arg())
            go_to_position(field, actions, robots, list_pos, idx + 1, min_dist, max_dist)
            used[rbt.r_id] = False
    return
        
def construction_well(
    field: fld.Field, 
    actions: list[Optional[Action]],
    robots: list[rbt.Robot], 
    left_well_point: aux.Point, 
    right_well_point: aux.Point
) -> None:
    """
    Логика для стенки из роботов
    на вход подаеться массив из роботов, из которых будет строиться стенка
    robots - массив роботов
    left_well_point - первая точка стенки
    right_well_point - вторая точка стенки
    """
    list_pos: list[aux.Point] = []
    vector_well: aux.Point = right_well_point - left_well_point
    interval: float = (vector_well.mag() - 2 * const.ROBOT_R) / (len(robots) - 1)
    print(interval, "interval", vector_well.mag())
    pos: aux.Point = left_well_point + vector_well.unity() * const.ROBOT_R
    
    list_pos.append(pos)
    for i in range(len(robots) - 1):
        pos += vector_well.unity() * interval
        list_pos.append(pos)

    go_to_position(field, actions, robots, list_pos)
    return