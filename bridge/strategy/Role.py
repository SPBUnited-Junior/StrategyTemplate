import math  # type: ignore
from time import time  # type: ignore
from typing import Optional
from enum import Enum

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, get_pass_voltage  # type: ignore
from bridge.strategy.check_point import check_goal_point

class Basic_Role:

    def __init__(self, 
        field: fld.Field,
        actions: list[Optional[Action]],
    ) -> None:
        
        self.actions: list[Optional[Action]] = actions
        self.field: fld.Field = field
        self.ally_robots: list[rbt.Robot] = []
        self.enemy_robots: list[rbt.Robot] = []

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
    
    class Block_Enemy_Pass(Basic_Role):

        def push(self, robot: rbt.Robot) -> None:
            """
            Добавляем робота в роль
            Если добавляем нашего он добавиться в массив защитников
            Если добавить вражеского то он будет в массиве роботов которых мы блокируем
            """

            is_ally: bool = robot.color == self.field.ally_color

            if (not is_ally):
                RuntimeError("In Role Block_Enemy_Pass push enemy robot")
            if (robot.r_id == const.GK):
                RuntimeError("In Role Block_Enemy_Pass push GK")
            self.ally_robots.append(robot)

        def block_pass_point(self, 
            ally_robot: rbt.Robot, 
            enemy_robot: rbt.Robot
        ) -> aux.Point:
            """
            Вычисляет оптимальную точку для блока паса
            """
            
            ball_pos: aux.Point = self.field.ball.get_pos()
            block_point: aux.Point = aux.closest_point_on_line(ball_pos, enemy_robot.get_pos(), ally_robot.get_pos(), "S")

            angle = (ball_pos - block_point).arg()
            if (aux.dist(enemy_robot.get_pos(), block_point) < 200): 
                block_point = enemy_robot.get_pos() + aux.rotate(aux.Point(200, 0), angle)
            if (aux.dist(ball_pos, block_point) < 200):
                dist: float = aux.dist(enemy_robot.get_pos(), ball_pos)
                block_point = enemy_robot.get_pos() + aux.rotate(aux.Point(dist - 200, 0), angle)

            return block_point

        def block_robot(self, 
            idx: int = 0,
            min_dist: float = 1e5,
            max_dist: float = 0,
            used: list[bool] = [False] * 15
        ) -> float:
            """
            (не) Оптимально  распределяет роботов для блокировки пасов
            """

            ball_pos = self.field.ball.get_pos()
            if (idx == len(self.ally_robots)):
                min_dist = max_dist
                return min_dist
            
            enemy_rbt = self.enemy_robots[idx]
            max_dist_old = max_dist
            for ally_rbt in self.ally_robots:
                block_point: aux.Point = self.block_pass_point(ally_rbt, enemy_rbt)
                dist = aux.dist(ally_rbt.get_pos(), block_point)

                if not used[ally_rbt.r_id] and dist < min_dist:
                    max_dist = max(max_dist, dist)
                    used[ally_rbt.r_id] = True
                    angle = (ball_pos - block_point).arg()
                    self.actions[ally_rbt.r_id] = Actions.GoToPoint(block_point, angle)
                    min_dist = self.block_robot(idx + 1, min_dist, max_dist, used)
                    max_dist = max_dist_old
                    used[ally_rbt.r_id] = False
            
            return min_dist
        
        def build_list(self) -> None:
            ball_pos = self.field.ball.get_pos()

            size = len(self.ally_robots)
            active_enemy_robots: list[tuple[float, int]] = []
            for enemy_rbt in self.field.active_enemies(False):
                active_enemy_robots.append((aux.dist(ball_pos, enemy_rbt.get_pos()), enemy_rbt.r_id))

            active_enemy_robots.sort()
            for i in range(1, min(size + 1, len(active_enemy_robots))):
                id: int = active_enemy_robots[i][1]
                self.enemy_robots.append(self.field.enemies[id])
            return


        def process(self) -> None:
            print(*self.field.pass_points)
            self.build_list()
            if (len(self.enemy_robots) == len(self.ally_robots)):
                self.block_robot()
            return

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
    used: list[bool] = [False] * 15
) -> float:
    """
    Распределяет позиции по роботам,
    чтобы они максимально быстро приехали во все точки
    robots - массив роботов
    list_pos - массив позиций в которые должны приехать роботы
    """
    if idx == len(list_pos):
        min_dist = max_dist
        return min_dist

    for rbt in robots:
        dist: float = aux.dist(rbt.get_pos(), list_pos[idx])
        if not used[rbt.r_id] and dist < min_dist:
            max_dist = max(max_dist, dist)
            used[rbt.r_id] = True
            actions[rbt.r_id] = Actions.GoToPoint(list_pos[idx], (list_pos[idx] - rbt.get_pos()).arg())
            min_dist = go_to_position(field, actions, robots, list_pos, idx + 1, min_dist, max_dist)
            used[rbt.r_id] = False
    return min_dist
        
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
    pos: aux.Point = left_well_point + vector_well.unity() * const.ROBOT_R
    
    list_pos.append(pos)
    for i in range(len(robots) - 1):
        pos += vector_well.unity() * interval
        list_pos.append(pos)

    go_to_position(field, actions, robots, list_pos)
    return