import math  # type: ignore
from time import time  # type: ignore
from typing import Optional
from enum import Enum

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, get_pass_voltage  # type: ignore
from bridge.strategy.check_point import check_goal_point
from bridge.strategy.flags import kick_status
from bridge.strategy.flags import Kick_Status

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

        def __init__(self, 
            field: fld.Field,
            actions: list[Optional[Action]],
        ) -> None:
        
            self.actions: list[Optional[Action]] = actions
            self.field: fld.Field = field
            self.attacker: Optional[rbt.Robot] = None

        def push(self, robot: rbt.Robot) -> None:
            is_ally: bool = robot.color == self.field.ally_color

            if (not is_ally):
                RuntimeError("In Role Attacker push enemy robot")
            if (robot.r_id == const.GK):
                RuntimeError("In Role Attacker push GK")
            self.attacker = robot
        
        def check_cath_ball(self, pas_point: aux.Point) -> bool:
            """
            Проверяем летит ли мяч в сторону робота
            """
            ball_pos: aux.Point = self.field.ball.get_pos()
            pos_cath = aux.closest_point_on_line(self.field.ball_start_point, ball_pos, pas_point, "R")
            self.field.strategy_image.draw_circle(self.field.ball_start_point, (255, 0, 255), 30)

            if(
                (pos_cath is None
                or aux.dist(pos_cath, pas_point) > const.DIST_CATCH_BALL
                or aux.dist(ball_pos, pas_point) > const.DIST_TO_PASS
                or self.field.ball.get_vel().mag() < const.VEL_TO_PASS)
            ):
                #self.passes_status = FlagToPasses.FALSE
                return False
            #self.passes_status = FlagToPasses.TRUE
            return True

        def process(self) -> None:
            if self.attacker is None: return
        
            pass_point: aux.Point = self.field.pass_points[0]
            ball_pos = self.field.ball.get_pos()
            voltage = get_pass_voltage(aux.dist(ball_pos, self.attacker.get_pos()))
            angle_nearest_robot = (ball_pos - self.attacker.get_pos()).arg()
            optimal_point: aux.Point = self.field.pass_points[0]
            point_kick_goal : Optional[aux.Point] = check_goal_point(self.field, ball_pos)[0]
            if kick_status == Kick_Status.Goal_Turn_Kick  and self.field.is_ball_in_ally_robot():
                """
                Если робот захватил мяч и бьет в ворота с Turn
                """
                if point_kick_goal is None:
                    self.kick_status = Kick_Status.Pass_Turn_Kick
                    self.actions[self.attacker.r_id] = KickActions.Turn_Kick(self.field.enemy_goal.center, angle_nearest_robot, voltage)
                else:
                    self.actions[self.attacker.r_id] = KickActions.Turn_Kick(point_kick_goal, angle_nearest_robot)

            elif kick_status == Kick_Status.Goal_Straight and self.field.is_ball_in_ally_robot():
                """
                Если робот захватил мяч и бьет в ворота с Straight
                """
                if point_kick_goal is None:
                    self.actions[self.attacker.r_id] = KickActions.Straight(self.field.enemy_goal.center)
                else:
                    self.actions[self.attacker.r_id] = KickActions.Straight(point_kick_goal)

            elif kick_status == Kick_Status.Pass_Straight and self.field.is_ball_in_ally_robot():
                """
                Если робот захватил мяч и бьет пасс с Straight
                """
                self.actions[self.attacker.r_id] = KickActions.Straight(optimal_point, voltage)
            
            elif kick_status == Kick_Status.Pass_Turn_Kick and self.field.is_ball_in_ally_robot():
                """
                Если робот захватил мяч и бьет пасс с Turn
                """
                self.actions[self.attacker.r_id] = KickActions.Turn_Kick(optimal_point, angle_nearest_robot, voltage)

            else:
                """
                Если робот не бьет  мяч
                """
                if ((kick_status.value == Kick_Status.Pass_Straight or kick_status.value == Kick_Status.Pass_Turn_Kick) 
                    and self.check_cath_ball(optimal_point) and not self.field.is_ball_in_ally_robot()):

                    arg = (ball_pos - self.attacker.get_pos()).arg()
                    self.actions[self.attacker.r_id] = Actions.GoToPoint(self.field.pass_points[1], arg)

                elif point_kick_goal is not None and aux.dist(self.attacker.get_pos(), self.field.enemy_goal.center) < 3500:
                    kick_status.value = Kick_Status.Goal_Turn_Kick
                    self.actions[self.attacker.r_id] = KickActions.Turn_Kick(self.field.enemy_goal.center, angle_nearest_robot)
                else:
                    kick_status.value = Kick_Status.Pass_Turn_Kick
                    self.actions[self.attacker.r_id] = KickActions.Turn_Kick(optimal_point, angle_nearest_robot, voltage)


    class Goalkeper:

        def __init__(self, 
            field: fld.Field,
            actions: list[Optional[Action]],
        ) -> None:
        
            self.actions: list[Optional[Action]] = actions
            self.field: fld.Field = field
            self.goalkeeper: rbt.Robot = field.allies[const.GK]

        def process(self) -> None:
            """
            The logic by which the goalkeeper acts

            includes (it is necessary to list the main points of the goalkeeper's strategy):
            """

            voltage_kik = 5

            robot_position_goalkeeper = self.goalkeeper.get_pos()

            robot_position_goalkeeper_enemy = self.field.enemies[const.ENEMY_GK].get_pos()

            ball = aux.Point(self.field.ball.get_pos().x, self.field.ball.get_pos().y)

            g_up_xy_goal = self.field.enemy_goal.up - self.field.enemy_goal.eye_up * 40    #определяется угол ворот противоположный от враторя
            g_down_xy_goal = self.field.enemy_goal.down + self.field.enemy_goal.eye_up * 40

            up_goal = (g_up_xy_goal - robot_position_goalkeeper_enemy).mag()
            down_goal = (robot_position_goalkeeper_enemy + g_down_xy_goal).mag()

            if up_goal > down_goal:
                goal_position_gates = g_up_xy_goal
            else:
                goal_position_gates = g_down_xy_goal     #закончилось NOTE очень понятно

            angle_goal_ball = (goal_position_gates - robot_position_goalkeeper).arg()
        
            # Определяем позицию для вратаря
            if self.field.ball_start_point is not None:
                goal_position = aux.closest_point_on_line(self.field.ball_start_point, ball, robot_position_goalkeeper, "R")
            else:
                goal_position = self.field.ally_goal.center

            position_goal = aux.is_point_inside_poly(goal_position, self.field.ally_goal.hull) # проверяем находится ли мяч в воротах

            if position_goal == False:  
                goal_position = self.field.ally_goal.center

            angle_goalkeeper = (ball - robot_position_goalkeeper).arg()
        
            self.actions[const.GK] = Actions.GoToPoint(goal_position, angle_goal_ball)

        
            if self.field.is_ball_stop_near_goal():
                #actions[self.gk_idx] = Actions.Kick(goal_position_gates, voltage_kik, is_upper=True)
                self.actions[const.GK] = KickActions.Straight(goal_position_gates, voltage_kik, False, True)

        
            if self.field.is_ball_in(self.field.allies[const.GK]):
                #actions[self.gk_idx] = Actions.Kick(goal_position_gates, voltage_kik,is_upper=True)
                self.actions[const.GK] = KickActions.Straight(goal_position_gates, voltage_kik, False, True)

    
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

    class Defer(Basic_Role):

        def push(self, robot: rbt.Robot) -> None:
            """
            Добавляем робота в роль
            Если добавляем нашего он добавиться в массив защитников
            Если добавить вражеского то он будет в массиве роботов которых мы блокируем
            """

            is_ally: bool = robot.color == self.field.ally_color

            if (not is_ally):
                RuntimeError("In Role Defer push enemy robot")
            if (robot.r_id == const.GK):
                RuntimeError("In Role Defer push GK")
            self.ally_robots.append(robot)

        def process(self) -> None:
            return

    class Pass(Basic_Role):

        def push(self, robot: rbt.Robot) -> None:
            """
            Добавляем робота в роль
            Если добавляем нашего он добавиться в массив защитников
            Если добавить вражеского то он будет в массиве роботов которых мы блокируем
            """

            is_ally: bool = robot.color == self.field.ally_color

            if (not is_ally):
                RuntimeError("In Role Pass push enemy robot")
            if (robot.r_id == const.GK):
                RuntimeError("In Role Pass push GK")
            self.ally_robots.append(robot)

        def check_cath_ball(self, pas_point: aux.Point) -> bool:
            """
            Проверяем летит ли мяч в сторону робота
            """
            ball_pos: aux.Point = self.field.ball.get_pos()
            pos_cath = aux.closest_point_on_line(self.field.ball_start_point, ball_pos, pas_point, "R")
            self.field.strategy_image.draw_circle(self.field.ball_start_point, (255, 0, 255), 30)

            if(
                (pos_cath is None
                or aux.dist(pos_cath, pas_point) > const.DIST_CATCH_BALL
                or aux.dist(ball_pos, pas_point) > const.DIST_TO_PASS
                or self.field.ball.get_vel().mag() < const.VEL_TO_PASS)
            ):
                return False
            return True
        
        
        def process(self) -> None:
            ball_pos = self.field.ball.get_pos()
            idx : int = 0
            for robot in self.ally_robots:

                if (self.check_cath_ball(robot.get_pos())):
                    pos = aux.closest_point_on_line(self.field.ball_start_point, ball_pos, robot.get_pos(), "R")
                    self.actions[robot.r_id] = Actions.CatchBall(pos, (ball_pos - robot.get_pos()).arg(), 8)
                
                else:
                    pos = self.field.pass_points[idx]
                    self.actions[robot.r_id] = Actions.GoToPoint(pos, (ball_pos - robot.get_pos()).arg())

                idx+=1




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