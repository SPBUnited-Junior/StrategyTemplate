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
from bridge.strategy.flags import Robot_Status

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
            self.kick_status = kick_status

        def push(self, robot: rbt.Robot) -> None:
            is_ally: bool = robot.color == self.field.ally_color

            if (not is_ally):
                RuntimeError("In Role Attacker push enemy robot")
            if (robot.r_id == const.GK):
                RuntimeError("In Role Attacker push GK")
            self.attacker = robot

        def process(self) -> None:
            # print("attacker Role: ", self.attacker)
            # print(kick_status)
            if self.attacker is None: return
            if (len(self.field.pass_points) == 0): return
        
            pass_point: aux.Point = self.field.pass_points[0]
            ball_pos = self.field.ball.get_pos()
            angle_nearest_robot = (ball_pos - self.attacker.get_pos()).arg()
            optimal_point: aux.Point = self.field.pass_points[0]
            # for rbt in self.field.active_allies(False):
            #     if (self.attacker == rbt): continue
            #     optimal_point = rbt.get_pos()
            voltage = get_pass_voltage(aux.dist(ball_pos, optimal_point))
            point_kick_goal : Optional[aux.Point] = check_goal_point(self.field, ball_pos)[0]
            if kick_status[self.attacker.r_id] == Robot_Status.Goal_Turn_Kick  and self.field.is_ball_in_ally_robot():
                """
                Если робот захватил мяч и бьет в ворота с Turn
                """
                if point_kick_goal is None:
                    kick_status[self.attacker.r_id] = Robot_Status.Pass_Turn_Kick
                    self.actions[self.attacker.r_id] = KickActions.Turn_Kick(self.field.enemy_goal.center, angle_nearest_robot, voltage)
                else:
                    self.actions[self.attacker.r_id] = KickActions.Turn_Kick(point_kick_goal, angle_nearest_robot)

            elif kick_status[self.attacker.r_id] == Robot_Status.Goal_Straight and self.field.is_ball_in_ally_robot():
                """
                Если робот захватил мяч и бьет в ворота с Straight
                """
                angle = (optimal_point - ball_pos).arg()
                diff_angle = aux.wind_down_angle(angle - self.attacker.get_angle())
                if (point_kick_goal is None):
                    self.actions[self.attacker.r_id] = KickActions.Straight(self.field.enemy_goal.center)
                else:
                    self.actions[self.attacker.r_id] = KickActions.Straight(point_kick_goal)

                if(not aux.point_nearest_to_goal_hull(ball_pos) and point_kick_goal is None):
                    kick_status[self.attacker.r_id] = Robot_Status.Pass_Turn_Kick

            elif kick_status[self.attacker.r_id] == Robot_Status.Pass_Straight and self.field.is_ball_in_ally_robot():
                """
                Если робот захватил мяч и бьет пасс с Straight
                """
                angle = (optimal_point - ball_pos).arg()
                diff_angle = aux.wind_down_angle(angle - self.attacker.get_angle())
                if (diff_angle > 0.2 and not aux.point_nearest_to_goal_hull(ball_pos)):
                    kick_status[self.attacker.r_id] = Robot_Status.Pass_Turn_Kick
                self.actions[self.attacker.r_id] = KickActions.Straight(optimal_point, voltage)
            
            elif kick_status[self.attacker.r_id] == Robot_Status.Pass_Turn_Kick and self.field.is_ball_in_ally_robot():
                """
                Если робот захватил мяч и бьет пасс с Turn
                """
                self.actions[self.attacker.r_id] = KickActions.Turn_Kick(optimal_point, angle_nearest_robot, voltage)

            else:
                """
                Если робот не бьет  мяч
                """
                if ((kick_status[self.attacker.r_id] == Robot_Status.Pass_Straight or kick_status[self.attacker.r_id] == Robot_Status.Pass_Turn_Kick) 
                    and self.field.check_cath_ball(optimal_point) and not self.field.is_ball_in_ally_robot()):

                    arg = (ball_pos - self.attacker.get_pos()).arg()
                    if (len(self.field.pass_points) >= 1):
                        self.actions[self.attacker.r_id] = Actions.GoToPoint(self.field.pass_points[1], arg)

                elif point_kick_goal is not None and aux.dist(self.attacker.get_pos(), self.field.enemy_goal.center) < 3500:
                    angle = (point_kick_goal - ball_pos).arg()
                    diff_angle = aux.wind_down_angle(angle - self.attacker.get_angle())

                    if (aux.point_nearest_to_goal_hull(ball_pos) or
                        (aux.dist(self.attacker.get_pos(), ball_pos) > 450 or diff_angle < 0.4 
                        and (aux.dist(self.attacker.get_pos(), ball_pos) > 150 or kick_status[self.attacker.r_id] == Robot_Status.Goal_Straight))
                    ):
                        kick_status[self.attacker.r_id] = Robot_Status.Goal_Straight
                        self.actions[self.attacker.r_id] = KickActions.Straight(point_kick_goal)
                    else:
                        kick_status[self.attacker.r_id] = Robot_Status.Goal_Turn_Kick
                        self.actions[self.attacker.r_id] = KickActions.Turn_Kick(self.field.enemy_goal.center, angle_nearest_robot)
                else:

                    angle = (optimal_point - ball_pos).arg()
                    diff_angle = aux.wind_down_angle(angle - self.attacker.get_angle())

                    if (aux.point_nearest_to_goal_hull(ball_pos) or
                        (aux.dist(self.attacker.get_pos(), ball_pos) > 450 or diff_angle < 0.4 
                        and (aux.dist(self.attacker.get_pos(), ball_pos) > 150 or kick_status[self.attacker.r_id] == Robot_Status.Goal_Straight))
                    ):

                        kick_status[self.attacker.r_id] = Robot_Status.Pass_Straight
                        self.actions[self.attacker.r_id] = KickActions.Straight(optimal_point, voltage)
                    else:

                        kick_status[self.attacker.r_id] = Robot_Status.Pass_Turn_Kick
                        self.actions[self.attacker.r_id] = KickActions.Turn_Kick(optimal_point, angle_nearest_robot, voltage)
            self.field.strategy_image.draw_circle(optimal_point, (255, 0, 0), 100)
            #print(kick_status[self.attacker.r_id])


    class Goalkeper:
    
        def __init__(self, 
                field: fld.Field,
                actions: list[Optional[Action]],
            ) -> None:
            
            self.actions: list[Optional[Action]] = actions
            self.field: fld.Field = field
            self.goalkeeper: rbt.Robot = field.allies[const.GK]
            
            # Параметры движения вратаря (только по Y, строго по центру)
            self.sweep_distance = 100  # Расстояние от центра вверх/вниз
            self.sweep_speed = 500     # Скорость движения
            self.last_sweep_change = 0.0
        
        def process(self) -> None:
            current_time = time()
            robot_pos = self.goalkeeper.get_pos()
            voltage_kik = 5
            robot_position_goalkeeper = self.goalkeeper.get_pos()
            robot_position_goalkeeper_enemy = self.field.enemies[const.ENEMY_GK].get_pos()
            ball = aux.Point(self.field.ball.get_pos().x, self.field.ball.get_pos().y)
    
            g_up_xy_goal = self.field.enemy_goal.up - self.field.enemy_goal.eye_up * 40
            g_down_xy_goal = self.field.enemy_goal.down + self.field.enemy_goal.eye_up * 40
    
            up_goal = (g_up_xy_goal - robot_position_goalkeeper_enemy).mag()
            down_goal = (robot_position_goalkeeper_enemy + g_down_xy_goal).mag()
    
            if up_goal > down_goal:
                goal_position_gates = g_up_xy_goal
            else:
                goal_position_gates = g_down_xy_goal
            
            angle_goal_ball = (goal_position_gates - robot_position_goalkeeper).arg()
    
            # Определяем позицию для вратаря
            if self.field.ball_start_point is not None:
                goal_position = aux.closest_point_on_line(self.field.ball_start_point, ball, robot_position_goalkeeper, "R")
            else:
                goal_position = self.field.ally_goal.center
    
            position_goal = (aux.is_point_inside_poly(goal_position, self.field.ally_goal.hull) and self.field.ball.get_vel().mag() > 50)

            if position_goal == False:
                # Центр ворот
                center_x = self.field.ally_goal.center.x
                center_y = self.field.ally_goal.center.y
                elapsed = current_time - self.last_sweep_change
                t = (elapsed * self.sweep_speed / self.sweep_distance) % 4
                if t < 2:
                    # Движение вверх
                    offset = -self.sweep_distance + (t / 2) * (2 * self.sweep_distance)
                else:
                    # Движение вниз
                    offset = self.sweep_distance - ((t - 2) / 2) * (2 * self.sweep_distance)
                
                # Целевая позиция
                goal_position = aux.Point(center_x, center_y + offset)
                angle_goalkeeper = (ball - robot_position_goalkeeper).arg()
                
                self.actions[const.GK] = Actions.GoToPoint(goal_position, angle_goalkeeper)
                kick_status[const.GK] = Robot_Status.Not_Kick
                
                self.field.strategy_image.draw_circle(goal_position, (255, 0, 0), 5)
                self.field.strategy_image.draw_line(
                    aux.Point(center_x, center_y - self.sweep_distance),
                    aux.Point(center_x, center_y + self.sweep_distance),
                    (0, 255, 0),
                    3
                )
                # Отмечаем центр ворот
                self.field.strategy_image.draw_circle(aux.Point(center_x, center_y), (255, 255, 0), 5)
                
            else:
                # Обычное движение к позиции
                angle_goalkeeper = (ball - robot_position_goalkeeper).arg()
                kick_status[const.GK] = Robot_Status.Not_Kick
                self.actions[const.GK] = Actions.GoToPoint(goal_position, angle_goal_ball)
            
            # Удары по мячу
            if self.field.is_ball_stop_near_goal():
                kick_status[const.GK] = Robot_Status.Kick_in_goal_hull
                self.actions[const.GK] = KickActions.Straight(goal_position_gates, voltage_kik, False, True)
            
            if self.field.is_ball_in(self.field.allies[const.GK]):
                kick_status[const.GK] = Robot_Status.Kick_in_goal_hull
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
                    kick_status[ally_rbt.r_id] = Robot_Status.Not_Kick
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
            #print("Block pass Role: ", *self.ally_robots)
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

        def _circle_to_two_tangents(
            self, radius: float, point: aux.Point, point1: aux.Point, point2: aux.Point, robot: aux.Point
            ) -> aux.Point:
            """
            Вычисляет точку на окружности между двумя касательными.
            Добавлена проверка на деление на ноль при вычислении синуса.
            """
            if point1.y > point2.y:
                lower_point = point2
                top_point = point1
            else:
                lower_point = point1
                top_point = point2
            angle = aux.get_angle_between_points(top_point, point, lower_point) / 2
            sin_val = math.sin(angle) if abs(math.sin(angle)) > 1e-6 else 1e-6
            center = lower_point - point
            center = center.unity() * (radius / abs(sin_val))
            center = aux.rotate(center, -angle)
            return aux.closest_point_on_line(aux.Point(point.x - 100, point.y), center + point, robot, "S") #center + point  # Используем point как исходную точку (аналог ball в оригинале)

        def process(self) -> None:
            if (len(self.ally_robots) == 0): return
            #print("Defer Role: ", *self.ally_robots)

            ball_pos = self.field.ball.get_pos()
            for rbt in self.ally_robots:
                pos = self._circle_to_two_tangents(80, ball_pos, self.field.ally_goal.down, self.field.ally_goal.up, rbt.get_pos())
                kick_status[rbt.r_id] = Robot_Status.Not_Kick
                self.actions[rbt.r_id] = Actions.GoToPoint(pos, (ball_pos - rbt.get_pos()).arg())

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
        
        def process(self) -> None:
            if (len(self.field.pass_points) == 0): return
            #print("Pass Role: ", *self.ally_robots)
            ball_pos = self.field.ball.get_pos()
            idx : int = 0
            is_catch : bool = False
            for robot in self.ally_robots:

                if (not is_catch and self.field.check_cath_ball(robot.get_pos()) and check_status_not_kick(self.field)):
                    pos = aux.closest_point_on_line(self.field.ball_start_point, ball_pos, robot.get_pos(), "R")
                    kick_status[robot.r_id] = Robot_Status.Not_Kick
                    self.actions[robot.r_id] = Actions.CatchBall(pos, (ball_pos - robot.get_pos()).arg(), 12)
                    is_catch = True
                
                else:
                    if (idx >= len(self.field.pass_points)): return
                    pos = self.field.pass_points[idx]
                    kick_status[robot.r_id] = Robot_Status.Not_Kick
                    self.actions[robot.r_id] = Actions.GoToPoint(pos, (ball_pos - robot.get_pos()).arg())

                idx+=1



def check_status_not_kick(
        field: fld.Field,
    ) -> bool:
    for rbt in field.active_allies(True):
        if (kick_status[rbt.r_id] != Robot_Status.Not_Kick):
            return False
    return True

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