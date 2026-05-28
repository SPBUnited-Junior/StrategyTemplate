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
                self.actions[self.attacker.r_id] = KickActions.Turn_Kick(optimal_point, angle_nearest_robot, voltage, True)

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


    class Goalkeper(Basic_Role):

        def __init__(
            self,
            field: fld.Field,
            actions: list[Optional[Action]],
        ) -> None:
            super().__init__(field, actions)
            self.gk_id: int = field.gk_id
            self.goalkeeper: rbt.Robot = field.allies[self.gk_id]
            self.points_on_arc: list[aux.Point] = []
            self.current_point_idx: int = 0
            self.direction: int = 1
            self.defend_mode: bool = True
            self.center: aux.Point = aux.Point(0, 0)
            self.radius: float = 0
            self.start_angle: float = 0
            self.end_angle: float = 0
            self.arc_points()

        def arc_points(self, num_points: int = 30) -> None:
            g = self.field.ally_goal

            self.center = (g.up + g.down) / 2

            self.radius = aux.dist(self.center, g.frw)

            field_direction = g.eye_forw

            center_angle = field_direction.arg()

            self.start_angle = center_angle - math.pi / 2
            self.end_angle = center_angle + math.pi / 2

            self.points_on_arc = []
            for i in range(num_points + 1):
                t = i / num_points
                angle = self.start_angle + (self.end_angle - self.start_angle) * t

                x = self.center.x + self.radius * math.cos(angle)
                y = self.center.y + self.radius * math.sin(angle)

                self.points_on_arc.append(aux.Point(x, y))
        def defend_position(self, ball_pos: aux.Point) -> aux.Point:
            goal_center = self.field.ally_goal.center

            ball_dir = ball_pos - goal_center
            if ball_dir.mag() < 0.1:
                ball_dir = aux.Point(1, 0)
            else:
                ball_dir = ball_dir.unity()

            target_angle = ball_dir.arg()
            target_angle = aux.wind_down_angle(target_angle)

            best_point = None
            min_angle_diff = float('inf')

            for point in self.points_on_arc:
                point_angle = (point - self.center).arg()
                point_angle = aux.wind_down_angle(point_angle)

                diff = abs(aux.wind_down_angle(point_angle - target_angle))
                if diff < min_angle_diff:
                    min_angle_diff = diff
                    best_point = point

            if best_point is None:
                best_point = self.points_on_arc[len(self.points_on_arc) // 2]

            self.field.strategy_image.draw_line(goal_center, ball_pos, (255, 255, 0), 2)
            self.field.strategy_image.draw_line(goal_center, best_point, (0, 255, 255), 2)
            self.field.strategy_image.draw_circle(best_point, (255, 0, 0), 20)

            return best_point

        def patrol_position(self) -> aux.Point:
            if not self.points_on_arc:
                return self.field.ally_goal.center

            target_pos = self.points_on_arc[self.current_point_idx]

            robot_pos = self.goalkeeper.get_pos()
            if aux.dist(robot_pos, target_pos) < 50:
                self.current_point_idx += self.direction

                if self.current_point_idx >= len(self.points_on_arc):
                    self.current_point_idx = len(self.points_on_arc) - 2
                    self.direction = -1
                elif self.current_point_idx < 0:
                    self.current_point_idx = 1
                    self.direction = 1

            return target_pos

        def process(self) -> None:
            if not self.points_on_arc:
                self.arc_points()
                return

            ball_pos = self.field.ball.get_pos()
            ball_vel = self.field.ball.get_vel()
            ball_speed = ball_vel.mag()
            robot_pos = self.goalkeeper.get_pos()

            dist_to_goal = aux.dist(ball_pos, self.field.ally_goal.center)
            voltage_kick = 12
            kick_target = self.field.enemy_goal.center

            ball_in_robot = self.field.is_ball_in_ally_robot()
            ball_near_goal = aux.is_point_inside_poly(ball_pos, self.field.ally_goal.hull)
            ball_stopped_near_goal = self.field.is_ball_stop_near_goal()

            if ball_in_robot and ball_near_goal:
                self.actions[self.gk_id] = KickActions.Straight(kick_target, voltage_kick, False, True)
                kick_status[self.gk_id] = Robot_Status.Kick_in_goal_hull
                self.field.strategy_image.draw_circle(robot_pos, (255, 0, 0), 30)
                return

            if ball_stopped_near_goal:
                g_up_xy_goal = self.field.enemy_goal.up - self.field.enemy_goal.eye_up * 40
                g_down_xy_goal = self.field.enemy_goal.down + self.field.enemy_goal.eye_up * 40

                up_goal = aux.dist(g_up_xy_goal, self.field.enemies[self.field.enemy_gk_id].get_pos())
                down_goal = aux.dist(self.field.enemies[self.field.enemy_gk_id].get_pos(), g_down_xy_goal)

                if up_goal > down_goal:
                    kick_target = g_up_xy_goal
                else:
                    kick_target = g_down_xy_goal

                self.actions[self.gk_id] = KickActions.Straight(kick_target, voltage_kick, False, True)
                kick_status[self.gk_id] = Robot_Status.Kick_in_goal_hull
                self.field.strategy_image.draw_circle(robot_pos, (255, 165, 0), 30)
                return

            if dist_to_goal > 2000 and ball_speed < 100:
                self.defend_mode = False
            else:
                self.defend_mode = True

            if self.defend_mode:
                target_pos = self.defend_position(ball_pos)

                if ball_speed > 200:
                    time_to_ball = aux.dist(robot_pos, ball_pos) / max(ball_speed, 0.1)
                    predicted_ball = ball_pos + ball_vel * min(time_to_ball, 0.5)
                    target_pos = self.defend_position(predicted_ball)
                    self.field.strategy_image.draw_circle(predicted_ball, (255, 165, 0), 15)
            else:
                target_pos = self.patrol_position()

            angle_to_ball = (ball_pos - robot_pos).arg()

            self.actions[self.gk_id] = Actions.GoToPoint(target_pos, angle_to_ball)
            kick_status[self.gk_id] = Robot_Status.Not_Kick

            self.field.strategy_image.draw_circle(target_pos, (0, 255, 255), 15)
            self.zone()

        def zone(self) -> None:
            if not self.points_on_arc:
                return

            g = self.field.ally_goal
            self.field.strategy_image.draw_circle(g.up, (0, 255, 0), 8)
            self.field.strategy_image.draw_circle(g.down, (0, 255, 0), 8)
            self.field.strategy_image.draw_circle(g.frw, (255, 0, 0), 8)
            self.field.strategy_image.draw_circle(g.center, (255, 255, 0), 8)
            self.field.strategy_image.draw_circle(self.center, (255, 0, 255), 10)

            last_point = None
            for point in self.points_on_arc:
                self.field.strategy_image.draw_circle(point, (0, 200, 0), 4)
                if last_point is not None:
                    self.field.strategy_image.draw_line(last_point, point, (0, 200, 0), 2)
                last_point = point

            goal_center = self.field.ally_goal.center
            for point in self.points_on_arc[::5]:
                self.field.strategy_image.draw_line(goal_center, point, (100, 100, 100), 1)
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
                if (aux.is_point_inside_poly(pos, self.field.ally_goal.hull)):
                    pos = (self.field.ally_goal.center - ball_pos).unity() * 120
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