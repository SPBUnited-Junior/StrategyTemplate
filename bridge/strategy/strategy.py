"""High-level strategy code"""

# !v DEBUG ONLY
import math  # type: ignore
from time import time  # type: ignore
from typing import Optional
from enum import Enum

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, get_pass_voltage  # type: ignore

"""
ONE ITERATION of strategy
NOTE: robots will not start acting until this function returns an array of actions,
        if an action is overwritten during the process, only the last one will be executed)

Examples of getting coordinates:
- field.allies[8].get_pos(): aux.Point -   coordinates  of the 8th  robot from the allies
- field.enemies[14].get_angle(): float - rotation angle of the 14th robot from the opponents

- field.ally_goal.center: Point - center of the ally goal
- field.enemy_goal.hull: list[Point] - polygon around the enemy goal area


Examples of robot control:
- actions[2] = Actions.GoToPoint(aux.Point(1000, 500), math.pi / 2)
        The robot number 2 will go to the point (1000, 500), looking in the direction π/2 (up, along the OY axis)

- actions[3] = Actions.Kick(field.enemy_goal.center)
        The robot number 3 will hit the ball to 'field.enemy_goal.center' (to the center of the enemy goal)

- actions[9] = Actions.BallGrab(0.0)
        The robot number 9 grabs the ball at an angle of 0.0 (it looks to the right, along the OX axis)

"""

class FlagToPasses(Enum): #флаги для состояние приянтия мячей
    FALSE = 0 # не ловим 
    TRUE = 1 # ловим
    RELEASE = 2 # поймали и отпускаем
 
class Strategy:
    """Main class of strategy"""

    def __init__(
        self,
    ) -> None:
        self.we_active = False

        # Индексы роботов

        self.goalkeeper_idx = 0
        self.idx1 = 1
        self.idx2 = 2

        # Индексы роботов соперника

        self.goalkeeper_idx_enemy = 0
        self.idx_enemy1 = 1
        self.idx_enemy2 = 2

        # статические переменные
        self.point_kick_goal : aux.Point | None = aux.Point(0, 0) # точки в воротах, в которую будет бить атакующий
        self.dist_line_goal = 0.0 #размер максимально длинного открытого отрезка в воротах

        self.old_ball = aux.Point(0, 0) #положение мяча когда он начал катиться
        self.ball = aux.Point(0, 0) # мяч

        # переменные для паса
        self.dist_to_pas = 2000 # расстояние до удара
        self.dist_cath_ball = 300 #расстояние на которое робот можнт отехать при ловле мяча
        self.passes_status = FlagToPasses.FALSE # флаг состояний
        self.time_stop_dribbler = 0.5 # время для остнаовки дриблера после паса
        self.timer_stop_dribbler = 0.0 #для остановки дриблера
        self.dist_after_catch = 140 # растояние на которое нужно отехать от мяча, после его поимки и остановки дриблера



    def process(self, field: fld.Field) -> list[Optional[Action]]:
        """
        Подсчет статических переменных (self)
        
        """
        print(GameStates)
        field.active_enemies # масив роботов на поле

        #p+9obot_position1.is_used # true на поле

        print("Ваня пипукааааааааааа")
        print('Ваня где рабочий кооооооооооод')
        """Game State Management"""
        voltage_kik = 5 

        robot_position_goalkeeper = field.allies[self.goalkeeper_idx].get_pos()
        robot_position1 = field.allies[self.idx1].get_pos()
        robot_position2 = field.allies[self.idx2].get_pos()

        robot_position_goalkeeper_enemy = field.enemies[self.goalkeeper_idx_enemy].get_pos()
        robot_position1_enemy = field.enemies[self.idx_enemy1].get_pos()
        robot_position2_enemy = field.enemies[self.idx_enemy2].get_pos()

        if field.game_state not in [GameStates.KICKOFF, GameStates.PENALTY]:
            if field.active_team in [const.Color.ALL, field.ally_color]:
                self.we_active = True
            else:
                self.we_active = False

        actions: list[Optional[Action]] = []
        for _ in range(const.TEAM_ROBOTS_MAX_COUNT):
            actions.append(None)


        ### расчет точки в воротах, в которую будет бить атакующий ###
        enemies = []
        for rbt in field.active_enemies(True):
            enemies.append(rbt.get_pos())

        kick_inf_list = self.check_goal_point(
            field,
            field.ball.get_pos(),
            enemies
        )

        self.point_kick_goal = kick_inf_list[0]
        self.dist_line_goal = kick_inf_list[1]

        ### полложение мяча ###
        self.ball = field.ball.get_pos()

        """Game State Management"""
        if field.game_state not in [GameStates.KICKOFF, GameStates.PENALTY]:
            if field.active_team in [const.Color.ALL, field.ally_color]:
                self.we_active = True
            else:
                self.we_active = False

        actions: list[Optional[Action]] = []
        for _ in range(const.TEAM_ROBOTS_MAX_COUNT):
            actions.append(None)

        if field.game_state == GameStates.RUN:
            self.run(field, actions)
            
        elif field.game_state == GameStates.TIMEOUT:
            pass
        elif field.game_state == GameStates.HALT:
            actions[self.idx1] = Actions.Stop()
            actions[self.idx2] = Actions.Stop()
            return actions
           
        elif field.game_state == GameStates.PREPARE_PENALTY and  not self.we_active:
            kik_angle1 = robot_position1_enemy - robot_position1
            kik_angle2 = robot_position2_enemy - robot_position2
            pos_kikoff1 = aux.Point(800 * field.polarity, 0)
            pos_kikoff2 = aux.Point(1200 * field.polarity, 0)
            actions[self.idx1] = Actions.GoToPoint(pos_kikoff1, kik_angle1.arg())
            actions[self.idx2] = Actions.GoToPoint(pos_kikoff2, kik_angle2.arg())
        
        elif field.game_state == GameStates.PREPARE_PENALTY and  self.we_active:
            kik_angle1 = robot_position1_enemy - robot_position1
            kik_angle2 = robot_position2_enemy - robot_position2
            pos_kikoff1 = aux.Point(800 * field.polarity, 0)
            pos_kikoff2 = aux.Point(1200 * field.polarity, 0)

            actions[self.idx1] = Actions.GoToPoint(pos_kikoff1, kik_angle1.arg())
            actions[self.idx2] = Actions.GoToPoint(pos_kikoff2, kik_angle2.arg())       
        
        elif field.game_state == GameStates.PENALTY and not self.we_active:
            position_penalty1 = self.ball
            angle_penalty1 = self.ball - robot_position1
            #ctions[self.idx1] = Actions.GoToPoint(position_penalty1, angle_penalty1.arg())

        elif field.game_state == GameStates.PENALTY and self.we_active:
            g_up_xy_attacker = field.enemy_goal.up - field.enemy_goal.eye_up * 35   #определяется угол ворот противоположный от враторя
            g_down_xy_attacker = field.enemy_goal.down + field.enemy_goal.eye_up * 35

            up_attacker = (g_up_xy_attacker - robot_position_goalkeeper_enemy).mag()
            down_attacker = (robot_position_goalkeeper_enemy - g_down_xy_attacker).mag()

            if up_attacker > down_attacker:
                position_attacker_gate = g_up_xy_attacker
            else:
                position_attacker_gate = g_down_xy_attacker
        
            actions[self.idx1] = Actions.Kick(position_attacker_gate, voltage_kik)

        elif field.game_state == GameStates.PREPARE_KICKOFF:
            kik_angle1 = robot_position1_enemy - robot_position1
            kik_angle2 = robot_position2_enemy - robot_position2

            pos_kikoff1 = aux.Point(500 * field.polarity, 0)   
            pos_kikoff2 = aux.Point(1000 * field.polarity, 0)

            actions[self.idx1] = Actions.GoToPoint(pos_kikoff1, kik_angle1.arg())
            actions[self.idx2] = Actions.GoToPoint(pos_kikoff2, kik_angle2.arg())
            
        elif field.game_state == GameStates.KICKOFF and  not self.we_active:
            kik_angle1 = robot_position1_enemy - robot_position1
            kik_angle2 = robot_position2_enemy - robot_position2
            pos_kikoff1 = aux.Point(500, 0)
            pos_kikoff2 = aux.Point(1000, 0)
            actions[self.idx1] = Actions.GoToPoint(pos_kikoff1, kik_angle1.arg())
            actions[self.idx2] = Actions.GoToPoint(pos_kikoff2, kik_angle2.arg())

        elif field.game_state == GameStates.KICKOFF and self.we_active:
            self.run(field, actions)
            print(456789)

        elif field.game_state == GameStates.FREE_KICK:
            self.run(field, actions)
            pass

        elif field.game_state == GameStates.STOP:
            ball_pos = field.ball.get_pos()
                        
            robot_to_ball1 = ball_pos - robot_position1
            robot_to_ball2 = ball_pos - robot_position2

            robot1_angle = (self.ball - robot_position1).arg()
            robot2_angle = (self.ball - robot_position2).arg() # вратарская зона
                        
            #if robot_to_ball1.mag() > 50:
            #    deltac = 50 + robot_to_ball1.x
            #    pos_stop1 = aux.Point(ball.x + deltac, robot_to_ball1.y)
            #    actions[self.idx1] = Actions.GoToPoint(pos_stop1, robot_to_ball1.arg())
            #else:
            #    
            #    actions[self.idx1] = Actions.Stop()
            #if robot_to_ball2.mag() > 50:
            #    deltac = 30 - robot_to_ball2.y
            #    deltac2 = 50 + robot_to_ball2.x
            #    pos_stop2 = aux.Point(ball.x + deltac2 , robot_to_ball2.y - deltac)
            #    
            #    actions[self.idx2] = Actions.GoToPoint(pos_stop2, robot_to_ball2.arg())
            #else:
            #    actions[self.idx2] = Actions.Stop()
            #
            position = field.ally_goal.center + field.ally_goal.eye_forw * 400
            #position1 = self._defer(robot_pos1, ball, field)
            if aux.dist(position, self.ball) < 500:
                position = (robot_position2 - self.ball).unity() * 500

            actions[self.idx1] = Actions.GoToPoint(robot_position1, robot1_angle)
            actions[self.idx2] = Actions.GoToPoint(robot_position2, robot2_angle)
        
        return actions
        
        return actions

    def run(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        
        if self.passes_status == FlagToPasses.FALSE:
            actions[1] = self.kick_ball_to_pas(field, field.allies[2])
        else:
            actions[1] = Actions.GoToPoint(field.allies[1].get_pos(), field.allies[1].get_angle())
        if self.check_cath_ball(field, field.allies[2]):
            actions[2] = self.process_catch_ball(field, field.allies[2])
        else:
            actions[2] = Actions.GoToPoint(aux.Point(0, 0), (self.ball - field.allies[2].get_pos()).arg())
        
        
        self.process_defender(field, actions)
        print(self.passes_status, field.ball_start_point, field.ball.get_vel().mag())
        self.process_goalkeeper(field,actions)

    def process_goalkeeper(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        """
        The logic by which the goalkeeper acts

        includes (it is necessary to list the main points of the goalkeeper's strategy):
        """
        voltage_kik = 5 

        robot_position_goalkeeper = field.allies[self.goalkeeper_idx].get_pos()
        robot_position1 = field.allies[self.idx1].get_pos()
        robot_position2 = field.allies[self.idx2].get_pos()

        robot_position_goalkeeper_enemy = field.enemies[self.goalkeeper_idx_enemy].get_pos()
        robot_position1_enemy = field.enemies[self.idx_enemy1].get_pos()
        robot_position2_enemy = field.enemies[self.idx_enemy2].get_pos()

        ball = field.ball.get_pos()


        g_up_xy_goal = field.enemy_goal.up - field.enemy_goal.eye_up * 40    #определяется угол ворот противоположный от враторя
        g_down_xy_goal = field.enemy_goal.down + field.enemy_goal.eye_up * 40

        up_goal = (g_up_xy_goal - robot_position_goalkeeper_enemy).mag()
        down_goal = (robot_position_goalkeeper_enemy + g_down_xy_goal).mag()

        if up_goal > down_goal:
            goal_position_gates = g_up_xy_goal
        else:
            goal_position_gates = g_down_xy_goal     #закончилось

        angle_goal_ball = (goal_position_gates - robot_position_goalkeeper).arg()
    
        # Определяем позицию для вратаря
        if field.ball_start_point is not None:
            goal_position = aux.closest_point_on_line(field.ball_start_point, ball, robot_position_goalkeeper, "R")
        else:
            goal_position = field.ally_goal.center

        position_goal = aux.is_point_inside_poly(goal_position, field.ally_goal.hull) # проверяем находится ли мяч в воротах

        if position_goal == False:
            goal_position = field.ally_goal.center

        angle_goalkeeper = (ball - robot_position_goalkeeper).arg()
    
        actions[self.goalkeeper_idx] = Actions.GoToPoint(goal_position, angle_goal_ball)

    
        if field.is_ball_stop_near_goal():
            #actions[self.gk_idx] = Actions.Kick(goal_position_gates, voltage_kik, is_upper=True)
            actions[self.goalkeeper_idx] = KickActions.Straight(goal_position_gates, voltage_kik, False, True)

    
        if field.is_ball_in(field.allies[self.goalkeeper_idx]):
            #actions[self.gk_idx] = Actions.Kick(goal_position_gates, voltage_kik,is_upper=True)
            actions[self.goalkeeper_idx] = KickActions.Straight(goal_position_gates, voltage_kik, False, True)
            

        #waypoints[self.gk_idx] = wp.Waypoint(goal_position, angle_goalkeeper, wp.WType.S_BALL_KICK)
    
        return actions


        pass

    def process_attacker(self, field: fld.Field, attacker: rbt.Robot, robot_catch_ball: rbt.Robot) -> Action:
        """
        The logic by which the attacker acts

        includes (it is necessary to list the main points of the attacker's strategy):
        """
        
        if self.point_kick_goal is None:
            """
            Если вражеские ворота полностью заблокированы или
            мяч слишком далеко для удара по воротам

            даем пас другому роботу
            """

            Action_attacker = self.kick_ball_to_pas(field, robot_catch_ball)
            
        else:
            """
            Если вражеские ворота открыты и 
            мяч достаточно близко для удара

            бъем в ворота
            """

            Action_attacker = self.kick_ball_to_goal(field)

        return Action_attacker
    
    def kick_ball_to_goal(self, field: fld.Field) -> Action:
        """
        Бъем в ворота
        Проверка на то чтобы point not is None 
        """

        point_kick_goal = self.point_kick_goal
        if point_kick_goal is None:
            point_kick_goal = field.enemy_goal.center

        return KickActions.Straight(point_kick_goal)
    
    def kick_ball_to_pas(self, field: fld.Field,  robot_catch_ball: rbt.Robot) -> Action:
        """
        Даем пас роботу
        """
        voltage = get_pass_voltage(aux.dist(self.ball, robot_catch_ball.get_pos()))
        return KickActions.Straight(robot_catch_ball.get_pos(), voltage)
    
    def process_defender(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        """
        The logic by which the attacker acts

        includes (it is necessary to list the main points of the defender's strategy):
        """
        robot_position_goalkeeper = field.allies[self.goalkeeper_idx].get_pos()
        robot_position1 = field.allies[self.idx1].get_pos()
        robot_position2 = field.allies[self.idx2].get_pos()

        robot_position_goalkeeper_enemy = field.enemies[self.goalkeeper_idx_enemy].get_pos()
        robot_position1_enemy = field.enemies[self.idx_enemy1].get_pos()
        robot_position2_enemy = field.enemies[self.idx_enemy2].get_pos()

        #Point1 = aux.circles_inter(robot_position1_enemy, robot_position2_enemy, aux.dist(robot_position1_enemy, robot_position2_enemy), aux.dist(robot_position1_enemy, robot_position2_enemy))[0]
        #Point2 = aux.circles_inter(robot_position1_enemy, robot_position2_enemy, aux.dist(robot_position1_enemy, robot_position2_enemy), aux.dist(robot_position1_enemy, robot_position2_enemy))[1]

        #middle_point = aux.get_line_intersection(robot_position1_enemy, robot_position2_enemy, Point1, Point2, "L")

        ball = field.ball.get_pos()
        nearest_enemy_dist = 5000.0
        nearest_enemy_point = aux.Point(0, 0)

        #определение ближайшего робота врага к мячу
        for i in field.active_enemies(False):
            if aux.dist(i.get_pos(), ball) < nearest_enemy_dist:
                nearest_enemy_dist = aux.dist(i.get_pos(), ball)
                nearest_enemy_point = i.get_pos()
        
        dist_to_robot_with_ball = (ball - nearest_enemy_point).unity() *200 + ball

        bottom_crossbar = field.ally_goal.down + aux.Point(0, 100) #Небольшое расстояние от нижней штанги к углу
        up_crossbar = field.ally_goal.up - aux.Point(0, 100)  #Небольшое расстояние от верхней штанги к углу

        bottom_block = aux.closest_point_on_line(nearest_enemy_point, bottom_crossbar, dist_to_robot_with_ball, "R")
        up_block = aux.closest_point_on_line(nearest_enemy_point, up_crossbar, dist_to_robot_with_ball, "R")

        #Вычисление точки для блокировки удара
        
        if aux.dist(dist_to_robot_with_ball, bottom_block) > aux.dist(dist_to_robot_with_ball, up_block):
            actions[self.idx1] = Actions.GoToPoint(up_block, (ball-robot_position1).arg())
        else:
            actions[self.idx1] = Actions.GoToPoint(bottom_block, (ball-robot_position1).arg())
            if  nearest_enemy_point == aux.Point(0, 0):
                actions[self.idx1] = Actions.GoToPoint(field.ally_goal.center + aux.Point(-1000, 0), 3.14)
        
        #Блокировка паса
        actions[self.idx2] = Actions.GoToPoint(aux.closest_point_on_line(robot_position2_enemy, robot_position1_enemy, robot_position2, "S"), (ball - robot_position2).arg())


    def process_catch_ball(self, field: fld.Field, robot: rbt.Robot) -> Action:
        """
        Логика принятия всех летящих в робота мячей

        Проверка того что мяч летит в робота происходит
        вне функции, с помощью check_cath_a_ball()
        """
        if self.passes_status != FlagToPasses.RELEASE:
            """
            Если мяч ещё летит то
            подстравиваемься под него
            """
            self.passes_status = FlagToPasses.TRUE
            pos = aux.closest_point_on_line(field.ball_start_point, self.ball, robot.get_pos(), "R")
            Action_robot: Action = Actions.CatchBall(pos, (self.ball - robot.get_pos()).arg(), 15)
            if field.is_ball_in(robot):
                """
                Ждем когда мы поймаем мяч
                """
                self.passes_status = FlagToPasses.RELEASE
                self.timer_stop_dribbler = time()
            
        else:
            """
            когда мы поймали мяч
            останавливаем дриблер
            и отезжаем назад от мяча
            """
            Action_robot = Actions.GoToPoint(robot.get_pos(), robot.get_angle())
            if time() - self.timer_stop_dribbler > self.time_stop_dribbler:
                pos = robot.get_pos() + (robot.get_pos() - self.ball).unity() * (const.ROBOT_R * 1.5)
                Action_robot = Actions.GoToPoint(pos, robot.get_angle())
                if aux.dist(robot.get_pos(), self.ball) > self.dist_after_catch:
                    self.passes_status = FlagToPasses.FALSE
            
        return Action_robot


        
    def check_cath_ball(self, field: fld.Field, robot: rbt.Robot) -> bool:
        """
        Проверяем летит ли мяч в сторону робота
        """
        pos_cath = aux.closest_point_on_line(field.ball_start_point, self.ball, robot.get_pos(), "R")
        field.strategy_image.draw_circle(field.ball_start_point, (255, 0, 255), 30)

        if(
            (pos_cath is None
            or aux.dist(pos_cath, robot.get_pos()) > self.dist_cath_ball
            or aux.dist(self.ball, robot.get_pos()) > self.dist_to_pas
            or field.ball.get_vel().mag() < 200)
            and self.passes_status != FlagToPasses.RELEASE
        ):
            self.passes_status = FlagToPasses.FALSE
            print(False)
            return False
        print(True)
        return True

    #### Вспомогательные функции ####
    def check_goal_point(
        self,
        field: fld.Field,
        ball: aux.Point,
        list_enemy: list[aux.Point],
    ) -> tuple[aux.Point | None, float]:
        """
        Строим косательные к вражеским роботам от ball
        строим отрезки в воротах которые защищены вражескими роботами
        Нахордим максимально большой по длинне, не защищенный отрезок в вражеских воротах
        и бъем в его центр
        """


        # Расчёт касательных к вражеским роботам
        result_cords = []
        for enemy in list_enemy:
            tangent_points = aux.get_tangent_points(enemy, ball, const.ROBOT_R)
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
    
    def _optimal_point(
        self,
        robot: aux.Point,
        ball: aux.Point,
        enemy_list: list[aux.Point],
        mid: aux.Point,
        field: fld.Field,
    ) -> aux.Point:
        """
        Находит оптимальную точку для паса, сравнивая расстояния.
        """
        maxim = 0
        minim_goal_dist = 10000.0
        res = aux.Point(0, 0)
        for x in range(int(ball.x) - 1100, int(ball.x) + 1100, 100):
            for y in range(int(ball.y) - 1100, int(ball.y) + 1100, 100):
                if abs(x) > 2250:
                    continue
                if abs(y) > 1500:
                    continue 
                cand = aux.Point(x, y)
                if ((const.GOAL_DX - const.GOAL_PEN_DX) - abs(cand.x) < 100) and (abs(cand.y) - abs(const.GOAL_PEN_DY / 2) < 100):
                    cond = 0
                    continue
                #field.strategy_image.draw_circle(cand, (255, 0, 255), 30)
                minim: float = 10000
                flag_to_point = True
                if aux.dist(cand, ball) < 500 or aux.dist(cand, ball) > 1200:
                    flag_to_point = False
                    continue
                for enemy in enemy_list:
                    if aux.dist(enemy, cand) < 500:
                        flag_to_point = False
                        break
                    minim = min((enemy - aux.closest_point_on_line(ball, cand, enemy, "S")).mag(), minim)
                if (
                    #self.check_point(field, cand, enemy_list)[0] > maxim
                    aux.dist(cand, field.enemy_goal.center) < minim_goal_dist
                    and aux.closest_point_on_line(robot, cand, ball).mag() > 200
                    and minim > const.ROBOT_R + 150 
                    and flag_to_point
                    and (mid is None or aux.dist(cand, aux.closest_point_on_line(ball, mid, cand)) > const.ROBOT_R + 100)
                ):
                    res = cand
                    minim_goal_dist = aux.dist(cand, field.enemy_goal.center)
        field.strategy_image.draw_circle(res, (255, 0, 0), 30)
        return res