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
from bridge.strategy.check_point import check_goal_point
from bridge.strategy.Role import Role

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

class BallStatus(Enum):
    Active = 0
    Passive = 1
    Ready = 2


class BallStatusInsidePoly(Enum):
    NotInsidePoly = 0
    InsidePoly = 1

class RicochetState:
    """Класс для хранения состояния рикошета"""
    def __init__(self):
        self.active = False
        self.shot_point: Optional[aux.Point] = None
        self.target_point: Optional[aux.Point] = None
        self.ball_pos_at_start: Optional[aux.Point] = None
        self.teammate_pos_at_start: Optional[aux.Point] = None
        self.goal_point_at_start: Optional[aux.Point] = None
        self.start_time = 0
        
    def is_valid(self, current_ball: aux.Point, current_teammate: aux.Point, current_goal: aux.Point) -> bool:
        if not self.active:
            return False
            
        if aux.dist(current_ball, self.ball_pos_at_start) > 200:
            return False
            
        if aux.dist(current_teammate, self.teammate_pos_at_start) > 100:
            return False
            
        if time() - self.start_time > 3.0:
            return False
            
        return True

class Strategy:
    """Main class of strategy"""

    def __init__(
        self,
    ) -> None:
        self.we_active = False
        self.ricochet = RicochetState()

        # Индексы роботов

        self.goalkeeper_idx = 3
        self.idx1 = 0
        self.idx2 = 2
        
        # Индексы роботов соперника

        self.goalkeeper_idx_enemy = 0
        self.idx_enemy1 = 1
        self.idx_enemy2 = 2

        self.enemies : list[aux.Point] = [] # массив позиций вражеских роботов

        # статические переменные
        self.point_kick_goal : aux.Point | None = aux.Point(0, 0) # точки в воротах, в которую будет бить атакующий
        self.dist_line_goal = 0.0 #размер максимально длинного открытого отрезка в воротах

        self.old_ball : aux.Point = aux.Point(0, 0) #положение мяча когда он начал катиться
        self.ball = aux.Point(0, 0) # мяч

        # переменные для паса
        self.dist_to_pas = 2000 # расстояние до удара
        self.dist_cath_ball = 300 #расстояние на которое робот можнт отехать при ловле мяча
        self.passes_status = FlagToPasses.FALSE # флаг состояний
        self.time_stop_dribbler = 0.2 # время для остнаовки дриблера после паса
        self.timer_stop_dribbler = 0.0 #для остановки дриблера
        self.time_work_dribbler = 0.3  #время для остановки мяча в дриблере после паса
        self.timer_work_dribbler = 0.0 #для работы дриблера чтобы остановить мяч
        self.dist_after_catch = 140 # растояние на которое нужно отехать от мяча, после его поимки и остановки дриблера


        self.robot_catch_ball: rbt.Robot | None = None
        self.nearest_robot: rbt.Robot | None = None

        #для состояний 
        self.dist_to_ball = 450

        self.ball_status = BallStatus.Passive
        self.kick_status = Kick_Status.Not_Kick
        self.ball_status_poly = BallStatusInsidePoly.NotInsidePoly
        self.kick_up_is_used = 1

        #массив для функции go_to_position
        self.used = [False] * const.ROBOTS_MAX_COUNT


    def spin_test(self, field: fld.Field, idx: int = 0) -> list[Optional[Action]]:
        actions: list[Optional[Action]] = [None] * const.TEAM_ROBOTS_MAX_COUNT  
    
        current_time = time()
        if not hasattr(self, 'spin_start_time'):
            self.spin_start_time = current_time
            self.last_k = 0

        el = int(current_time - self.spin_start_time)
        k = el
        DRIBBLER_SPEED = 15
        print(k)

        ANGULAR_SPEED = k * math.pi

        actions[idx] = Actions.VelocityWithDribbler(
            velocity=aux.Point(0, 0),
            angle=ANGULAR_SPEED,
            dribbler_speed=DRIBBLER_SPEED,
            control_angle_by_speed=True
        )

        #actions[idx] = Actions.SimpleDribbler()
        return actions
#
    def process(self, field: fld.Field) -> list[Optional[Action]]:
        """
        Подсчет статических переменных (self)
        
        """
        print(GameStates)
        field.active_enemies # масив роботов на поле
        self.old_ball = field.ball_start_point or aux.Point(0, 0)

        #p+9obot_position1.is_used # true на поле

        """Game State Management"""
        voltage_kik = 6

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
        
        self.enemies = []
        for rbt in field.active_enemies(True):
            self.enemies.append(rbt.get_pos())

        kick_inf_list = check_goal_point(
            field,
            self.ball
        )

        self.point_kick_goal = kick_inf_list[0]
        self.dist_line_goal = kick_inf_list[1]

        ### полложение мяча ###
        #if self.ball == aux.Point(0, 0) or aux.dist(aux.Point(field.ball.get_pos().x,field.ball.get_pos().y), self.ball) < 1000.0:
        self.ball = aux.Point(field.ball.get_pos().x,field.ball.get_pos().y)
        field.strategy_image.draw_circle(self.ball, (255, 255, 255), 30)
        

        """Game State Management"""
        if field.game_state not in [GameStates.KICKOFF, GameStates.PENALTY]:
            if field.active_team in [const.Color.ALL, field.ally_color]:
                self.we_active = True
            else:
                self.we_active = False
                
        actions: list[Optional[Action]] = []
        for _ in range(const.TEAM_ROBOTS_MAX_COUNT):
            actions.append(None)

        print(field.game_state, self.we_active)

        if field.game_state == GameStates.RUN:
            self.run(field, actions)
            #self.process_ricochet(field,actions)
            
        elif field.game_state == GameStates.TIMEOUT:
            pass

        elif field.game_state == GameStates.HALT:
            actions[self.idx1] = Actions.Stop()
            actions[self.idx2] = Actions.Stop()
            actions[self.goalkeeper_idx] = Actions.Stop()
           
        elif field.game_state == GameStates.PREPARE_PENALTY and  not self.we_active:
            kik_angle1 = robot_position1_enemy - robot_position1
            kik_angle2 = robot_position2_enemy - robot_position2
            pos_kikoff1 = aux.Point(800  * field.polarity, 0)
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

            actions[self.idx1] = Actions.GoToPoint(pos_kikoff1, kik_angle1.arg())
            actions[self.idx2] = Actions.GoToPoint(pos_kikoff2, kik_angle2.arg())       
        
        elif field.game_state == GameStates.PENALTY and not self.we_active:
            position_penalty1 = self.ball
            angle_penalty1 = self.ball - robot_position1
            actions[self.idx1] = Actions.GoToPoint(position_penalty1, angle_penalty1.arg())

        elif field.game_state == GameStates.PENALTY and self.we_active:
            g_up_xy_attacker = field.enemy_goal.up - field.enemy_goal.eye_up * 35   #определяется угол ворот противоположный от враторя
            g_down_xy_attacker = field.enemy_goal.down + field.enemy_goal.eye_up * 35

            up_attacker = (g_up_xy_attacker - robot_position_goalkeeper_enemy).mag()
            down_attacker = (robot_position_goalkeeper_enemy - g_down_xy_attacker).mag()

            if up_attacker > down_attacker:
                position_attacker_gate = g_up_xy_attacker
            else:
                position_attacker_gate = g_down_xy_attacker

            actions[self.idx1] = KickActions.Straight(position_attacker_gate, voltage_kik)

        elif field.game_state == GameStates.PREPARE_KICKOFF:
            kik_angle1 = self.ball - robot_position1
            kik_angle2 = self.ball- robot_position2

            pos_kikoff1 = aux.Point(500 * field.polarity, 0)   
            pos_kikoff2 = aux.Point(1000 * field.polarity, 0)

            actions[self.idx1] = Actions.GoToPoint(pos_kikoff1, kik_angle1.arg())
            actions[self.idx2] = Actions.GoToPoint(pos_kikoff2, kik_angle2.arg())
            
        elif field.game_state == GameStates.KICKOFF and  not self.we_active:
            kik_angle1 = self.ball - robot_position1
            kik_angle2 = self.ball - robot_position2
            pos_kikoff1 = aux.Point(500 * field.polarity, 0)
            pos_kikoff2 = aux.Point(1000 * field.polarity, 0)
            actions[self.idx1] = Actions.GoToPoint(pos_kikoff1, kik_angle1.arg())
            actions[self.idx2] = Actions.GoToPoint(pos_kikoff2, kik_angle2.arg())

        elif field.game_state == GameStates.KICKOFF and self.we_active:
            self.process_attacker(field, actions)

        elif field.game_state == GameStates.FREE_KICK and self.we_active:
            self.process_attacker(field, actions)

        elif field.game_state == GameStates.FREE_KICK and not self.we_active:
            self.process_defender(field, actions)

        elif field.game_state == GameStates.STOP:
            self.flag = False
            pos_attacker1 =  self.ball + (field.ally_goal.center - self.ball).unity() * self.dist_to_ball
            angle_attacker1 = (self.ball - robot_position1).arg()
            pos_attacker2 = field.ally_goal.center + field.ally_goal.eye_forw * 800
            angle_attacker2 = (self.ball - robot_position2).arg()

            if aux.dist(pos_attacker1, self.ball) < 500:
                pos_attacker1 = self.ball + (field.ally_goal.center - self.ball).unity() * self.dist_to_ball

            if aux.dist(pos_attacker2, self.ball) < 500:
                pos_attacker2 = (robot_position2 - self.ball).unity() * 500

            if abs(field.enemy_goal.center.x - pos_attacker1.x) < 800:
                pos_attacker1 = field.enemy_goal.up - field.ally_goal.eye_forw * 800

            actions[self.idx1] = Actions.GoToPoint(pos_attacker1, angle_attacker1)
            actions[self.idx2] = Actions.GoToPoint(pos_attacker2, angle_attacker2)
        
        for rbt in field.active_allies(True):
            if abs(rbt.get_pos().x) > 2250 or abs(rbt.get_pos().y) > 1500:
                actions[rbt.r_id] = Actions.GoToPoint(aux.Point(0, 0), 0)

        if abs(self.ball.x) > 2250 or abs(self.ball.y) > 1500:
            actions[self.idx1] = Actions.Stop()
            actions[self.idx2] = Actions.Stop()
            actions[self.goalkeeper_idx] = Actions.Stop()
            print("мяч за полем")

        
        return actions

    def run(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        
        # #actions = self.process_attacker(field, actions)
        # dist_ally = aux.dist(fld.find_nearest_robot(self.ball, field.active_allies(False)).get_pos(), self.ball)
        # dist_enemy = aux.dist(fld.find_nearest_robot(self.ball, field.active_enemies(False)).get_pos(), self.ball)

        '''
        if (dist_ally > dist_enemy and ((self.ball.x < 0 and field.ally_goal.center.x < 0) or (self.ball.x > 0 and field.ally_goal.center.x > 0))):
            self.process_defender(field, actions)
        else:
            self.process_attacker(field, actions)
            pass
        '''
        # dist_ enemy = aux.dist(fld.find_nearest_robot(self.ball, field.active_enemies(False)).get_pos(), self.ball)
        # dist_ally = aux.dist(fld.find_nearest_robot(self.ball, field.active_allies(False)).get_pos(), self.ball)

        # if(
        #     ((self.ball.x < 0 and field.ally_goal.center.x < 0) or (self.ball.x > 0 and field.ally_goal.center.x > 0))
        #     and dist_enemy < dist_ally  ### Определяем чей робот ближе, наш или вражеский
        # ):
        #     self.process_defender(field, actions)
        # else:
        #     self.process_attacker(field, actions)


        # if False and self.point_kick_goal is not None:
        #     actions[0] = KickActions.Turn_Kick(self.point_kick_goal, (self.ball - field.allies[0].get_pos()).arg())
        # else:
        #     actions[0] = KickActions.Turn_Kick(field.ally_goal.center, (self.ball - field.allies[0].get_pos()).arg())

        robot1 = field.allies[0]
        # robot2 = field.allies[3]
        # robot3 = field.allies[7]
        # self.construction_well(field, actions, [robot1, robot2, robot3], aux.Point(200, 300), aux.Point(-200, -300))
        # print(robot1.get_pos(), robot2.get_pos(), robot3.get_pos())

        #actions[0] = KickActions.Turn_Kick(field.ally_goal.center, (self.ball - field.allies[0].get_pos()).arg())
        block = Role.Block_Enemy_Pass(field, actions)
        block.push(field.allies[1])
        block.push(field.allies[2])
        block.process()

        

    def process_goalkeeper(self, field: fld.Field, actions: list[Optional[Action]]) ->  list[Optional[Action]]:
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

        ball = aux.Point(field.ball.get_pos().x,field.ball.get_pos().y)

        g_up_xy_goal = field.enemy_goal.up - field.enemy_goal.eye_up * 40    #определяется угол ворот противоположный от враторя
        g_down_xy_goal = field.enemy_goal.down + field.enemy_goal.eye_up * 40

        up_goal = (g_up_xy_goal - robot_position_goalkeeper_enemy).mag()
        down_goal = (robot_position_goalkeeper_enemy + g_down_xy_goal).mag()

        if up_goal > down_goal:
            goal_position_gates = g_up_xy_goal
        else:
            goal_position_gates = g_down_xy_goal     #закончилось NOTE очень понятно

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
                
        return actions

    def process_attacker(self, field: fld.Field, actions: list[Optional[Action]]) -> list[Optional[Action]]:
        """
        The logic by which the attacker acts

        includes (it is necessary to list the main points of the attacker's strategy):
        """

        if self.passes_status == FlagToPasses.FALSE or field.is_ball_in_ally_robot():
            """
            Распределяем какой робот что делает
            """
            self.nearest_robot =  fld.find_nearest_robot(self.ball, field.active_allies(False))

            for robot in field.active_allies(False):
                if robot != self.nearest_robot:
                    self.robot_catch_ball = robot

        if self.nearest_robot is None or self.robot_catch_ball is None:
            """
            защита против type None
            """
            print("Error", self.ball.mag())
            return actions
        

        voltage = get_pass_voltage(aux.dist(self.ball, self.robot_catch_ball.get_pos()))
        angle_nearest_robot = (self.ball - self.nearest_robot.get_pos()).arg()
        if self.kick_status == Kick_Status.Goal_Turn_Kick    and field.is_ball_in_ally_robot():
            """
            Если робот захватил мяч и бьет в ворота с Turn
            """
            actions[self.robot_catch_ball.r_id] = Actions.GoToPoint(optimal_point, (self.ball - self.robot_catch_ball.get_pos()).arg())
            if self.point_kick_goal is None:
                self.kick_status = Kick_Status.Pass_Turn_Kick
                actions[self.nearest_robot.r_id] = KickActions.Turn_Kick(field.enemy_goal.center, angle_nearest_robot, voltage)
            else:
                actions[self.nearest_robot.r_id] = KickActions.Turn_Kick(self.point_kick_goal, angle_nearest_robot)

        elif self.kick_status == Kick_Status.Goal_Straight and field.is_ball_in_ally_robot():
            """
            Если робот захватил мяч и бьет в ворота с Straight
            """
            actions[self.robot_catch_ball.r_id] = Actions.GoToPoint(optimal_point, (self.ball - self.robot_catch_ball.get_pos()).arg())
            if self.point_kick_goal is None:
                actions[self.nearest_robot.r_id] = KickActions.Straight(field.enemy_goal.center)
            else:
                actions[self.nearest_robot.r_id] = KickActions.Straight(self.point_kick_goal)

        elif self.kick_status == Kick_Status.Pass_Straight and field.is_ball_in_ally_robot():
            """
            Если робот захватил мяч и бьет пасс с Straight
            """
            actions[self.robot_catch_ball.r_id] = Actions.GoToPoint(optimal_point_not_kick, (self.ball - self.robot_catch_ball.get_pos()).arg())
            actions[self.nearest_robot.r_id] = KickActions.Straight(self.robot_catch_ball.get_pos(), voltage)
        
        elif self.kick_status == Kick_Status.Pass_Turn_Kick and field.is_ball_in_ally_robot():
            """
            Если робот захватил мяч и бьет пасс с Turn
            """
            actions[self.robot_catch_ball.r_id] = Actions.GoToPoint(optimal_point_not_kick, (self.ball - self.robot_catch_ball.get_pos()).arg())
            actions[self.nearest_robot.r_id] = KickActions.Turn_Kick(self.robot_catch_ball.get_pos(), angle_nearest_robot, voltage)

        else:
            """
            Если робот не бьет  мяч
            """
            if ((self.kick_status == Kick_Status.Pass_Straight or self.kick_status == Kick_Status.Pass_Turn_Kick) 
                and self.check_cath_ball(field, self.robot_catch_ball) and not field.is_ball_in_ally_robot()):

                actions[self.robot_catch_ball.r_id] = self.process_catch_ball(field, self.robot_catch_ball)
                actions[self.nearest_robot.r_id] = Actions.GoToPoint(self.optimal_point(field, self.ball, None), (self.ball - self.nearest_robot.get_pos()).arg())

            elif self.point_kick_goal is not None and aux.dist(self.nearest_robot.get_pos(), field.enemy_goal.center) < 3500:
                self.kick_status = Kick_Status.Goal_Turn_Kick
                actions[self.nearest_robot.r_id] = KickActions.Turn_Kick(field.enemy_goal.center, angle_nearest_robot)
                actions[self.robot_catch_ball.r_id] = Actions.GoToPoint(optimal_point, (self.ball - self.robot_catch_ball.get_pos()).arg())
            else:
                actions[self.robot_catch_ball.r_id] = Actions.GoToPoint(optimal_point_not_kick, (self.ball - self.robot_catch_ball.get_pos()).arg())
                self.kick_status = Kick_Status.Pass_Turn_Kick
                actions[self.nearest_robot.r_id] = KickActions.Turn_Kick(self.robot_catch_ball.get_pos(), angle_nearest_robot, voltage)  


        return actions
    
    def kick_ball_to_goal(self, field: fld.Field) -> Action:
        """
        Бъем в ворота
        Проверка на то чтобы point not is None 
        """

        point_kick_goal = self.point_kick_goal
        if point_kick_goal is None:
            point_kick_goal = field.enemy_goal.center

        return KickActions.Turn_Kick(point_kick_goal, 0)
    
    def kick_ball_to_pas(self, field: fld.Field,  point_to_pas: aux.Point) -> Action:
        """
        Даем пас роботу
        """
        voltage = get_pass_voltage(aux.dist(self.ball, point_to_pas))
        return KickActions.Straight(point_to_pas, voltage)
    
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

        ball = aux.Point(field.ball.get_pos().x,field.ball.get_pos().y)
        nearest_enemy_dist = 5000.0
        nearest_enemy_point = aux.Point(0, 0)

        # определение ближайшего робота врага к мячу
        for i in field.active_enemies(False):
            if aux.dist(i.get_pos(), ball) < nearest_enemy_dist:
                nearest_enemy_dist = aux.dist(i.get_pos(), ball)
                nearest_enemy_point = i.get_pos()
        
        dist_to_robot_with_ball = (ball - nearest_enemy_point).unity() * 200 + ball

        bottom_crossbar = field.ally_goal.down - aux.Point(0, 200*field.polarity) # Небольшое расстояние от нижней штанги к углу
        up_crossbar = field.ally_goal.up + aux.Point(0, 200*field.polarity)  # Небольшое расстояние от верхней штанги к углу


        bottom_block = aux.closest_point_on_line(nearest_enemy_point, bottom_crossbar, dist_to_robot_with_ball, "R")
        up_block = aux.closest_point_on_line(nearest_enemy_point, up_crossbar, dist_to_robot_with_ball, "R")

        # Вычисление точки для блокировки удара
        
        if robot_position1.x - 50 >= nearest_enemy_point.x:
            if aux.dist(dist_to_robot_with_ball, bottom_block) > aux.dist(dist_to_robot_with_ball, up_block):
                actions[self.idx1] = Actions.GoToPoint(up_block, (ball - robot_position1).arg())
            else:
                actions[self.idx1] = Actions.GoToPoint(bottom_block, (ball - robot_position1).arg())

        else:
            actions[self.idx1] = Actions.GoToPoint(self._circle_to_two_tangents(80, ball, field.ally_goal.down, field.ally_goal.up, robot_position1), (ball - robot_position1).arg())

            
        #Блокировка паса
        actions[self.idx2] = Actions.GoToPoint(aux.point_on_line(robot_position1_enemy, robot_position2_enemy, aux.dist(robot_position1_enemy, robot_position2_enemy)/2), (ball - robot_position2).arg())


    def process_catch_ball(self, field: fld.Field, robot: rbt.Robot) -> Action:
        """
        Логика принятия всех летящих в робота мячей

        Проверка того что мяч летит в робота происходит
        вне функции, с помощью check_cath_a_ball()
        """

        """
        Если мяч ещё летит то
        подстравиваемься под него
        """
        self.passes_status = FlagToPasses.TRUE
        pos = aux.closest_point_on_line(field.ball_start_point, self.ball, robot.get_pos(), "R")
        Action_robot: Action = Actions.CatchBall(pos, (self.ball - robot.get_pos()).arg(), 8)
        if field.is_ball_in(robot):
            """
            Ждем когда мы поймаем мяч
            """
            self.passes_status = FlagToPasses.FALSE
            self.timer_work_dribbler = time()
            self.timer_stop_dribbler = time()
            for robot in field.active_allies(False):
                if robot != self.nearest_robot:
                    self.robot_catch_ball = robot
            

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
        ):
            self.passes_status = FlagToPasses.FALSE
            return False
        self.passes_status = FlagToPasses.TRUE
        return True

    #### Вспомогательные функции ####
    
    def _process_goalkeeper(
        self,
        field: fld.Field,
        ball: aux.Point,
        attacker: aux.Point,
        idx: int,
        goalkeeper: aux.Point,
    ) -> Action:
        """
        Обрабатывает логику вратаря:
          - Если противник близко к мячу, рассчитываем траекторию спасения;
          - Если противник далеко, корректируем позицию вратаря в зависимости от положения мяча.
        Также обрабатывается ситуация, когда мяч движется в зоне ворот.
        """
        Action_goalkeeper: Action = Actions.GoToPoint(goalkeeper, goalkeeper.arg())
        field.strategy_image.draw_circle(field.ball_start_point, (255, 0, 255), 50)
        # Если противник близко к мячу (готовится к удару)
        if aux.dist(attacker, ball) < 220:
            predict_pos = ball + aux.rotate(aux.Point(400, 0), field.enemies[idx].get_angle())
            field.strategy_image.draw_line(ball, predict_pos, (255, 255, 255), 5)
            pos = aux.closest_point_on_line(ball, predict_pos, goalkeeper, "R")
            cords1 = aux.get_line_intersection(
                ball,
                predict_pos,
                field.ally_goal.up + field.ally_goal.eye_forw * 120,
                field.ally_goal.down + field.ally_goal.eye_forw * 120,
                "RL",
            )
            result: Optional[aux.Point] = None
            if cords1 is not None:
                cords_sr = cords1
                if not aux.is_point_inside_poly(ball, field.ally_goal.hull):
                    result = aux.get_line_intersection(
                        ball,
                        cords1,
                        field.ally_goal.frw_up - field.ally_goal.eye_forw,
                        field.ally_goal.frw_down - field.ally_goal.eye_forw,
                        "RL",
                    )
                    if result is None:
                        result = aux.get_line_intersection(
                            ball,
                            cords1,
                            field.ally_goal.frw_down - field.ally_goal.eye_forw * 1,
                            field.ally_goal.center_down + field.ally_goal.eye_forw * 120,
                            "RS",
                        )
                    if result is None:
                        result = aux.get_line_intersection(
                            ball,
                            cords1,
                            field.ally_goal.frw_up - field.ally_goal.eye_forw * 1,
                            field.ally_goal.center_up + field.ally_goal.eye_forw * 120,
                            "RL",
                        )
                    if result is None:
                        result = aux.get_line_intersection(
                            ball,
                            cords1,
                            field.ally_goal.frw_up - field.ally_goal.eye_forw * 1,
                            field.ally_goal.frw_down - field.ally_goal.eye_forw * 1,
                            "RL",
                        )
                    if result is None:
                        pos = field.ally_goal.center + field.ally_goal.eye_forw * 300
                        angle = field.enemy_goal.center.arg()
                    else:
                        pos = aux.closest_point_on_line(result, cords1, goalkeeper, "S")
                    #     field.strategy_image.draw_line(result, cords1, (0, 0, 255), 5)
                    # field.strategy_image.draw_line(field.ally_goal.up + field.ally_goal.eye_forw * 120, field.ally_goal.down  + field.ally_goal.eye_forw * 120, (0, 0, 255), 5)
                    # field.strategy_image.draw_line(pos, self.old_ball, (0, 0, 255), 5)
                    # field.strategy_image.draw_circle(pos, (0, 0, 0), 40)
        else:
            # Если противник не у мяча
            pos = self._golakeeper_afk_point(field, ball)
            angle = field.enemy_goal.center.arg()

        # Если вратарь готов (мяч ранее отмечен как Ready) и противник далеко
        if field.ball.get_vel().mag() > 50:
            pos = aux.closest_point_on_line(self.old_ball, ball, goalkeeper, "R")
            cords1 = aux.get_line_intersection(
                self.old_ball,
                ball,
                field.ally_goal.up + field.ally_goal.eye_forw * 120,
                field.ally_goal.down + field.ally_goal.eye_forw * 120,
                "RL",
            )

            if cords1 is not None:
                cords_sr = cords1
                if not aux.is_point_inside_poly(ball, field.ally_goal.hull):
                    result = aux.get_line_intersection(
                        self.old_ball,
                        cords1,
                        field.ally_goal.frw_up - field.ally_goal.eye_forw,
                        field.ally_goal.frw_down - field.ally_goal.eye_forw,
                        "RL",
                    )
                    if result is None:
                        result = aux.get_line_intersection(
                            self.old_ball,
                            cords1,
                            field.ally_goal.frw_down - field.ally_goal.eye_forw * 1,
                            field.ally_goal.center_down + field.ally_goal.eye_forw * 120,
                            "RS",
                        )
                    if result is None:
                        result = aux.get_line_intersection(
                            self.old_ball,
                            cords1,
                            field.ally_goal.frw_up - field.ally_goal.eye_forw * 1,
                            field.ally_goal.center_up + field.ally_goal.eye_forw * 120,
                            "LL",
                        )
                    if result is None:
                        result = aux.get_line_intersection(
                            self.old_ball,
                            cords1,
                            field.ally_goal.frw_up - field.ally_goal.eye_forw * 1,
                            field.ally_goal.frw_down - field.ally_goal.eye_forw * 1,
                            "RL",
                        )

                    pos = aux.closest_point_on_line(result, cords1, goalkeeper, "S")
                    field.strategy_image.draw_line(result, cords1, (0, 0, 255), 5)
                    field.strategy_image.draw_line(
                        field.ally_goal.up + field.ally_goal.eye_forw * 120,
                        field.ally_goal.down + field.ally_goal.eye_forw * 120,
                        (0, 0, 255),
                        5,
                    )
                    field.strategy_image.draw_line(pos, self.old_ball, (0, 0, 255), 5)
                    field.strategy_image.draw_circle(pos, (0, 0, 0), 40)
            else:
                pos = self._golakeeper_afk_point(field, ball)
            if aux.is_point_inside_poly(ball, field.ally_goal.hull):
                self.ball_status_poly = BallStatusInsidePoly.InsidePoly

        # Если мяч вылетел за зону ворот после удара
        if self.ball_status_poly == BallStatusInsidePoly.InsidePoly and not aux.is_point_inside_poly(
            ball, field.ally_goal.hull
        ):
            self.ball_status_poly = BallStatusInsidePoly.NotInsidePoly
            self.ball_status = BallStatus.Passive
            pos = self._golakeeper_afk_point(field, ball)

        # Если позиция вне зоны ворот, корректируем позицию и направление
        if field.ally_goal.up.y > 0:
            lt = [ball, field.ally_goal.up + aux.Point(0, 100), field.ally_goal.down + aux.Point(0, -100)]
            field.strategy_image.draw_line(field.ally_goal.up + aux.Point(0, 100), field.ally_goal.down + aux.Point(0, -100), (255, 0, 0), 40)

        else:
            lt = [ball, field.ally_goal.up + aux.Point(0, -100), field.ally_goal.down + aux.Point(0, 100)]
            field.strategy_image.draw_line(field.ally_goal.up + aux.Point(0, -100), field.ally_goal.down + aux.Point(0, 100), (255, 0, 0), 40)
        if not aux.is_point_inside_poly(pos, field.ally_goal.hull) or not aux.is_point_inside_poly(pos, lt):
            pos = self._golakeeper_afk_point(field, ball)
            angle = field.enemy_goal.center.arg()
        else:
            angle = field.enemy_goal.center.arg()

        # Если мяч остановился после удара в зоне ворот

        if aux.is_point_inside_poly(ball, field.ally_goal.hull):
            # Используем _pas для расчёта данных паса (здесь лишь для установки флага удара)
            pos = ball
            if self.kick_up_is_used:
                Action_goalkeeper = KickActions.Straight(aux.Point(0, 0), 13, False, True)
            else:
                Action_goalkeeper = KickActions.Straight(goalkeeper + aux.Point(0, 1000), 10)
        else:
            Action_goalkeeper = Actions.GoToPoint(pos, angle)

        field.strategy_image.draw_circle(pos, (255, 0, 0), 50)
        self.old_pos = pos
        return Action_goalkeeper
    
    def _golakeeper_afk_point(self, field: fld.Field, ball: aux.Point) -> aux.Point:
        result_list = aux.line_circle_intersect(
            ball, field.ally_goal.center, field.ally_goal.center - field.ally_goal.eye_forw * 100, 450, "S"
        )
        result = field.ally_goal.center + field.ally_goal.eye_forw * 300
        if len(result_list) == 1:
            result = result_list[0]
        elif len(result_list) == 2:
            if abs(result_list[0].x) < abs(result_list[1].x):
                result = result_list[0]
            else:
                result = result_list[1]

        field.strategy_image.draw_circle(result, (255, 0, 0), 30)
        # field.strategy_image.draw_circle(field.ally_goal.center - field.ally_goal.eye_forw * 100, (255, 0, 0), 500)
        return result
    
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