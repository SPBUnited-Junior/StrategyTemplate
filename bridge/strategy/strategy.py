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
        self.point_kick_goal = None # точки в воротах, в которую будет бить атакующий
        self.dist_line_goal = 0 #размер максимально длинного открытого отрезка в воротах

        self.old_ball = aux.Point(0, 0) #положение мяча когда он начал катиться
        self.ball = aux.Point(0, 0) # мяч

        # переменные для паса
        self.dist_to_pas = 2000 # расстояние до удара
        self.dist_cath_ball = 300 #расстояние на которое робот можнт отехать при ловле мяча
        self.passes_status = FlagToPasses.FALSE # флаг состояний
        self.time_stop_dribbler = 0.5 # время для остнаовки дриблера после паса
        self.timer_stop_dribbler = 0 #для остановки дриблера
        self.dist_after_catch = 140 # растояние на которое нужно отехать от мяча, после его поимки и остановки дриблера



    def process(self, field: fld.Field) -> list[Optional[Action]]:
        """
        Подсчет статических переменных (self)
        """

        ### расчет точки в воротах, в которую будет бить атакующий ###
        enemies = []
        for rbt in field.active_enemies(1):
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

        field.game_state = GameStates.RUN
        match field.game_state:
            case GameStates.RUN:
                self.run(field, actions)
            case GameStates.TIMEOUT:
                pass
            case GameStates.HALT:
                return [None] * const.TEAM_ROBOTS_MAX_COUNT
            case GameStates.PREPARE_PENALTY:
                pass
            case GameStates.PENALTY:
                pass
            case GameStates.PREPARE_KICKOFF:
                pass
            case GameStates.KICKOFF:
                pass
            case GameStates.FREE_KICK:
                pass
            case GameStates.STOP:
                # The router will automatically prevent robots from getting too close to the ball
                self.run(field, actions)

        
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
        print(self.passes_status, field.ball_start_point, field.ball.get_vel().mag())

    def process_goalkeeper():
        """
        The logic by which the goalkeeper acts

        includes (it is necessary to list the main points of the goalkeeper's strategy):
        """
        pass

    def process_attacker(self, field: fld.Field, attacker: rbt.Robot, robot_catch_ball: rbt.Robot):
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
    
    def kick_ball_to_goal(self, field: fld.Field):
        """
        Бъем в ворота
        Проверка на то чтобы point not is None 
        """

        point_kick_goal = self.point_kick_goal
        if self.point_kick_goal is None:
            point_kick_goal = field.enemy_goal.center

        return KickActions.Straight(point_kick_goal)
    
    def kick_ball_to_pas(self, field: fld.Field,  robot_catch_ball: rbt.Robot):
        """
        Даем пас роботу
        """
        voltage = get_pass_voltage(aux.dist(self.ball, robot_catch_ball.get_pos()))
        return KickActions.Straight(robot_catch_ball.get_pos(), voltage)
    
    def process_defender(self, field: fld.Field) -> list[Optional[Action]]:
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
        actions: list[Optional[Action]] = []

        #Вычисление точки для блокировки удара
        if aux.dist(dist_to_robot_with_ball, bottom_block) > aux.dist(dist_to_robot_with_ball, up_block):
            actions[self.idx1] = Actions.GoToPoint(up_block, 3.14)
        else:
            actions[self.idx1] = Actions.GoToPoint(bottom_block, 3.14)
            if  nearest_enemy_point == aux.Point(0, 0):
                actions[self.idx1] = Actions.GoToPoint(field.ally_goal.center + aux.Point(-1000, 0), 3.14)

        #Блокировка паса
        actions[self.idx2] = Actions.GoToPoint(aux.closest_point_on_line(robot_position2, robot_position1_enemy, robot_position2_enemy, "S"), (ball - robot_position2).arg())
        
        return actions


    def process_catch_ball(self, field: fld.Field, robot: rbt.Robot):
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
            Action = Actions.CatchBall(pos, (self.ball - robot.get_pos()).arg(), 15)
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
            Action = Actions.GoToPoint(robot.get_pos(), robot.get_angle())
            if time() - self.timer_stop_dribbler > self.time_stop_dribbler:
                pos = robot.get_pos() + (robot.get_pos() - self.ball).unity() * (const.ROBOT_R * 1.5)
                Action = Actions.GoToPoint(pos, robot.get_angle())
                if aux.dist(robot.get_pos(), self.ball) > self.dist_after_catch:
                    self.passes_status = FlagToPasses.FALSE
            
        return Action


        
    def check_cath_ball(self, field: fld.Field, robot: rbt.Robot) -> aux.Point:
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
    ) -> tuple[aux.Point, float]:
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
        left: float = min(result_cords[0][0], field_down.y) if result_cords else None
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
        minim_goal_dist = 10000
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
                    maxim = self.check_point(field, cand, enemy_list)[0]
                    minim_goal_dist = aux.dist(cand, field.enemy_goal.center)
        field.strategy_image.draw_circle(res, (255, 0, 0), 30)
        return res