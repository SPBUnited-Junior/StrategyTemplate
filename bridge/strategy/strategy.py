"""High-level strategy code"""

# !v DEBUG ONLY
import math  # type: ignore
from time import time  # type: ignore
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, get_pass_voltage  # type: ignore

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

        self.ball = aux.Point(0, 0) # мяч



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
            enemies#
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
        actions[1] = self.process_attacker(field, field.allies[1], field.allies[2])


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
    
    def process_defender():
        """
        The logic by which the attacker acts

        includes (it is necessary to list the main points of the defender's strategy):
        """
        pass

    def process_catch_a_pas():
        pass




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