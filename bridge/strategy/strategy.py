"""High-level strategy code"""

# !v DEBUG ONLY
import math  # type: ignore
from time import time  # type: ignore
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions  # type: ignore


class Strategy:
    """Main class of strategy"""

    def __init__(
        self,
    ) -> None:
        self.we_active = False
        #Индексы союзных роботов
        self.goalkeeper_idx = 0
        self.idx1 = 1
        self.idx2 = 2

        #Индексы вражеских роботов
        self.goalkeeper_idx_enemy = 0
        self.idx_enemy1 = 1
        self.idx_enemy2 = 2


    def process(self, field: fld.Field) -> list[Optional[Action]]:
        """Game State Management"""
        if field.game_state not in [GameStates.KICKOFF, GameStates.PENALTY]:
            if field.active_team in [const.Color.ALL, field.ally_color]:
                self.we_active = True
            else:
                self.we_active = False

        actions: list[Optional[Action]] = []
        for _ in range(const.TEAM_ROBOTS_MAX_COUNT):
            actions.append(None)

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

    def process_goalkeeper():
        """
        The logic by which the goalkeeper acts

        includes (it is necessary to list the main points of the goalkeeper's strategy):
        """
        pass

    def process_attacker():
        """
        The logic by which the attacker acts

        includes (it is necessary to list the main points of the attacker's strategy):
        """
        pass

    def process_defender(self, field: fld.Field) -> list[Optional[Action]]:
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


        """
        The logic by which the attacker acts

        includes (it is necessary to list the main points of the defender's strategy):
        """

    def precess_catch_a_pas():
        pass