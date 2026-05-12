"""
Processor that creates the field
"""

import typing
from time import time

import attr
import math

# from scipy.optimize import minimize
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.processors import BaseProcessor

from bridge import const, drawing
from bridge.auxiliary import aux, fld
from bridge.strategy.check_point import check_goal_point


@attr.s(auto_attribs=True)
class ExplorePasses(BaseProcessor):
    """class that creates the field"""

    processing_pause: typing.Optional[float] = 0.001
    reduce_pause_on_process_time: bool = False

    ally_color: const.Color = const.Color.BLUE  

    def initialize(self, data_bus: DataBus) -> None:
        """Инициализация"""
        super().initialize(data_bus)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.passes_writer = DataWriter(data_bus, const.PASS_TOPIC, 1)
        self.image_writer = DataWriter(data_bus, const.IMAGE_TOPIC, 10)

        self.field = fld.Field(self.ally_color)
        self.image = drawing.Image(drawing.ImageTopic.PASSES)
        self.image.timer = drawing.FeedbackTimer(time(), 200, 5)

    def process(self) -> None:
        """
        Метод обратного вызова процесса
        """

        new_field = self.field_reader.read_last()
        if new_field is not None:
            updated_field = new_field.content
            self.field.update_field(updated_field)
        else:
            return
        self.image.timer.start(time())

        list_points: list[aux.Point] = self.optimal_point(self.field, self.field.ball.get_pos())

        self.passes_writer.write(list_points)
        self.image.timer.end(time())
        self.image_writer.write(self.image)
        self.image.clear()

    def optimal_point(
        self,
        field: fld.Field,
        ball: aux.Point,
    ) -> list[aux.Point]:
        """
        Находит оптимальную точку для паса, сравнивая расстояния.
        """
        maxim = 0.0
        points: list[tuple[float, aux.Point]] = []
        for x in range(-const.FIELD_DX + 200, const.FIELD_DX - 200, 50):
            for y in range(-const.FIELD_DY + 200, const.FIELD_DY - 100, 50):
                if abs(x) > 2250:
                    continue
                if abs(y) > 1500:
                    continue 
                cand = aux.Point(x, y)
                # if ((const.GOAL_DX - const.GOAL_PEN_DX) - abs(cand.x) < 500) and (abs(cand.y) - abs(const.GOAL_PEN_DY / 2) < 500):
                #     cond = 0
                #     continue

                minim: float = 10000
                quality = self.quality_point(field, cand)
                red = int(255 * (1 - quality))
                green = int(255 * quality)
                self.image.draw_circle(cand, (red, green, 0))
                points.append((quality, cand))
        points.sort(key=lambda x: x[0])
        points.reverse()
        positions: list[aux.Point] = []
        for i in range(len(points)):
            is_correct_point: bool = True
            cord = points[i][1]
            for pnt in positions:
                if aux.dist(pnt, cord) < const.DIST_PASSES_POINT:
                    is_correct_point = False
            
            if is_correct_point:
                positions.append(cord)

            if len(positions) >= const.COUNT_PASSES_POINT:
                return positions

        return positions

    def quality_point(
        self,
        field: fld.Field,
        point: aux.Point,
    ) -> float:
        kick_goal_point = check_goal_point(field, field.ball.get_pos())[0]
        ball = field.ball.get_pos()

        if self.point_in_goal(field, point):
            return 0
        if self.block_kick_goal(field, point, kick_goal_point):
            return 0
        if self.nearest_to_ball(field, point):
            return 0
        
        nearest_robot = fld.find_nearest_robot(ball, field.active_allies(False))
        cath_dist = aux.dist(nearest_robot.get_pos(), ball) - const.BALL_R

        target_angle = (point - nearest_robot.get_pos()).arg()
        diff_angle = abs(aux.wind_down_angle(target_angle - nearest_robot.get_angle()))

        catch_time = cath_dist / const.MAX_SPEED + 0.2 * diff_angle / const.ANGLE_VEL_MAX


        """
        вероятность возможности блокировки пасса вражеским роботом 
        """
        risk_intercept: float = 1
        for rbt in field.active_enemies(False):
            rbt_catch_point = aux.closest_point_on_line(ball, point, rbt.get_pos(), "S")
            rbt_dist_to_point = (rbt.get_pos() - rbt_catch_point).mag()

            dist_flight_ball = aux.dist(ball, rbt_catch_point)
            enemy_t = rbt_dist_to_point / 1000
            ball_t = dist_flight_ball / 2800

            delta_t = ball_t - enemy_t
            x = min(max(-20, (12 * delta_t)), 20)
            risk_intercept = min(risk_intercept, 1 / (1 + math.e ** x))

        """
        коэффицент чем ближе наш робот тем больше коэффицент 
        """
        reception: float = 0
        dist_flight_ball = aux.dist(ball, point)
        ball_t = dist_flight_ball / 2000
        for rbt in field.active_allies(False):
            if (rbt.r_id == nearest_robot.r_id): continue
            dist_to_point = aux.dist(rbt.get_pos(), point)
            rbt_t = dist_to_point / const.MAX_SPEED
 
            delta_t = rbt_t - ball_t - catch_time
            x = min(max(-20, (12 * delta_t)), 20)
            reception = max(reception, 1 / (1 + math.e ** x))
        """
        коэффициент возможностьи ударить в ворота
        """
        P_goal: float = 0
        list_goal_point = check_goal_point(field, point, False)
        size = list_goal_point[1]
        goal_point = list_goal_point[0]
        if goal_point is not None:
            down: aux.Point = aux.Point(goal_point.x, goal_point.y - size / 2)
            up: aux.Point = aux.Point(goal_point.x, goal_point.y + size / 2)

            open_angle = abs(aux.wind_down_angle(aux.get_angle_between_points(down, point, up)))
            max_angle = 0.9 #abs(aux.wind_down_angle(aux.get_angle_between_points(field.enemy_goal.center_down, ball, field.enemy_goal.center_up)))
            goal_thread_raw = open_angle / max_angle
            x = min(max(-50, (-8 * (goal_thread_raw - 0.2))), 50)
            P_goal = 1 / (1 + math.e ** x)


        forward_bonus = 1 # - (abs(point.x - field.enemy_goal.center.x)) / 4500

        """
        сумма весов
        """
        return reception * risk_intercept * P_goal * forward_bonus
    
    def point_in_goal(
        self,
        field: fld.Field,
        point: aux.Point,
        ) -> bool:
        """
        Проверка на то что точка находиться не в воротах
        """
        ball = field.ball.get_pos()

        if aux.is_point_inside_poly(point, field.ally_goal.hull): return True
        if aux.is_point_inside_poly(point, field.enemy_goal.hull): return True
        return False

    def block_kick_goal(
        self,
        field: fld.Field,
        point: aux.Point,
        kick_goal_point: None | aux.Point
    ) -> bool:
        """
        Проверка на то что точка не стоит на траектории удара в ворота
        """
        ball = field.ball.get_pos()
        if kick_goal_point is None: return False
        return aux.dist(point, aux.closest_point_on_line(ball, kick_goal_point, point)) < const.MIN_DIST_FROM_KICK

    def nearest_to_ball(
        self,
        field: fld.Field,
        point: aux.Point,
    ) -> bool:
        """
        Проверка на тол что точка не слишком близко к мячу
        """
        ball = field.ball.get_pos()
        return aux.dist(ball, point) < const.MIN_PASS_DIST