"""
Processor that creates the field
"""

import typing
from time import time

import attr

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
        for x in range(-const.FIELD_DX + 200, const.FIELD_DX - 200, 200):
            for y in range(-const.FIELD_DY + 200, const.FIELD_DY - 100, 200):
                if abs(x) > 2250:
                    continue
                if abs(y) > 1500:
                    continue 
                cand = aux.Point(x, y)
                if ((const.GOAL_DX - const.GOAL_PEN_DX) - abs(cand.x) < 500) and (abs(cand.y) - abs(const.GOAL_PEN_DY / 2) < 500):
                    cond = 0
                    continue

                minim: float = 10000
                quality = self.quality_point(field, cand)
                red = int(max(0, 255 / 5000 * (5000 - quality)))
                green = int(min(255, 255 / 5000 * quality))
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
            #print(point)
            return 0
        if self.block_kick_goal(field, point, kick_goal_point):
            #print(point)
            return 0
        if self.nearest_to_ball(field, point):
            #print(point)
            return 0
        
        """
        коэффицент возможности блокировки пасса вражеским роботом 
        """
        for rbt in field.active_enemies(False):
            rbt_catch_point = aux.closest_point_on_line(ball, point, rbt.get_pos(), "S")
            rbt_dist_to_point = (rbt.get_pos() - rbt_catch_point).mag()
            if (rbt_dist_to_point * 1.5 < aux.dist(ball, rbt_catch_point)): return 0 

        """
        коэффицент чем ближе наш робот тем больше коэффицент 
        """
        nearest_robot = fld.find_nearest_robot(ball, field.active_allies(False))
        pass_weight: float = 0
        for rbt in field.active_allies(False):
            if (rbt == nearest_robot): continue
            dist_to_point = aux.dist(rbt.get_pos(), point)
            pass_weight = max(pass_weight, abs(3000 - dist_to_point))
        """
        коэффициент возможностьи ударить в ворота
        """
        kick_to_goal_weight: float = check_goal_point(field, point, False)[1] * 5

        """
        сумма весов
        """
        weight: float = kick_to_goal_weight + pass_weight
        return weights
    
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