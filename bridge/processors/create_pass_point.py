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
        for x in range(int(ball.x) - 1400, int(ball.x) + 1400, 200):
            for y in range(int(ball.y) - 1400, int(ball.y) + 1400, 200):
                if abs(x) > 2250:
                    continue
                if abs(y) > 1500:
                    continue 
                cand = aux.Point(x, y)
                if ((const.GOAL_DX - const.GOAL_PEN_DX) - abs(cand.x) < 100) and (abs(cand.y) - abs(const.GOAL_PEN_DY / 2) < 100):
                    cond = 0
                    continue

                minim: float = 10000
                red = int(max(0, 255 / 22500000 * (22500000 - self.quality_point(field, cand))))
                green = int(min(255, 255 / 22500000 * self.quality_point(field, cand)))
               # print(self.quality_point(field, cand))
                #print(quality_point(field, cand, mid))
                self.image.draw_circle(cand, (red, green, 0))
                points.append((self.quality_point(field, cand), cand))
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
        block_weight: float = 3000
        for rbt in field.active_enemies(False):
            dist_to_point = (rbt.get_pos() - aux.closest_point_on_line(ball, point, rbt.get_pos(), "S")).mag()
            block_weight = min(block_weight, dist_to_point * 3)
        """
        коэффициент возможностьи ударить в ворота
        """
        kick_to_goal_weight: float = check_goal_point(field, point)[1] * 11

        """
        сумма весов
        """
        weight: float = block_weight * kick_to_goal_weight
        return weight
    
    def point_in_goal(
        self,
        field: fld.Field,
        point: aux.Point,
        ) -> bool:
        """
        Проверка на то что точка находиться не в воротах
        """
        ball = field.ball.get_pos()

        if aux.is_point_inside_poly(ball, field.ally_goal.hull): return True
        if aux.is_point_inside_poly(ball, field.enemy_goal.hull): return True
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