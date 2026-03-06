"""
Модуль для расчёта рикошета мяча от робота-партнера.
Использует закон отражения: угол падения = угол отражения.
"""

import math
from typing import Optional

from bridge.auxiliary import aux
from bridge.router.base_actions import Action, Actions, KickActions
import math  # type: ignore
from time import time  # type: ignore
from typing import Optional
from enum import Enum

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions, get_pass_voltage  # type: ignore
from bridge.strategy.check_point import check_goal_point

def calculate_ricochet_shot(
    ball_pos: aux.Point,
    teammate_pos: aux.Point,
    goal_point: aux.Point,
    teammate_radius: float = 180,
    ball_radius: float = 45
) -> Optional[aux.Point]:
    """
    Рассчитывает точку на роботе-партнере для рикошета в ворота.
    
    Args:
        ball_pos: позиция мяча
        teammate_pos: позиция центра робота-партнера
        goal_point: точка цели (центр ворот или рассчитанная точка)
        teammate_radius: радиус робота (с запасом для коллизий)
        ball_radius: радиус мяча
    
    Returns:
        Optional[aux.Point]: точка на роботе для удара или None
    """
    
    effective_radius = teammate_radius + ball_radius
    
    # Векторы
    to_teammate = teammate_pos - ball_pos
    to_goal = goal_point - teammate_pos
    
    # Проверка: робот должен быть между мячом и воротами
    if aux.scal_mult(to_teammate, to_goal) > 0:
        return None
    
    # Расстояние до робота должно быть достаточным
    if to_teammate.mag() < effective_radius:
        return None
    
    # Находим точку на линии "мяч-ворота", ближайшую к роботу
    point_on_goal_line = aux.closest_point_on_line(
        ball_pos, 
        goal_point, 
        teammate_pos, 
        "L"  # бесконечная линия
    )
    
    # Находим точку касания на роботе
    dir_to_tangent = (point_on_goal_line - teammate_pos).unity()
    hit_point = teammate_pos + dir_to_tangent * effective_radius
    
    # Проверяем, что точка в пределах поля
    if abs(hit_point.x) > 2500 or abs(hit_point.y) > 1800:
        return None
    
    # Проверяем условие отражения
    if not _check_reflection_condition(ball_pos, teammate_pos, goal_point, hit_point):
        # Если условие не выполняется, пробуем уточнить
        hit_point = _refine_ricochet_point(
            ball_pos, teammate_pos, goal_point, effective_radius, hit_point
        )
    
    return hit_point


def _check_reflection_condition(
    ball_pos: aux.Point,
    teammate_pos: aux.Point,
    goal_point: aux.Point,
    hit_point: aux.Point,
    tolerance: float = 0.1
) -> bool:
    """
    Проверяет условие отражения в точке удара.
    """
    normal = (hit_point - teammate_pos).unity()
    shot_dir = (hit_point - ball_pos).unity()
    goal_dir = (goal_point - hit_point).unity()
    
    cos_in = abs(aux.scal_mult(shot_dir, normal))
    cos_out = abs(aux.scal_mult(goal_dir, normal))
    
    return abs(cos_in - cos_out) < tolerance


def _refine_ricochet_point(
    ball_pos: aux.Point,
    teammate_pos: aux.Point,
    goal_point: aux.Point,
    radius: float,
    initial_point: aux.Point,
    max_iterations: int = 10
) -> Optional[aux.Point]:
    """
    Уточняет точку рикошета итеративно.
    """
    hit_point = initial_point
    
    for i in range(max_iterations):
        normal = (hit_point - teammate_pos).unity()
        shot_dir = (hit_point - ball_pos).unity()
        goal_dir = (goal_point - hit_point).unity()
        
        cos_in = aux.scal_mult(shot_dir, normal)
        cos_out = aux.scal_mult(goal_dir, normal)
        
        if abs(cos_in - cos_out) < 0.05:
            return hit_point
        
        # Корректируем направление
        if cos_in > cos_out:
            # Нужно уменьшить угол падения
            angle_correction = -0.05
        else:
            # Нужно увеличить угол падения
            angle_correction = 0.05
        
        # Поворачиваем точку вдоль окружности
        new_dir = aux.rotate(hit_point - teammate_pos, angle_correction).unity()
        hit_point = teammate_pos + new_dir * radius
    
    # Проверяем последнюю итерацию
    if _check_reflection_condition(ball_pos, teammate_pos, goal_point, hit_point, 0.15):
        return hit_point
    
    return None


def is_ricochet_possible(
    ball_pos: aux.Point,
    teammate_pos: aux.Point,
    goal_point: aux.Point,
    teammate_radius: float = 180
) -> bool:
    """
    Быстрая проверка возможности рикошета.
    """
    to_teammate = teammate_pos - ball_pos
    to_goal = goal_point - teammate_pos
    
    # Робот должен быть между мячом и воротами
    if aux.scal_mult(to_teammate, to_goal) > 0:
        return False
    
    # Расстояние до робота должно быть достаточным
    if to_teammate.mag() < teammate_radius + 45:
        return False
    
    return True


def get_ricochet_action(
    robot_kicker_id: int,
    robot_target_id: int,
    ball_pos: aux.Point,
    teammate_pos: aux.Point,
    goal_point: aux.Point,
    voltage: float = 6.0
) -> tuple[Optional[Action], Optional[Action]]:
    """
    Создаёт действия для рикошета.
    
    Returns:
        tuple[Optional[Action], Optional[Action]]: (действие для бьющего, действие для принимающего)
    """
    hit_point = calculate_ricochet_shot(ball_pos, teammate_pos, goal_point)
    
    if hit_point is None:
        return None, None
    
    # Действие для бьющего робота
    kicker_action = KickActions.Turn_Kick(
        hit_point,
        (hit_point - ball_pos).arg(),
        voltage
    )
    
    # Действие для принимающего робота (подстраивается под удар)
    target_action = Actions.GoToPoint(
        hit_point,
        (ball_pos - teammate_pos).arg()
    )
    
    return kicker_action, target_action


def visualize_ricochet(
    field,
    ball_pos: aux.Point,
    teammate_pos: aux.Point,
    goal_point: aux.Point,
    hit_point: Optional[aux.Point] = None
) -> None:
    """
    Визуализирует траекторию рикошета для отладки.
    """
    if hit_point is None:
        hit_point = calculate_ricochet_shot(ball_pos, teammate_pos, goal_point)
        
    if hit_point is None:
        return
    
    # Рисуем точку удара
    field.strategy_image.draw_circle(hit_point, (0, 255, 0), 40)
    
    # Рисуем траекторию мяча
    field.strategy_image.draw_line(ball_pos, hit_point, (0, 255, 0), 3)
    field.strategy_image.draw_line(hit_point, goal_point, (0, 255, 0), 3)
    
    # Рисуем нормаль в точке удара
    normal = (hit_point - teammate_pos).unity() * 100
    field.strategy_image.draw_line(
        hit_point - normal,
        hit_point + normal,
        (255, 255, 0),
        2
    )
    
    # Рисуем окружность робота
    for angle in range(0, 360, 30):
        rad = math.radians(angle)
        p1 = teammate_pos + aux.Point(math.cos(rad), math.sin(rad)) * 180
        p2 = teammate_pos + aux.Point(math.cos(rad + 0.2), math.sin(rad + 0.2)) * 180
        field.strategy_image.draw_line(p1, p2, (100, 100, 255), 1)