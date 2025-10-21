from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt
from bridge.auxiliary.entity import Entity
from bridge.router.action import ActionDomain


def calc_passthrough_point(
    domain: ActionDomain,
    target: aux.Point,
    *,
    avoid_ball: bool = False,
    ignore_ball: bool = False,
    ignore_robots: dict[const.Color, list[int]] = {}
) -> Optional[aux.Point]:
    """
    Рассчитать ближайшую промежуточную путевую точку
    """
    robot = domain.robot
    field = domain.field

    obstacles_dist: list[tuple[Entity, float]] = []

    if avoid_ball:
        if aux.is_point_inside_circle(robot.get_pos(), field.ball.get_pos(), const.KEEP_BALL_DIST - 50):
            return aux.nearest_point_on_circle(robot.get_pos(), field.ball.get_pos(), const.KEEP_BALL_DIST + 50)
        ball = Entity(
            field.ball.get_pos(),
            field.ball.get_angle(),
            const.KEEP_BALL_DIST - const.ROBOT_R - 50,
        )
        obstacles_dist.append((ball, aux.dist(ball.get_pos(), robot.get_pos())))
    elif not ignore_ball:
        ball = field.ball

        if (
            len(aux.line_circle_intersect(robot.get_pos(), target, ball.get_pos(), const.ROBOT_R + ball.get_radius(), "S"))
            > 0
        ):
            obstacles_dist.append((ball, aux.dist(ball.get_pos(), robot.get_pos())))

    for obstacle in field.allies:
        if field.ally_color in ignore_robots and obstacle.r_id in ignore_robots[field.ally_color]:
            continue
        dist = (obstacle.get_pos() - robot.get_pos()).mag()
        if obstacle.is_used() and obstacle.get_radius() + robot.get_radius() < dist < const.VIEW_DIST:
            obstacles_dist.append((obstacle.to_entity(), dist))

    for obstacle in field.enemies:
        if field.ally_color.reverse() in ignore_robots and obstacle.r_id in ignore_robots[field.ally_color.reverse()]:
            continue
        dist = (obstacle.get_pos() - robot.get_pos()).mag()
        if obstacle.is_used() and obstacle.get_radius() + robot.get_radius() < dist < const.VIEW_DIST:
            obstacles_dist.append((obstacle.to_entity(), dist))

    sorted_obstacles = sorted(obstacles_dist, key=lambda x: x[1])

    obstacles = []
    for obst in sorted_obstacles:
        obstacles.append(obst[0])

    pth_point = calc_next_point(field, robot.get_pos(), target, domain.robot, obstacles)
    if pth_point is None:
        return None
    field.path_image.draw_line(robot.get_pos(), pth_point[0], color=(0, 0, 0))
    if pth_point[0] == target:
        return None

    return pth_point[0]


def calc_next_point(
    field: fld.Field,
    position: aux.Point,
    target: aux.Point,
    robot: rbt.Robot,
    obstacles: list[Entity],
) -> Optional[tuple[aux.Point, float]]:
    """Calculate next point for robot"""
    remaining_obstacles: list[Entity] = obstacles.copy()
    skipped_obstacles: list[Entity] = []
    while len(remaining_obstacles) > 0:
        obstacle = remaining_obstacles.pop(0)
        skipped_obstacles.append(obstacle)

        time_to_reach = aux.dist(obstacle.get_pos(), position) / const.MAX_SPEED
        center = obstacle.get_pos() + obstacle.get_vel() * time_to_reach
        radius = (
            obstacle.get_radius()
            + const.ROBOT_R
            + const.ROBOT_R * (robot.get_vel().mag() / const.MAX_SPEED) * 1  # <-- coefficient of fear [0; 1] for fast speed
            + time_to_reach * obstacle.get_vel().mag() * 0.5  # <-- coefficient of fear [0; 1], for moving obst
        )
        field.path_image.draw_circle(
            center,
            (127, 127, 127),
            radius,
        )
        if len(aux.line_circle_intersect(position, target, center, radius, "S")) > 0:
            tangents = aux.get_tangent_points(center, position, radius)
            if tangents is None or len(tangents) < 2:
                return None

            tangents[0] = aux.point_on_line(
                center,
                tangents[0],
                radius + const.ROBOT_R * 0.5,
            )
            tangents[1] = aux.point_on_line(
                center,
                tangents[1],
                radius + const.ROBOT_R * 0.5,
            )

            path_before0 = calc_next_point(field, position, tangents[0], robot, skipped_obstacles[:-1])
            path_before1 = calc_next_point(field, position, tangents[1], robot, skipped_obstacles[:-1])
            path_after0 = calc_next_point(field, tangents[0], target, robot, remaining_obstacles)
            path_after1 = calc_next_point(field, tangents[1], target, robot, remaining_obstacles)

            if (path_before0 is None or path_after0 is None) and (path_before1 is not None and path_after1 is not None):
                pth_point = path_before1[0]
                length = path_before1[1] + path_after1[1]
                return pth_point, length
            if (path_before0 is not None and path_after0 is not None) and (path_before1 is None or path_after1 is None):
                if path_after0 is None:
                    return None
                pth_point = path_before0[0]
                length = path_before0[1] + path_after0[1]
                return pth_point, length
            if (path_before0 is not None and path_after0 is not None) and (
                path_before1 is not None and path_after1 is not None
            ):

                length0 = path_before0[1] + path_after0[1]
                length1 = path_before1[1] + path_after1[1]
                in_zone0 = aux.is_point_inside_poly(path_before0[0], field.ally_goal.big_hull) or aux.is_point_inside_poly(
                    path_before0[0], field.enemy_goal.big_hull
                )
                in_zone1 = aux.is_point_inside_poly(path_before1[0], field.ally_goal.big_hull) or aux.is_point_inside_poly(
                    path_before1[0], field.enemy_goal.big_hull
                )

                if (length0 < length1 or in_zone1) and not in_zone0:
                    pth_point = path_before0[0]
                    length = path_before0[1] + path_after0[1]
                    field.path_image.draw_line(position, pth_point, color=(255, 0, 255))
                    return pth_point, length
                if (length1 < length0 or in_zone0) and not in_zone1:
                    pth_point = path_before1[0]
                    length = path_before1[1] + path_after1[1]
                    field.path_image.draw_line(position, pth_point, color=(255, 0, 255))
                    return pth_point, length

            return None

    field.path_image.draw_line(position, target, color=(255, 0, 127))
    return target, aux.dist(position, target)


def correct_target_pos(field: fld.Field, robot: rbt.Robot, target: aux.Point, avoid_ball: bool) -> aux.Point:
    """Correct target position"""
    if not aux.is_point_inside_poly(target, field.big_hull):
        target = aux.nearest_point_on_poly(target, field.big_hull)

    if avoid_ball:
        if aux.is_point_inside_circle(target, field.ball.get_pos(), const.KEEP_BALL_DIST):
            target = aux.nearest_point_on_circle(target, field.ball.get_pos(), const.KEEP_BALL_DIST)

    if robot.r_id != field.gk_id:
        if aux.is_point_inside_poly(target, field.ally_goal.big_hull):
            target = aux.nearest_point_on_poly(target, field.ally_goal.big_hull)
        elif aux.is_point_inside_poly(target, field.enemy_goal.big_hull):
            target = aux.nearest_point_on_poly(target, field.enemy_goal.big_hull)
    return target
