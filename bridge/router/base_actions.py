"""
Class with robot actions
"""

import math
from time import time
from typing import Optional

import bridge.auxiliary.quickhull as qh
from bridge import const
from bridge.auxiliary import aux, fld, tau
from bridge.router.action import Action, ActionDomain, ActionValues, limit_action
from bridge.router.path_generation import calc_passthrough_point, correct_target_pos
from bridge.strategy.strategy import GameStates

# Actions: ActionDomain -> ActionValues


class Actions:
    """Class with all user-available actions (except kicks)"""

    class Stop(Action):
        """Stop the robot"""

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:
            """Behavior"""
            current_action.vel = aux.Point(0, 0)
            current_action.angle = domain.robot.get_angle()
            current_action.beep = 0

    class GoToPointIgnore(Action):
        """Go to point ignore obstacles"""

        def __init__(
            self,
            target_pos: aux.Point,
            target_angle: float,
            ball_interact: bool = False,
            target_vel: aux.Point = aux.Point(0, 0),
        ) -> None:
            self.target_pos = target_pos
            self.target_angle = target_angle
            self.ball_interact = ball_interact
            self.target_vel = target_vel

            self.use_dribbler = False

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:
            cur_robot = domain.robot
            vec_err = self.target_pos - cur_robot.get_pos()
            cur_vel = cur_robot.get_vel()

            ball_escorting = self.ball_interact and aux.dist(cur_robot.get_pos(), domain.field.ball.get_pos()) < 700
            if ball_escorting:
                cur_robot.pos_reg_x.select_mode(tau.Mode.SOFT)
                cur_robot.pos_reg_y.select_mode(tau.Mode.SOFT)
            else:
                cur_robot.pos_reg_x.select_mode(tau.Mode.NORMAL)
                cur_robot.pos_reg_y.select_mode(tau.Mode.NORMAL)

            u_x = cur_robot.pos_reg_x.process_(vec_err.x, -cur_vel.x, time() - cur_robot.prev_sended_time)
            u_y = cur_robot.pos_reg_y.process_(vec_err.y, -cur_vel.y, time() - cur_robot.prev_sended_time)
            current_action.vel = aux.Point(u_x, u_y)
            # return
            cur_vel_abs = aux.rotate(current_action.vel, -cur_robot.get_angle())
            prev_vel_abs = aux.rotate(cur_robot.prev_sended_vel, -cur_robot.prev_sended_angle)
            if (cur_vel_abs - prev_vel_abs).mag() / (
                time() - cur_robot.prev_sended_time
            ) > const.MAX_ACCELERATION and cur_vel_abs.mag() > prev_vel_abs.mag():
                # domain.field.router_image.draw_circle(aux.Point(0, 1000), size_in_mms=200)
                current_action.vel = aux.rotate(
                    prev_vel_abs
                    + (cur_vel_abs - prev_vel_abs).unity() * const.MAX_ACCELERATION * (time() - cur_robot.prev_sended_time),
                    cur_robot.get_angle(),
                )

            # current_action.vel = aux.Point(0,500)
            cur_robot.prev_sended_vel = current_action.vel
            cur_robot.prev_sended_angle = cur_robot.get_angle()
            cur_robot.prev_sended_time = time()
            current_action.angle = self.target_angle

            if self.use_dribbler:
                current_action.dribbler_speed = 15

            DumbActions.AddFinalVelocityAction(self.target_pos, self.target_vel).process(domain, current_action)

    class GoToPoint(Action):
        """Go to point and avoid obstacles"""

        def __init__(
            self,
            target_pos: aux.Point,
            target_angle: float,
            ball_interact: bool = False,
            ignore_ball: bool = False,
            target_vel: aux.Point = aux.Point(0, 0),
            ignore_robots: dict[const.Color, list[int]] = {},
        ) -> None:
            self.target_pos = target_pos
            self.target_angle = target_angle
            self.ball_interact = ball_interact
            self.ignore_ball = ignore_ball
            self.target_vel = target_vel
            self.ignore_robots = ignore_robots

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list["Action"]:
            avoid_ball = domain.game_state in [GameStates.STOP, GameStates.PREPARE_KICKOFF] or (
                domain.game_state in [GameStates.FREE_KICK, GameStates.KICKOFF] and not domain.we_active
            )
            self.target_pos = correct_target_pos(domain.field, domain.robot, self.target_pos, avoid_ball)

            angle0 = self.target_angle
            next_point = self.target_pos

            if domain.robot.r_id != domain.field.gk_id:
                if aux.is_point_inside_poly(domain.robot.get_pos(), domain.field.ally_goal.hull):
                    next_point = aux.nearest_point_on_poly(domain.robot.get_pos(), domain.field.ally_goal.big_hull)
                    return [Actions.GoToPointIgnore(next_point, angle0)]
                elif aux.is_point_inside_poly(domain.robot.get_pos(), domain.field.enemy_goal.hull):
                    next_point = aux.nearest_point_on_poly(domain.robot.get_pos(), domain.field.enemy_goal.big_hull)
                    return [Actions.GoToPointIgnore(next_point, angle0)]

                pint = aux.segment_poly_intersect(domain.robot.get_pos(), next_point, domain.field.ally_goal.hull)
                if pint is not None:
                    convex_hull = qh.shortesthull(domain.robot.get_pos(), next_point, domain.field.ally_goal.big_hull)
                    for j in range(len(convex_hull) - 2, 0, -1):
                        next_point = convex_hull[j]

                pint = aux.segment_poly_intersect(domain.robot.get_pos(), next_point, domain.field.enemy_goal.hull)
                if pint is not None:
                    convex_hull = qh.shortesthull(domain.robot.get_pos(), next_point, domain.field.enemy_goal.big_hull)
                    for j in range(len(convex_hull) - 2, 0, -1):
                        next_point = convex_hull[j]

            pth_wp = calc_passthrough_point(
                domain, next_point, avoid_ball=avoid_ball, ignore_ball=self.ignore_ball, ignore_robots=self.ignore_robots
            )
            if pth_wp is not None:
                target_speed = min(const.MAX_SPEED, aux.dist(pth_wp, next_point))
                target_vel = (pth_wp - domain.robot.get_pos()).unity() * target_speed
                return [Actions.GoToPointIgnore(pth_wp, angle0, target_vel=target_vel)]
            if next_point != self.target_pos:
                target_speed = min(const.MAX_SPEED / 2, aux.dist(self.target_pos, next_point))
                target_vel = (next_point - domain.robot.get_pos()).unity() * target_speed
                return [Actions.GoToPointIgnore(next_point, angle0, target_vel=target_vel)]
            return [Actions.GoToPointIgnore(self.target_pos, angle0, self.ball_interact, self.target_vel)]

    class BallPlacement(Action):
        """Move ball to target_point"""

        def __init__(self, target_point: aux.Point) -> None:
            self.target_point = target_point

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:
            current_action.dribbler_speed = 0

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list["Action"]:
            # current_action.dribbler_speed = 15
            target_angle = (-domain.field.ball.get_pos() + self.target_point).arg()
            if aux.dist(domain.field.ball.get_pos(), self.target_point) < 10:
                current_action.dribbler_speed = 0
                current_action.angle = (self.target_point - domain.robot.get_pos()).arg()
                return []
            if domain.field.is_ball_in(domain.robot):
                return [Actions.GoToPointIgnore(self.target_point, target_angle), DumbActions.LimitSpeed(500)]
            return [Actions.BallGrab(target_angle), DumbActions.LimitSpeed(700)]

    class BallGrab(Action):
        """Grab ball in a given direction"""

        def __init__(self, target_angle: float) -> None:
            self.target_angle = target_angle

        def is_defined(self, domain: ActionDomain) -> bool:
            return (
                aux.dist(domain.robot.get_pos(), domain.field.ball.get_pos()) < 3000
                and (
                    domain.robot.r_id == domain.field.gk_id
                    or (
                        not aux.is_point_inside_poly(domain.field.ball.get_pos(), domain.field.enemy_goal.hull)
                        and not aux.is_point_inside_poly(domain.field.ball.get_pos(), domain.field.ally_goal.hull)
                    )
                )
                and domain.game_state not in [GameStates.STOP, GameStates.PREPARE_KICKOFF]
                and (domain.game_state not in [GameStates.FREE_KICK, GameStates.KICKOFF] or domain.we_active)
            )

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:
            ball_pos = domain.field.ball.get_pos()
            align_pos = ball_pos - aux.rotate(aux.RIGHT, self.target_angle) * const.GRAB_ALIGN_DIST
            transl_vel = get_grab_speed(
                domain.robot.get_pos(),
                current_action.vel,
                domain.field,
                align_pos,
                self.target_angle,
            )
            transl_vel += domain.field.ball.get_vel() / 1.2

            current_action.vel = transl_vel
            current_action.angle = self.target_angle

            current_action.dribbler_speed = 15

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list["Action"]:
            ball_pos = domain.field.ball.get_pos()
            align_pos = ball_pos - aux.rotate(aux.RIGHT, self.target_angle) * const.GRAB_ALIGN_DIST
            ignore_ball = len(aux.line_circle_intersect(domain.robot.get_pos(), align_pos, ball_pos, const.ROBOT_R, "S")) < 2
            return [Actions.GoToPoint(align_pos, self.target_angle, True, ignore_ball)]

    class Velocity(Action):
        """Move robot with velocity and angle_speed"""

        def __init__(self, velocity: aux.Point, angle: float, control_angle_by_speed: bool = False) -> None:
            self.velocity = velocity
            self.angle = angle  # angle to turn / angle speed

            self.control_angle_by_speed = control_angle_by_speed

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:

            current_action.vel = self.velocity
            current_action.angle = self.angle

            if self.control_angle_by_speed:
                current_action.beep = 1

    class Kick(Action):
        """Choose type of kick (from KickActions)"""

        def __init__(
            self,
            target_pos: aux.Point,
            voltage: int = const.VOLTAGE_SHOOT,
            is_pass: bool = False,
            is_upper: bool = False,
        ) -> None:
            self.kick_args = (target_pos, voltage, is_pass, is_upper)

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list["Action"]:
            return [KickActions.Straight(*self.kick_args)]


class KickActions:
    """Class with available types of ball kicks"""

    class Kick(Action):
        """Base class"""

        def __init__(
            self,
            target_pos: aux.Point,
            voltage: int = const.VOLTAGE_SHOOT,
            is_pass: bool = False,
            is_upper: bool = False,
        ) -> None:
            self.target_pos = target_pos
            self.voltage = voltage  # ignore if is_pass
            self.is_upper = is_upper

            if self.voltage > const.VOLTAGE_SHOOT:
                self.voltage = const.VOLTAGE_SHOOT

            if self.is_upper:
                self.voltage = const.VOLTAGE_UP

            self.pass_pos: Optional[aux.Point] = None
            if is_pass:
                self.pass_pos = self.target_pos

    class Straight(Kick):
        """Grab the ball and kick it straight"""

        def use_behavior_of(self, domain: ActionDomain, current_action: ActionValues) -> list["Action"]:
            kick_angle = aux.angle_to_point(domain.field.ball.get_pos(), self.target_pos)

            actions = [
                Actions.BallGrab(kick_angle),
                DumbActions.ShootAction(self.target_pos, self.is_upper),
                DumbActions.ControlVoltageAction(self.voltage, self.pass_pos),
            ]

            return actions


class DumbActions:
    """User-unavailable actions, are used in Actions"""

    class ShootAction(Action):
        """Shoot the target when kick is aligned"""

        def __init__(self, target_pos: aux.Point, is_upper: bool = False, angle_bounds: Optional[float] = None) -> None:
            self.target_pos = target_pos
            self.autokick = 2 if is_upper else 1
            self.angle_bounds = angle_bounds

        def is_defined(self, domain: ActionDomain) -> bool:
            kick_angle = aux.angle_to_point(domain.robot.get_pos(), self.target_pos)
            is_aligned = (
                domain.robot.is_kick_aligned_by_angle(kick_angle, angle_bounds=self.angle_bounds)
                if self.angle_bounds is not None
                else domain.robot.is_kick_aligned_by_angle(kick_angle)
            )
            return domain.field.is_ball_in(domain.robot) and is_aligned

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:
            current_action.auto_kick = self.autokick

    class ControlVoltageAction(Action):
        """Control voltage before shooting"""

        def __init__(self, voltage: int = 15, pass_pos: Optional[aux.Point] = None) -> None:
            self.voltage = voltage
            self.pass_pos = pass_pos

        def is_defined(self, domain: ActionDomain) -> bool:
            return aux.dist(domain.robot.get_pos(), domain.field.ball.get_pos()) < 1000

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:
            if self.pass_pos is not None:
                self.voltage = get_pass_voltage(aux.dist(domain.robot.get_pos(), self.pass_pos))

            current_action.kicker_voltage = self.voltage
            # NOTE test 15 when is_pass

    class AddFinalVelocityAction(Action):
        """Add velocity in final target"""

        def __init__(
            self, target: aux.Point, final_velocity: aux.Point, max_dist: float = 300, min_dist: float = 100
        ) -> None:
            self.target = target
            self.final_velocity = final_velocity
            self.min_dist = min_dist
            self.max_dist = max_dist

        def is_defined(self, domain: ActionDomain) -> bool:
            return aux.dist(self.target, domain.robot.get_pos()) < self.max_dist

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:
            vec_to_target = self.target - domain.robot.get_pos()
            cur_speed = self.final_velocity * aux.minmax(
                (self.max_dist - vec_to_target.mag()) / (self.max_dist - self.min_dist), 0, 1
            )

            current_action.vel += cur_speed

    class LimitSpeed(Action):
        """Limit robot speed"""

        def __init__(self, limit: float = const.MAX_SPEED) -> None:
            self.limit = limit

        def behavior(self, domain: ActionDomain, current_action: ActionValues) -> None:
            limit_action(domain, current_action, self.limit)


def get_pass_voltage(length: float) -> int:
    """Calc voltage for pass by length"""
    if const.IS_SIMULATOR_USED:
        # TODO fix control decoder
        return int(aux.minmax(0.003 * length + 1.8, 6, const.VOLTAGE_SHOOT))
    return int(aux.minmax(0.0014 * length + 2.4, 6, const.VOLTAGE_SHOOT))


def get_grab_speed(
    robot_pos: aux.Point,
    transl_vel: aux.Point,
    field: fld.Field,
    grab_point: aux.Point,
    grab_angle: float,
    target_speed: float = 0,
) -> aux.Point:
    """Calculate speed for carefully grabbing a ball"""
    ball = field.ball.get_pos()

    point_on_center_line = aux.closest_point_on_line(ball, grab_point, robot_pos, "S")
    dist_to_center_line = aux.dist(robot_pos, point_on_center_line)
    ball_dist_center_line = aux.dist(point_on_center_line, ball)

    if ball_dist_center_line == 0:
        offset_angle = const.GRAB_OFFSET_ANGLE
    else:
        offset_angle = math.atan(dist_to_center_line / ball_dist_center_line)

    dist_to_catch = (ball - aux.rotate(aux.RIGHT, grab_angle) * const.GRAB_DIST) - robot_pos

    vel_to_catch = dist_to_catch * const.GRAB_MULT

    vel_to_catch_r = (
        aux.scal_mult(
            vel_to_catch,
            aux.rotate(aux.RIGHT, grab_angle),
        )
        + target_speed
    )

    vel_to_align_r = aux.scal_mult(
        transl_vel,
        aux.rotate(aux.RIGHT, grab_angle),
    )

    vel_to_align = transl_vel - aux.rotate(aux.RIGHT, grab_angle) * vel_to_align_r

    board = min(offset_angle / const.GRAB_OFFSET_ANGLE, 1)  # 0 - go to ball; 1 - go to grab_point

    vel_r = vel_to_catch_r * (1 - board) + vel_to_align_r * board
    vel = vel_to_align + aux.rotate(aux.RIGHT, grab_angle) * vel_r

    return vel


def convert_to_screen(
    ball_screen: aux.Point,
    scale: float,
    angle: float,
    ball: aux.Point,
    point: aux.Point,
) -> aux.Point:
    """Convert cord on field to cord on image"""
    vec_from_ball = point - ball
    scaled_vec = vec_from_ball * scale
    rotated_vec = aux.rotate(scaled_vec, angle)
    final_point = ball_screen + rotated_vec
    return final_point
