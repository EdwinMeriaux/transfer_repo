"""ROS-independent geometry and state machines for simple BlueBoat field runs."""
from dataclasses import dataclass
import math

import numpy as np


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(quaternion):
    x, y, z, w = quaternion
    if not np.isfinite([x, y, z, w]).all():
        raise ValueError('Nonfinite orientation')
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not 0.98 <= norm <= 1.02:
        raise ValueError('Orientation quaternion is not normalized')
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))


@dataclass(frozen=True)
class Command:
    surge_m_s: float
    yaw_rate_rad_s: float
    stage: str
    complete: bool = False


class MissionController:
    """Position-feedback mission controller whose output matches the PPO command contract."""

    def __init__(self, mission, *, distance_m=20.0, return_distance_m=18.0, center_distance_m=25.0,
                 return_offset_m=2.0, diameter_m=12.0, speed_m_s=0.4, max_yaw_rate_rad_s=0.3,
                 arrival_radius_m=0.6):
        if mission not in ('out-and-back', 'center-circle'):
            raise ValueError('Unknown mission')
        values = (distance_m, return_distance_m, center_distance_m, return_offset_m, diameter_m, speed_m_s,
                  max_yaw_rate_rad_s, arrival_radius_m)
        if not np.isfinite(values).all() or any(value <= 0 for value in values):
            raise ValueError('Mission dimensions and limits must be finite and positive')
        if speed_m_s > 0.8 or max_yaw_rate_rad_s > 0.4:
            raise ValueError('Mission exceeds PPO command limits')
        if mission == 'out-and-back' and return_distance_m > distance_m:
            raise ValueError('Return distance cannot exceed outbound distance')
        if mission == 'center-circle' and return_offset_m >= center_distance_m:
            raise ValueError('Circle return offset must be inside the center distance')
        self.mission = mission
        self.distance = float(distance_m)
        self.return_distance = float(return_distance_m)
        self.center_distance = float(center_distance_m)
        self.return_offset = float(return_offset_m)
        self.radius = float(diameter_m) / 2.0
        self.speed = float(speed_m_s)
        self.max_yaw_rate = float(max_yaw_rate_rad_s)
        self.arrival_radius = float(arrival_radius_m)
        self.origin = self.initial_yaw = None
        self.stage = 'waiting'
        self.target = self.center = self.rim = None
        self.turn_target = None
        self.circle_last_angle = None
        self.circle_progress = 0.0
        self.stable_ticks = 0

    def start(self, position_xy, yaw):
        position = np.asarray(position_xy, dtype=float)
        if position.shape != (2,) or not np.isfinite(position).all() or not math.isfinite(yaw):
            raise ValueError('Invalid starting pose')
        self.origin = position.copy()
        self.initial_yaw = float(yaw)
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        if self.mission == 'out-and-back':
            self.stage = 'outbound'
            self.target = self.origin + self.distance * forward
        else:
            self.stage = 'to_center'
            self.target = self.origin + self.center_distance * forward

    def _rotate(self, yaw, target, stage):
        error = wrap_angle(target - yaw)
        if abs(error) < math.radians(4.0):
            self.stable_ticks += 1
        else:
            self.stable_ticks = 0
        rate = float(np.clip(1.2 * error, -self.max_yaw_rate, self.max_yaw_rate))
        return Command(0.0, rate, stage), self.stable_ticks >= 5

    def _drive(self, position, yaw, target, stage):
        delta = target - position
        distance = float(np.linalg.norm(delta))
        if distance <= self.arrival_radius:
            return Command(0.0, 0.0, stage), True
        bearing = math.atan2(delta[1], delta[0])
        error = wrap_angle(bearing - yaw)
        yaw_rate = float(np.clip(1.4 * error, -self.max_yaw_rate, self.max_yaw_rate))
        alignment = max(0.0, math.cos(error))
        surge = min(self.speed, max(0.10, 0.55 * distance)) * alignment
        if abs(error) > math.radians(70.0):
            surge = 0.0
        return Command(float(surge), yaw_rate, stage), False

    def _circle(self, position, yaw):
        radial = position - self.center
        distance = float(np.linalg.norm(radial))
        if distance < 0.25:
            return Command(0.0, self.max_yaw_rate, 'circle'), False
        angle = math.atan2(radial[1], radial[0])
        if self.circle_last_angle is None:
            self.circle_last_angle = angle
        else:
            delta = wrap_angle(angle - self.circle_last_angle)
            # This mission travels clockwise. Ignore impossible one-cycle jumps.
            if abs(delta) < math.radians(45.0):
                self.circle_progress = max(0.0, self.circle_progress - delta)
            self.circle_last_angle = angle
        radial_unit = radial / distance
        tangent_clockwise = np.array([radial_unit[1], -radial_unit[0]])
        correction = -0.55 * (distance - self.radius) * radial_unit
        direction = tangent_clockwise + correction
        desired_yaw = math.atan2(direction[1], direction[0])
        error = wrap_angle(desired_yaw - yaw)
        yaw_rate = float(np.clip(1.5 * error, -self.max_yaw_rate, self.max_yaw_rate))
        surge = self.speed * max(0.0, math.cos(error))
        if abs(error) > math.radians(70.0):
            surge = 0.0
        complete = self.circle_progress >= 2.0 * math.pi and np.linalg.norm(position - self.rim) <= 1.2
        return Command(float(surge), yaw_rate, 'circle', complete), complete

    def update(self, position_xy, yaw):
        position = np.asarray(position_xy, dtype=float)
        if position.shape != (2,) or not np.isfinite(position).all() or not math.isfinite(yaw):
            raise ValueError('Invalid mission pose')
        if self.origin is None:
            self.start(position, yaw)

        if self.mission == 'out-and-back':
            if self.stage == 'outbound':
                command, arrived = self._drive(position, yaw, self.target, self.stage)
                if arrived:
                    self.stage = 'turn_180'
                    self.turn_target = wrap_angle(self.initial_yaw + math.pi)
                    self.stable_ticks = 0
                return command
            if self.stage == 'turn_180':
                command, aligned = self._rotate(yaw, self.turn_target, self.stage)
                if aligned:
                    self.stage = 'return'
                    forward = np.array([math.cos(self.initial_yaw), math.sin(self.initial_yaw)])
                    self.target = self.origin + (self.distance - self.return_distance) * forward
                return command
            if self.stage == 'return':
                command, arrived = self._drive(position, yaw, self.target, self.stage)
                if arrived:
                    self.stage = 'complete'
                    return Command(0.0, 0.0, self.stage, True)
                return command

        if self.stage == 'to_center':
            command, arrived = self._drive(position, yaw, self.target, self.stage)
            if arrived:
                # The actual reached point is the requested center. Move one radius
                # to its left, then trace a clockwise circle so the initial tangent
                # is the original starting direction.
                self.center = position.copy()
                left = np.array([-math.sin(self.initial_yaw), math.cos(self.initial_yaw)])
                self.rim = self.center + self.radius * left
                self.target = self.rim.copy()
                self.stage = 'to_rim'
            return command
        if self.stage == 'to_rim':
            command, arrived = self._drive(position, yaw, self.target, self.stage)
            if arrived:
                self.stage = 'align_tangent'
                self.turn_target = self.initial_yaw
                self.stable_ticks = 0
            return command
        if self.stage == 'align_tangent':
            command, aligned = self._rotate(yaw, self.turn_target, self.stage)
            if aligned:
                self.stage = 'circle'
                self.circle_last_angle = math.atan2(position[1]-self.center[1], position[0]-self.center[0])
            return command
        if self.stage == 'circle':
            command, complete = self._circle(position, yaw)
            if complete:
                self.stage = 'circle_return'
                forward = np.array([math.cos(self.initial_yaw), math.sin(self.initial_yaw)])
                self.target = self.origin + self.return_offset * forward
                return Command(0.0, 0.0, self.stage)
            return command
        if self.stage == 'circle_return':
            command, arrived = self._drive(position, yaw, self.target, self.stage)
            if arrived:
                self.stage = 'complete'
                return Command(0.0, 0.0, self.stage, True)
            return command

        return Command(0.0, 0.0, 'complete', True)
