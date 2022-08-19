from enum import Enum
from turtle import pos
from rrt import Route
import numpy as np
from vector import Vector
from node import DirState
from typing import Union
from utils import normalize_angle
from geometry_msgs.msg import PoseStamped
from math import pi, acos, sin
from rospy import Publisher
from nav_msgs.msg import Path


class PathState(Enum):
    BACK = 0
    INSIDE = 1
    FRONT = 2


class MotionController:
    SPEED = 0.5
    ANGLE_MAX = pi / 10

    def __init__(self):
        # type: () -> None
        self.list_route = []  # type: list[Route]
        self.pos_prev = None  # type: Union[None, Vector]
        self.pub_path = Publisher("/path", Path, queue_size=1)
        self.msg_path = Path()
        self.msg_path.header.frame_id = "hector_map"

    def __get_distance(self, start, finish, position):
        # type: (Vector, Vector, Vector) -> float
        dir_line = finish - start
        dir_car = position - start
        tmp_length = dir_car.length
        if not dir_car.length > 0:
            return 0
        try:
            angle = acos(
                dir_line.dot(dir_car) / (dir_car.length * dir_line.length))
        except ZeroDivisionError:
            print(dir_car, dir_line)
            raise ZeroDivisionError
        return sin(angle) * tmp_length

    def __get_state(self, start, finish, current):
        # type: (Vector, Vector, Vector) -> PathState
        dir_line = finish - start
        if dir_line.dot(current - finish) >= 0:
            return PathState.FRONT

        if dir_line.dot(current - start) <= 0:
            return PathState.BACK

        if dir_line.dot(current - finish) <= 0 and dir_line.dot(current -
                                                                start) >= 0:
            return PathState.INSIDE

        print(start, finish, current)
        raise Exception("wrong!!")

    def set_path(self, list_route, start_pos):
        # type: (list[Route], Vector) -> None
        self.list_route = list_route
        self.pos_prev = start_pos

        self.msg_path.poses = []
        for r in list_route:
            p = PoseStamped()
            p.pose.position.x = r.pos.x
            p.pose.position.y = r.pos.y
            p.pose.position.z = r.pos.z
            p.header.frame_id = "hector_map"
            self.msg_path.poses.append(p)

    def __pop(self):
        popped = self.list_route.pop(0)
        self.pos_prev = popped.pos

        self.msg_path.poses.pop(0)  # type: ignore

    def __normalize_angle(self, angle):
        # type: (float) -> float
        a = np.exp(angle / 2) - 1  # type: ignore
        # print("angle: " + str(a))
        return min(a, self.ANGLE_MAX)

    def main(self, position, angle_heading):
        # type: (Vector, float) -> tuple[float, float, bool]
        """
        returns speed and angle 
        """

        if not self.pos_prev:
            raise Exception(" pos prev must be exist!!")

        if not self.list_route:
            return 0, 0, True

        route = self.list_route[0]
        target = route.pos

        dir_angle = angle_heading if route.dir is DirState.FORWARD else angle_heading - pi

        dif = target - position
        # print("direction => ", route.dir)
        # print("nav - remaining length => ", dif.length)
        dif_angle = dif.angle - dir_angle
        # dif_angle = refactor_angle2(dif_angle)
        dif_angle = normalize_angle(dif_angle)
        # print("nav - angle => ", dif_angle)

        speed = self.SPEED * route.dir.value
        angle = self.__normalize_angle(dif_angle) * route.dir.value

        state = self.__get_state(self.pos_prev, target, position)

        print(state)
        print(route.dir)
        print("distance from path => ",
              self.__get_distance(self.pos_prev, target, position))
        # if state is PathState.FRONT:
        #     self.__pop()

        if dif.length < 0.20 or state is PathState.FRONT:
            self.__pop()

        self.pub_path.publish(self.msg_path)
        return speed, angle, False
