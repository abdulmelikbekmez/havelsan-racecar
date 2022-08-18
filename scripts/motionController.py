from rrt import Route
import numpy as np
from vector import Vector
from node import DirState
from typing import Union
from utils import normalize_angle
from geometry_msgs.msg import PoseStamped
from math import pi
from rospy import Publisher
from nav_msgs.msg import Path


class MotionController:
    SPEED = 0.3

    def __init__(self):
        # type: () -> None
        self.list_route = []  # type: list[Route]
        self.pos_prev = None  # type: Union[None, Vector]
        self.pub_path = Publisher("/path", Path, queue_size=1)
        self.msg_path = Path()
        self.msg_path.header.frame_id = "hector_map"

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
        return a

    def main(self, position, angle_heading):
        # type: (Vector, float) -> tuple[float, float, bool]
        """
        returns speed and angle 
        """
        if not self.list_route:
            return 0, 0, True
        route = self.list_route[0]
        target = route.pos

        dir_angle = angle_heading if route.dir is DirState.FORWARD else angle_heading - pi

        dif = target - position
        print("direction => ", route.dir)
        print("nav - remaining length => ", dif.length)
        dif_angle = dif.angle - dir_angle
        # dif_angle = refactor_angle2(dif_angle)
        dif_angle = normalize_angle(dif_angle)
        print("nav - angle => ", dif_angle)

        speed = self.SPEED * route.dir.value
        angle = self.__normalize_angle(dif_angle) * route.dir.value

        if dif.length < 0.20:
            self.__pop()

        self.pub_path.publish(self.msg_path)
        return speed, angle, False
