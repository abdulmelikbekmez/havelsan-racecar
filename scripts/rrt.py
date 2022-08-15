from enum import Enum
from time import sleep
from random import random as r
from utils import refactor_angle
from node import Node, Connection
from typing import Tuple
from rospy import Duration
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker
from typing import List, TYPE_CHECKING
from vector import Vector
from nav_msgs.msg import Path

if TYPE_CHECKING:
    from map import Map
    from node import DirState


class Route:

    def __init__(self, pos, dir):
        # type: (Vector, DirState) -> None
        self.pos = pos
        self.dir = dir


class RRT:
    MARGIN_MAX = 0.11
    MARGIN_MIN = 0.05
    MAX_ANGLE = 14
    BIAS = .5
    MAX_ITER_COUNT = 10000

    def __init__(self, head_pos, angle):
        # type: (Vector, float) -> None
        self.head = Node(head_pos, angle)
        self.tail = self.head
        self.count = 1
        self.started = False
        self.stop = False

    def generate_marker(self):
        # type: () -> Tuple[Marker, Marker]
        point_marker = Marker()
        point_marker.type = Marker.POINTS
        point_marker.header.frame_id = "hector_map"
        point_marker.action = Marker.ADD
        point_marker.scale = Vector3(0.05, 0.05, 0.05)
        point_marker.color.r = 0.0
        point_marker.color.g = 1.0
        point_marker.color.b = 0.0
        point_marker.color.a = 1.0
        point_marker.lifetime = Duration(1)
        point_marker.points = []

        line_marker = Marker()
        line_marker.type = Marker.LINE_LIST
        line_marker.header.frame_id = "hector_map"
        line_marker.action = Marker.ADD
        line_marker.scale = Vector3(0.03, 0.03, 0.03)
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.lifetime = Duration(10)
        line_marker.points = []

        l = [self.head]
        while l:
            node = l.pop()
            p = Point()
            p.x = node.pos.x
            p.y = node.pos.y
            p.z = node.pos.z
            point_marker.points.append(p)
            l.extend(node.childs)
            for child in node.childs:
                p_1 = node.pos.generate_point()
                p_2 = child.pos.generate_point()
                line_marker.points.append(p_1)
                line_marker.points.append(p_2)

        return point_marker, line_marker

    def __get_closest_node(self, to):
        # type: (Vector) -> Node
        l = []
        node_list = [self.head]
        while node_list:
            node = node_list.pop()
            l.append(Connection(node, (node.pos - to).length))
            node_list.extend(node.childs)

        return min(l, key=lambda x: x.length)._from

    def refactor_angle(self, angle):
        # type: (float) -> Tuple[float, bool]
        angle = refactor_angle(angle)
        dir_changed = False
        x = -1 if angle < 0 else 1
        angle = abs(angle)
        if angle < 90:
            angle = min(self.MAX_ANGLE / 2, angle)
        else:
            angle = max(180 - self.MAX_ANGLE / 2, 180 - angle)
            dir_changed = True
        res = angle * x
        return res, dir_changed

    def refactor_length(self, length):
        # type: (float) -> float
        l = 0
        if length > self.MARGIN_MAX:
            l = self.MARGIN_MAX
        elif length < self.MARGIN_MIN:
            l = self.MARGIN_MIN
        else:
            l = length

        return l

    def __get_closest_point_from_node_with_max_angle(self, node, point):
        # type: (Node, Vector) -> Tuple[Vector, bool]
        dif = point - node.pos
        tmp_angle = dif.angle
        dif_angle = node.angle - tmp_angle
        dif_angle, dir = self.refactor_angle(dif_angle)
        length = self.refactor_length(dif.length)
        v = Vector.from_polar(length, node.angle - dif_angle)
        return node.pos + v, dir

    def generate_random_point(self, map, goal):
        # type: (Map, Vector) -> Vector
        bias = r()
        if bias < self.BIAS:
            return goal
        x = +12 + r() * -24
        y = +12 + r() * -24
        v = Vector(x, y, 0)
        return v

    def __create_new_node(self, goal, map, path):
        # type: (Vector, Map, Path) -> bool
        """
        Creates new node randomly and returns True if new node is unexpored point
        """
        random_point = self.generate_random_point(map, goal)
        node = self.__get_closest_node(random_point)
        closest_point, dir_changed = self.__get_closest_point_from_node_with_max_angle(
            node, random_point)

        coord = map.get_map_coord(closest_point)
        if coord == -1 or coord == 100:
            #print("wrong point", coord)
            return True
        child_node = node.add_child(closest_point, dir_changed)
        map.add_pos_to_map(path, closest_point)
        self.tail = child_node
        # return map.get_map_coord(closest_point) >= 0
        return (closest_point - goal).length > self.MARGIN_MAX

    def get_vector_list(self, goal, map, path):
        # type: (Vector, Map, Path) -> List[Route]
        iter_count = 0

        while self.__create_new_node(goal, map, path) and iter_count < self.MAX_ITER_COUNT:
            iter_count += 1

        if iter_count == self.MAX_ITER_COUNT:
            print("max iter count reached!!")
            return []
        tmp = self.tail
        while tmp:
            parent = tmp.parent
            if parent:
                parent.childs = [n for n in parent.childs if n is tmp]
            tmp = parent

        res = []
        tmp = self.tail
        while tmp:
            r = Route(tmp.pos, tmp.dir)
            res.insert(0, r)
            tmp = tmp.parent
        return res
