from random import random as r
from utils import normalize_angle
from node import Node
from typing import Tuple, Union
from rospy import Duration
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker
from typing import List, TYPE_CHECKING
from vector import Vector
from nav_msgs.msg import Path
from math import pi

if TYPE_CHECKING:
    from map import Map
    from node import DirState


class Route:

    def __init__(self, pos, dir):
        # type: (Vector, DirState) -> None
        self.pos = pos
        self.dir = dir


class RRT:
    BIAS = .5
    MAX_ITER_COUNT = 4000

    def __init__(self, head_pos, angle):
        # type: (Vector, float) -> None
        self.head = Node(head_pos,0, angle)
        self.started = False
        self.stop = False
        self.finded_node = None # type: Union[None, Node]

        self.random_points = []


    
    def __iter__(self):
        self.__iter_list = [self.head]
        return self
    
    def __next__(self):
        if not self.__iter_list:
            raise StopIteration()

        node = self.__iter_list.pop()
        self.__iter_list.extend(node.childs)
        return node

    def next(self):
        if not self.__iter_list:
            raise StopIteration()

        node = self.__iter_list.pop()
        self.__iter_list.extend(node.childs)
        return node

    def generate_marker(self):
        # type: () -> Tuple[Marker, Marker, Marker]
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



        random_marker = Marker()
        random_marker.type = Marker.POINTS
        random_marker.header.frame_id = "hector_map"
        random_marker.action = Marker.ADD
        random_marker.scale = Vector3(0.05, 0.05, 0.05)
        random_marker.color.r = 0.0
        random_marker.color.g = 1.0
        random_marker.color.b = 0.0
        random_marker.color.a = 1.0
        random_marker.lifetime = Duration(1)
        random_marker.points = self.random_points

        l = [self.head]
        while l:
            node = l.pop()
            p = node.pos.generate_point()
            point_marker.points.append(p)
            l.extend(node.childs)
            for child in node.childs:
                p_1 = node.pos.generate_point()
                p_2 = child.pos.generate_point()
                line_marker.points.append(p_1)
                line_marker.points.append(p_2)

        return point_marker, line_marker, random_marker

    def __get_closest_node(self, to):
        # type: (Vector) -> Node
        return min([node for node in self], key=lambda node: (node.pos - to).length)

    def refactor_angle(self, angle, closest_node):
        # type: (float, Node) -> Tuple[float, bool]
        angle = normalize_angle(angle)
        dir_changed = False
        x = -1 if angle < 0 else 1
        angle = abs(angle)
        if angle < pi / 2:
            angle = min(Node.MAX_ANGLE / 2, angle)
        else:
            angle = max(pi - Node.MAX_ANGLE / 2, pi - angle)
            dir_changed = True
        res = angle * x
        return res, dir_changed


    def __get_closest_point_from_node_with_max_angle(self, closest_node, random_point):
        # type: (Node, Vector) -> Tuple[Vector, bool]
        direction = random_point - closest_node.pos
        dif_angle = closest_node.direction.angle_to(direction)
        # print("dif angle", dif_angle)
        dif_angle_refactored, dir = self.refactor_angle(dif_angle, closest_node)
        # print("dif angle refactored", dif_angle_refactored)
        length = Node.refactor_length(direction.length)
        v = Vector.from_polar(length, closest_node.angle + dif_angle_refactored)
        return closest_node.pos + v, dir

    def generate_random_point(self, map, goal):
        # type: (Map, Vector) -> Vector
        bias = r()
        if bias < self.BIAS:
            return goal
        return map.generate_random_point()
    
    def get_best_parent(self, closest_point, current_parent):
        # type: (Vector, Node) -> Node
        l = [node for node in self if node.is_in_range(closest_point)]
        return (
            min(l, key=lambda node: node.cost + (node.pos - closest_point).length)
            if l
            else current_parent
        )
    
    def __rewire(self, possible_parent):
        # type: (Node) -> None
        neighbours = [node for node in self if possible_parent.can_be_child(node)]
        for neighbour in neighbours:
            cost = (neighbour.pos - possible_parent.pos).length
            new_cost = possible_parent.cost + cost
            if new_cost < neighbour.cost:
                neighbour.update_parent(possible_parent, new_cost)



    def __create_new_node(self, goal, map, path):
        # type: (Vector, Map, Path) -> bool
        """
        Creates new node randomly and returns True if new node is unexpored point
        """
        random_point = self.generate_random_point(map, goal)
        p =  random_point.generate_point()
        # print(p)
        self.random_points.append(p)
        closest_node = self.__get_closest_node(random_point)
        closest_point, dir_changed = self.__get_closest_point_from_node_with_max_angle(
            closest_node, random_point)

        coord = map.get_map_coord(closest_point)
        if coord == -1 or coord == 100: 
            #print("wrong point", coord)
            return True
        
        parent = self.get_best_parent(closest_point, closest_node)
        child_node = parent.add_child(closest_point, dir_changed)
        self.__rewire(child_node)
        map.add_pos_to_map(path, closest_point)
        # return map.get_map_coord(closest_point) >= 0
        if not child_node.is_close_enough(goal):
            return True
        else:
            self.finded_node = child_node
            return False

    def get_vector_list(self, goal, map, path):
        # type: (Vector, Map, Path) -> List[Route]
        iter_count = 0

        while self.__create_new_node(goal, map, path) and iter_count < self.MAX_ITER_COUNT:
            iter_count += 1

        if iter_count == self.MAX_ITER_COUNT:
            print("max iter count reached!!")
            return []
        tmp = self.finded_node
        while tmp:
            parent = tmp.parent
            if parent:
                parent.childs = [n for n in parent.childs if n is tmp]
            tmp = parent

        res = []
        tmp = self.finded_node
        while tmp:
            r = Route(tmp.pos, tmp.dir)
            res.insert(0, r)
            tmp = tmp.parent
        return res
