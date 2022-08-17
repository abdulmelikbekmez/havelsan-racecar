from math import pi
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from typing import Tuple, TYPE_CHECKING
from rospy import Duration
if TYPE_CHECKING:
    from rrt import RRT

def normalize_angle(angle):
    # type: (float) -> float
    if angle < -pi:
        angle += 2 * pi
    elif angle > pi:
        angle -= 2 * pi
    return angle

def generate_tree_marker(tree):
    # type: (RRT) -> Tuple[Marker, Marker, Marker]
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
    random_marker.points = tree.random_points

    l = [tree.head]
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
