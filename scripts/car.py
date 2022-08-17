from enum import Enum
from node import DirState
from vector import Vector
from geometry_msgs.msg import PoseStamped, PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
from utils import normalize_angle, generate_tree_marker
from visualization_msgs.msg import Marker
import rospy
from rrt import RRT, Route
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
import numpy as np
from map import Map
from math import pi

class CarState(Enum):
    PLANNING = 0
    NAVIGATE = 1
    IDLE = 2



class Car:
    MAX_ANGLE = 0.3
    SPEED = 0.5

    def __init__(self, pos, angle):
        # type: (Vector, float) -> None

        self.state = CarState.IDLE
        self.map = Map()
        self.pos = pos
        self.angle = angle
        self.path = []  # type: list[Route]
        self.rate = rospy.Rate(10)
        self.msg = AckermannDriveStamped()
        self.pub = rospy.Publisher(
            "/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1
        )
        self.pub_point = rospy.Publisher("/path", Path, queue_size=1)
        self.msg_point = Path()
        self.msg_point.header.frame_id = "hector_map"
        self.pub_marker_points = rospy.Publisher("/marker_point", Marker, queue_size=1)
        self.pub_marker_lines = rospy.Publisher("/marker_line", Marker, queue_size=1)

        self.pub_marker_random_points = rospy.Publisher(
            "/random_point", Marker, queue_size=1
        )

        self.rrt = None

    def __hector_cb(self, msg):
        # type: (PoseStamped) -> None
        self.pos.update(msg.pose)
        o = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.angle = normalize_angle(yaw)


    def __clicked_cb(self, msg):
        # type: (PointStamped) -> None
        clicked_pos = Vector(msg.point.x, msg.point.y, msg.point.z)
        val = self.map.get_map_coord(clicked_pos)
        if val == 100:
            print("this pos is already obstacle. Try another !!!")
            return
        self.state = CarState.PLANNING
        self.rrt = RRT(self.pos, self.angle)
        route_list = self.rrt.get_vector_list(clicked_pos, self.map, self.msg_point)
        self.path = route_list
        self.map.set_path(route_list, self.msg_point)
        self.state = CarState.NAVIGATE

    def subscribe(self):
        self.map.subscribe()

        rospy.Subscriber(
            "/hector/slam_out_pose", PoseStamped, self.__hector_cb, queue_size=1
        )
        rospy.Subscriber(
            "/clicked_point", PointStamped, self.__clicked_cb, queue_size=1
        )

    def __normalize_angle(self, angle):
        a = np.exp(angle / 2) - 1  # type: ignore
        # print("angle: " + str(a))
        return a

    def __navigate(self):

        if not self.path:
            return


        route = self.path[0]
        target = route.pos

        dir_angle = self.angle if route.dir is DirState.FORWARD else self.angle - pi

        dif = target - self.pos
        print("direction => ", route.dir)
        print("nav - remaining length => ", dif.length)
        dif_angle = dif.angle - dir_angle
        # dif_angle = refactor_angle2(dif_angle)
        dif_angle = normalize_angle(dif_angle)
        print("nav - angle => ", dif_angle)

        self.msg.drive.speed = self.SPEED * route.dir.value
        self.msg.drive.steering_angle = (
            self.__normalize_angle(dif_angle) * route.dir.value
        )

        if dif.length < 0.20:
            self.path.pop(0)
            return
        print("publishing")
        # self.pub.publish(self.msg)

    def main(self):

        while not rospy.is_shutdown():

            # self.pub_point.publish(self.msg_point)
            self.rate.sleep()

            if self.state is CarState.PLANNING and self.rrt:
                points, lines, random = generate_tree_marker(self.rrt)
                self.pub_marker_lines.publish(lines)
                self.pub_marker_points.publish(points)
                self.pub_marker_random_points.publish(random)
            elif self.state is CarState.NAVIGATE:
                self.__navigate()
