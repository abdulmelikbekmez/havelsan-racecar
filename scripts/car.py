from enum import Enum
from motionController import MotionController
from vector import Vector
from geometry_msgs.msg import PoseStamped, PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
from utils import normalize_angle, generate_tree_marker
from visualization_msgs.msg import Marker
import rospy
from tf.transformations import euler_from_quaternion
from rrt import RRT
from map import Map


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
        self.angle_heading = angle
        self.rate = rospy.Rate(20)
        self.msg = AckermannDriveStamped()
        self.msg.drive.acceleration = 0.05
        self.msg.drive.steering_angle_velocity = 0.05
        self.pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation",
                                   AckermannDriveStamped,
                                   queue_size=1)
        self.pub_marker_points = rospy.Publisher("/marker_point",
                                                 Marker,
                                                 queue_size=1)
        self.pub_marker_lines = rospy.Publisher("/marker_line",
                                                Marker,
                                                queue_size=1)

        self.pub_marker_random_points = rospy.Publisher("/random_point",
                                                        Marker,
                                                        queue_size=1)

        self.rrt = None
        self.motion_controller = MotionController()

    def __hector_cb(self, msg):
        # type: (PoseStamped) -> None
        self.pos.update(msg.pose)
        o = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.angle_heading = float(yaw)

    def __clicked_cb(self, msg):
        # type: (PointStamped) -> None
        clicked_pos = Vector(msg.point.x, msg.point.y, msg.point.z)
        map_value = self.map.get_map_coord(clicked_pos)
        if map_value == 100:
            print("this pos is already obstacle. Try another !!!")
            return
        self.state = CarState.PLANNING
        self.rrt = RRT(self.pos, self.angle_heading)
        list_route = self.rrt.get_vector_list(
            clicked_pos,
            self.map,
        )
        self.motion_controller.set_path(list_route, self.pos)
        self.state = CarState.NAVIGATE

    def subscribe(self):
        self.map.subscribe()

        rospy.Subscriber("/hector/slam_out_pose",
                         PoseStamped,
                         self.__hector_cb,
                         queue_size=1)
        rospy.Subscriber("/clicked_point",
                         PointStamped,
                         self.__clicked_cb,
                         queue_size=1)

    def __navigate(self):

        speed, angle, arrived = self.motion_controller.main(
            self.pos, self.angle_heading)
        self.msg.drive.speed = speed
        self.msg.drive.steering_angle = angle
        self.pub.publish(self.msg)
        if arrived:
            self.state = CarState.IDLE

    def main(self):

        while not rospy.is_shutdown():

            self.rate.sleep()

            if self.state is CarState.PLANNING and self.rrt:
                points, lines, random = generate_tree_marker(self.rrt)
                self.pub_marker_lines.publish(lines)
                self.pub_marker_points.publish(points)
                self.pub_marker_random_points.publish(random)
            elif self.state is CarState.NAVIGATE:
                self.__navigate()
            elif self.state is CarState.IDLE:
                pass
