#!/usr/bin/env python
from car import Car
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
from vector import Vector
from utils import normalize_angle
from ackermann_msgs.msg import AckermannDriveStamped
from random import random


class MoveBase:

    def __init__(self):
        self.pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation",
                                   AckermannDriveStamped,
                                   queue_size=1)
        self.m = AckermannDriveStamped()
        self.rate = rospy.Rate(10)

    def sub(self):
        rospy.Subscriber("/cmd_vel", Twist, self.__cb, queue_size=1)

    def __cb(self, msg):
        # type: (Twist) -> None
        self.m.drive.speed = msg.linear.x

        self.m.drive.steering_angle = msg.angular.z
        self.pub.publish(self.m)

    def main(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.m)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node('ilerle', anonymous=True)
    m = MoveBase()
    m.sub()
    # h_msg = rospy.wait_for_message('/hector/slam_out_pose',
    #                                PoseStamped)  # type: (PoseStamped)
    # o = h_msg.pose.orientation
    # _, _, dir = euler_from_quaternion([o.x, o.y, o.z, o.w])
    # dir = normalize_angle(dir)
    # pos = Vector.from_pose(h_msg.pose)
    # car = Car(pos, dir)
    # car.subscribe()
    # car.main()
    rospy.spin()
