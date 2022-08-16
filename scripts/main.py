#!/usr/bin/env python
from car import Car
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from vector import Vector
from utils import normalize_angle


if __name__ == "__main__":
    rospy.init_node('ilerle', anonymous=True)
    h_msg = rospy.wait_for_message('/hector/slam_out_pose',
                                   PoseStamped)  # type: (PoseStamped)
    o = h_msg.pose.orientation
    _, _, dir = euler_from_quaternion([o.x, o.y, o.z, o.w])
    dir = normalize_angle(dir)
    pos = Vector.from_pose(h_msg.pose)
    car = Car(pos, dir)
    car.subscribe()
    car.main()
    rospy.spin()
