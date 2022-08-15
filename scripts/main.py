#!/usr/bin/env python
from car import Car
from map import Map
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import threading
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, numpy as np
from tf.transformations import euler_from_quaternion
from vector import Vector
# counter=0
# msg = AckermannDriveStamped()
# flag = False
# posx = 500
# posy = 500
# counter = 0

# def vesc_callback(speed, angle):
#     global msg
#     msg.drive.speed = speed
#     msg.drive.steering_angle = angle
#     pub.publish(msg)

# def map_callback(map_data):
#     # type: (OccupancyGrid) ->None
#     global counter
#     data = np.asarray(map_data.data)
#     data[data == -1] = -100
#     data[data == 0] = 1
#     data_metadata = map_data.info
#     data_size = data.shape[0]
#     data = data.reshape(int(data_metadata.width), int(data_metadata.height))

#     origin = data_metadata.origin
#     print("origin => ", origin)

#     data_copy = np.copy(data).flatten()
#     indices = np.argwhere(data_copy==100)
#     data_copy=np.copy(data)
#     for i in indices:

#         x=i.item()//4000
#         y=i.item()%4000

#         data_copy[x-5:x+5, y-5:y+5] = 100
#     plt.imshow(data)
#     plt.savefig("./visualize_numpy_array_0" + str(counter) + ".png", dpi=1000 )
#     counter +=1
#     print("gg")


def refactor_angle(angle):
    if angle < -180:
        angle += 360
    elif angle > 180:
        angle -= 360
    return angle


# def hector_callback(msg):
#     # type: (PoseStamped) -> None
#     p = msg.pose.position
#     o = msg.pose.orientation

#     _ ,_ ,yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
#     #print("yaw => ", yaw)
#     #print("pose => ",p)

if __name__ == "__main__":
    rospy.init_node('ilerle', anonymous=True)
    h_msg = rospy.wait_for_message('/hector/slam_out_pose',
                                   PoseStamped)  # type: (PoseStamped)
    #rospy.Subscriber('/hector/map', OccupancyGrid, map_callback, queue_size=1)
    o = h_msg.pose.orientation
    _, _, dir = euler_from_quaternion([o.x, o.y, o.z, o.w])
    dir = refactor_angle(np.rad2deg(dir))  #type:ignore
    pos = Vector.from_pose(h_msg.pose)
    car = Car(pos, dir)
    car.subscribe()
    car.main()
    rospy.spin()
