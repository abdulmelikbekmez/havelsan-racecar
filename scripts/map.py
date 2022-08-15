import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
from typing import Tuple, List, TYPE_CHECKING
from vector import Vector
from nav_msgs.msg import Path
import matplotlib.pyplot as plt

if TYPE_CHECKING:
    from rrt import Route


class Map:
    THICKNESS = 8

    def __init__(self):
        grid = rospy.wait_for_message('/hector/map',
                                      OccupancyGrid)  # type: (OccupancyGrid)
        self.width = grid.info.width
        print("map width => ", self.width)
        self.height = grid.info.height
        print("map height => ", self.height)
        self.resolution = grid.info.resolution
        print("map resolution => ", self.resolution)
        print("origin => ", grid.info.origin)
        self.saved = False
        self.map_publisher = rospy.Publisher('/new_map',
                                             OccupancyGrid,
                                             queue_size=1)

    def get_map_coord(self, point):
        # type: (Vector) -> int
        x = int((self.width / 2) + (point.x / self.resolution))
        y = int((self.height / 2) + (point.y / self.resolution))
        return self.data[x][y]

    @staticmethod
    def set_path(list_vector, path):
        # type: (List[Route], Path) -> None
        path.poses = []
        for r in list_vector:
            p = PoseStamped()
            p.pose.position.x = r.pos.x
            p.pose.position.y = r.pos.y
            p.pose.position.z = r.pos.z
            p.header.frame_id = "hector_map"
            path.poses.append(p)

    @staticmethod
    def add_pos_to_map(path, pos):
        # type: (Path, Vector) -> None
        p = PoseStamped()
        p.pose.position.x = pos.x
        p.pose.position.y = pos.y
        p.pose.position.z = pos.z
        p.header.frame_id = "hector_map"
        path.poses.append(p)

    def __map_cb(self, msg):
        # type: (OccupancyGrid) -> None

        data = np.array(msg.data).reshape(
            (int(self.width), int(self.height))).T

        data_copy = np.copy(data).flatten()
        indices = np.argwhere(data_copy == 100)
        data_copy = np.copy(data)
        for i in indices:

            x = i.item() // self.width
            y = i.item() % self.height

            data_copy[x - self.THICKNESS:x + self.THICKNESS,
                      y - self.THICKNESS:y + self.THICKNESS] = 100
        self.data = data_copy

        msg.data = self.data.T.flatten().tolist()
        self.map_publisher.publish(msg)
        # if not self.saved:
        #     plt.imshow(self.data)
        #     plt.savefig("../img/visualize_numpy_array_0" + ".png", dpi=1000)
        #     self.saved = True

    def subscribe(self):
        rospy.Subscriber('/hector/map',
                         OccupancyGrid,
                         self.__map_cb,
                         queue_size=1)
