import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
from typing import List, TYPE_CHECKING
from vector import Vector
from nav_msgs.msg import Path
from random import randint as rint

if TYPE_CHECKING:
    from rrt import Route


class Map:
    THICKNESS = 6

    def __init__(self):
        grid = rospy.wait_for_message(
            "/hector/map", OccupancyGrid
        )  # type: (OccupancyGrid)
        self.width = grid.info.width
        print("map width => ", self.width)
        self.height = grid.info.height
        print("map height => ", self.height)
        self.resolution = grid.info.resolution
        print("map resolution => ", self.resolution)
        print("origin => ", grid.info.origin)
        self.saved = False
        self.map_publisher = rospy.Publisher("/new_map", OccupancyGrid, queue_size=1)
        self.x_max = 0
        self.x_min = 0
        self.y_max = 0
        self.y_min = 0

    def generate_random_point(self):
        x = rint(self.x_min, self.x_max)
        y = rint(self.y_min, self.y_max)
        # print("x_max = {} - x_min = {} - y_max = {} - y_min = {}".format(self.x_max, self.x_min, self.y_max, self.y_min))

        x = (x - (self.width / 2)) * self.resolution
        y = (y - (self.height / 2)) * self.resolution
        v = Vector(x, y, 0)
        # print("generated point => {}".format(v))
        return v

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
        path.poses.append(p)  # type: ignore

    def __map_cb(self, msg):
        # type: (OccupancyGrid) -> None

        data = np.array(msg.data).reshape((int(self.width), int(self.height))).T

        data_copy = np.copy(data).flatten()
        indices = np.argwhere(data_copy == 100)
        data_copy = np.copy(data)
        for i in indices:

            x = i.item() // self.width
            y = i.item() % self.height

            data_copy[
                x - self.THICKNESS : x + self.THICKNESS,
                y - self.THICKNESS : y + self.THICKNESS,
            ] = 100
        self.data = data_copy

        indices = np.argwhere(self.data == 0)
        x_list = [i[0] for i in indices]
        y_list = [i[1] for i in indices]
        self.x_max = max(x_list)
        self.x_min = min(x_list)

        self.y_max = max(y_list)
        self.y_min = min(y_list)

        msg.data = self.data.T.flatten().tolist()
        self.map_publisher.publish(msg)
        # if not self.saved:
        #     plt.imshow(self.data)
        #     plt.savefig("../img/visualize_numpy_array_0" + ".png", dpi=1000)
        #     self.saved = True

    def subscribe(self):
        rospy.Subscriber("/hector/map", OccupancyGrid, self.__map_cb, queue_size=1)
