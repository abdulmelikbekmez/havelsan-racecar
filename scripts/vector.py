from geometry_msgs.msg import Pose, Point
import numpy as np
from math import cos, sin


class Vector:

    def __init__(self, x, y, z):
        # type: (float, float, float) -> None
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def from_pose(cls, pos):
        # type: (Pose) -> Vector
        return cls(pos.position.x, pos.position.y, pos.position.z)

    @classmethod
    def from_polar(cls, length, angle):
        # type: (float, float) -> Vector
        x = cos(angle)
        y = sin(angle)
        return cls(x, y, 0) * length

    def generate_point(self):
        # type: () -> Point
        p = Point()
        p.x = self.x
        p.y = self.y
        p.z = self.z
        return p

    def __repr__(self):
        return "x:{:.2f} y:{:.2f} z:{:.2f}".format(self.x, self.y, self.z)

    def update(self, pose):
        # type: (Pose) -> None
        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z

    def __add__(self, other):
        if not isinstance(other, Vector):
            raise Exception("wrong type")

        return Vector(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        if not isinstance(other, Vector):
            raise Exception("wrong type")

        return Vector(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        if isinstance(other, Vector):
            return Vector(self.x * other.x, self.y * other.y, self.z * other.z)

        elif isinstance(other, int) or isinstance(other, float):
            return Vector(self.x * other, self.y * other, self.z * other)

        raise Exception("Unsupported Operand!!")

    @property
    def angle(self):
        # type: () -> float
        return np.arctan2(self.y, self.x)  #type:ignore

    @property
    def length(self):
        # type: () -> float
        return (self.x**2 + self.y**2 + self.z**2)**.5

    def angle_to(self, other):
        # type: (Vector) -> float
        return other.angle - self.angle

    def dot(self, other):
        # type: (Vector) -> float
        return self.x * other.x + self.y * other.y
