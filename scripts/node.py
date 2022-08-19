from enum import Enum
import numpy as np
from math import sin, cos, pi
from vector import Vector


class DirState(Enum):
    FORWARD = 1
    BACKWARD = -1


class Node:
    ID = 0
    MARGIN_MAX = 0.25
    MARGIN_MIN = 0.15
    MAX_ANGLE = pi / 10
    NEIGHBOUR_MAX = MARGIN_MAX + .5

    def __init__(self, pos, angle, cost, dir=DirState.FORWARD, parent=None):
        # type: (Vector, float, float, DirState, Node| None) -> None
        self.pos = pos
        self.dir = dir
        self.angle = angle
        self.parent = parent
        self.childs = []  # type: list[Node]
        self.id = Node.ID
        Node.ID += 1
        self.cost = cost

    @property
    def direction(self):
        # type: () -> Vector
        x = cos(self.angle)
        y = sin(self.angle)
        return Vector(x, y, 0)

    def __eq__(self, other):
        # type: (object) -> bool
        if not isinstance(other, Node):
            return False

        return other.id == self.id

    @classmethod
    def refactor_length(cls, length):
        # type: (float) -> float
        if length > cls.MARGIN_MAX:
            return cls.MARGIN_MAX
        elif length < cls.MARGIN_MIN:
            return cls.MARGIN_MIN
        else:
            return length

    def can_be_child(self, other):
        # type: (Node) -> bool
        direction = other.pos - self.pos
        if direction.length > self.NEIGHBOUR_MAX:
            return False
        angle = self.direction.angle_to(direction)
        angle_abs = abs(angle)
        if angle_abs > 90 and not 180 - angle_abs > 180 - self.MARGIN_MAX / 2:
            return False
        elif not angle_abs < self.MAX_ANGLE / 2:
            return False

        new_angle = self.get_angle_from_child(other.pos)
        for child in other.childs:
            dif = abs(child.angle - new_angle)
            if dif > self.MAX_ANGLE / 2:
                return False

        return True

    def is_in_range(self, point):
        # type: (Vector) -> tuple[bool, bool]
        direction = point - self.pos
        if direction.length > self.NEIGHBOUR_MAX:
            return False, False
        angle = abs(self.direction.angle_to(direction))
        if angle > 90:
            return 180 - angle > 180 - self.MARGIN_MAX / 2, True
        else:
            return angle < self.MAX_ANGLE / 2, False

    def is_close_enough(self, goal_pos):
        # type: (Vector) -> bool
        return (self.pos - goal_pos).length < self.MARGIN_MAX

    def get_angle_from_child(self, child):
        # type: (Vector) -> float
        direction = child - self.pos
        return np.arctan2(direction.y, direction.x)  # type: ignore

    def add_child(self, pos, dir_changed):
        # type: (Vector, bool) -> Node
        angle = self.get_angle_from_child(pos)
        if self.dir is DirState.FORWARD:
            if dir_changed:
                dir = DirState.BACKWARD
            else:
                dir = DirState.FORWARD
        else:
            if dir_changed:
                dir = DirState.FORWARD
            else:
                dir = DirState.BACKWARD

        cost = self.cost + (self.pos - pos).length
        child = Node(pos, angle, cost, dir, self)
        self.childs.append(child)
        return child

    def update_childs_cost(self):
        for child in self.childs:
            new_cost = (self.pos - child.pos).length
            child.cost = self.cost + new_cost
            child.update_childs_cost()

    def update_parent(self, new_parent, new_cost):
        # type: (Node, float) -> None
        if self.parent:
            self.parent.childs.remove(self)

        self.parent = new_parent
        self.parent.childs.append(self)

        self.angle = self.parent.get_angle_from_child(self.pos)
        self.cost = new_cost
        self.update_childs_cost()
