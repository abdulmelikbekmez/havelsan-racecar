from enum import Enum
import numpy as np
from typing import Union, List
from vector import Vector


class DirState(Enum):
    FORWARD = 1
    BACKWARD = -1


class Node:

    def __init__(self, pos, angle, dir=DirState.FORWARD, parent=None):
        # type: (Vector, float, DirState, Union[Node, None]) -> None
        self.pos = pos
        self.dir = dir
        self.angle = angle
        self.parent = parent
        self.childs = []  # type: List[Node]

    def __get_angle_from_child(self, child):
        # type: (Vector) -> float
        dif = child - self.pos
        return np.rad2deg(np.arctan2(dif.y, dif.x))  # type: ignore

    def add_child(self, pos, dir_changed):
        # type: (Vector, bool) -> Node
        angle = self.__get_angle_from_child(pos)
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
        child = Node(pos, angle, dir, self)
        self.childs.append(child)
        return child


class Connection:

    def __init__(self, _from, length):
        # type: (Node, float) -> None
        self._from = _from
        self.length = length
