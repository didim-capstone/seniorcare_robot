import py_trees
from rclpy.node import Node
from enum import IntEnum

class RobotMode(IntEnum):
    INIT = 0
    FOLLOWING_MODE = 1
    PATROL_MODE = 2


class IsTalkRequested(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="IsTalkRequested")
        self.node = node

    def update(self):
        if self.node.talk_requested:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsPoseRecogRequested(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="IsPoseRecogRequested")
        self.node = node

    def update(self):
        if self.node.pose_recog_requested:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class FollowingMode(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node, mode: int):
        super().__init__(name="FollowingMode")
        self.node = node
        self.mode = mode

    def update(self):
        if self.mode == RobotMode.FOLLOWING_MODE:#visible and nearby
            print("Mode 1 selected: User Tracking")
            return py_trees.common.Status.SUCCESS #next step(=leaf node)
        elif self.mode == RobotMode.PATROL_MODE:  # not visible and not talk requested or dontfollow
            print("Mode 1 selected: Don't follow me") #go to patrol mode
        return py_trees.common.Status.FAILURE #go to first


class PatrolMode(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="PatrolMode")
        self.node = node

    def update(self):
        if self.node.is_talk_requested is True:
            print("Voice recog, go to person pos")
            return py_trees.common.Status.FAILURE  # Patrol 중단, 다음 branch로
        return py_trees.common.Status.SUCCESS  # Patrol 계속
