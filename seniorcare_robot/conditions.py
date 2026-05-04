import py_trees
from rclpy.node import Node

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


class IsMode1Selected(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="IsMode1Selected")
        self.node = node

    def update(self):
        if self.node.tracking_mode == 1:
            print("Mode 1 selected: User Tracking")
            return py_trees.common.Status.SUCCESS
        if self.node.tracking_mode == 0:
            print("Mode 0 selected: Don't follow me")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE


class IsMode2Selected(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="IsMode2Selected")
        self.node = node

    def update(self):
        if self.node.tracking_mode == 2:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
