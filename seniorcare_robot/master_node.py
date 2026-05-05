import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from std_msgs.msg import Float64MultiArray
from math import pi

import py_trees

import argparse
import sys
from enum import IntEnum

from senior_msg.msg import ImuMsg, Vison2Master, Master2llm, Master2Base

from senior_care_robot.base_commander import BaseCommander

from seniorcare_robot.conditions import (
    IsTalkRequested,
    IsPoseRecogRequested,
    FollowingMode,
    PatrolMode,
    IsVoiceCallRequested,
    RobotMode,
)

from seniorcare_robot.actions import (
    TryTalk,
    TrackingPerson,
    MoveToHomeTarget,
    MoveToLidarPersonPosition,
)


class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        qos_realiable = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                   depth=10,
                                   reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                   durability=QoSDurabilityPolicy.VOLATILE)

        self.state_publisher_ = self.create_publisher(Float64MultiArray, "state", qos_profile)
        self.motor_pub = self.create_publisher(
            Master2Base, "/base_command", qos_profile
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            ImuMsg, 'Imu', self.imu_callback, qos_realiable)
        self.vision_sub = self.create_subscription(
            Vison2Master, "vision2master", self.vision_callback, qos_realiable
        )
        self.llm_sub = self.create_subscription(
            Master2llm, "llm2master", self.llm_callback, qos_realiable
        )

        self.mode = 0
        self.tracking_mode = 1
        self.pose_recog_requested = False
        self.talk_requested = False
        self.commander = BaseCommander(self.motor_pub)

        self.create_timer(0.01, self.tick_tree)
        self.tree = self.create_behavior_tree()

    def slam_callback(self, msg):
        self.person_x = msg.x
        self.person_y = msg.y
        self.person_dist = msg.dist
        self.home_x = msg.home_x
        self.home_y = msg.home_y
        self.robot_x = msg.robot_x
        self.robot_y = msg.robot_y

    def imu_callback(self, msg):
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw

    def vision_callback(self, msg):
        self.tracking_mode = msg.mode

    def llm_callback(self, msg):
        self.is_talk_requested = msg.mode
        self.dont_follow = msg.mode

    def tick_tree(self):
        self.tree.tick()

        self.get_logger().info(
            f"[STATE] mode={self.tracking_mode}, "
            f"talk={self.is_talk_requested}, "
            f"talk_2={self.dont_follow},"
            f"action={self.pose_recog_requested}, "
        )

        # self.jointstate_publisher.publish(dxl)

    def create_behavior_tree(self):
        # mode setting(transition)
        if self.dont_follow is True:
            if self.detect_fall is True:
                self.mode = RobotMode.EMERGENCY_MODE
            else:
                self.mode = RobotMode.PATROL_MODE
        elif ((self.is_talk_requested is True) or
                (self.dont_follow is False and self.person_dist < 30)):#self.person_x != 999 or
                # 사람이 가까이 위치하고 카메라에 보일 때(근데 가까이 위치했는데 카메라에 안 보이는 경우?->or)
            self.mode = RobotMode.FOLLOWING_MODE
        else:#stop, init
            pass


        # Event 2: Talk
        talk_branch = py_trees.composites.Sequence(name="TALK_BRANCH", memory=False)
        talk_branch.add_children(
            [
                IsTalkRequested(self),
                TryTalk(self),
            ]
        )
        # Mode 1: User Tracking
        tracking_branch = py_trees.composites.Sequence(
            name="TRACKING_BRANCH", memory=False
        )
        tracking_branch.add_children(
            [
                FollowingMode(self, self.mode),
                TrackingPerson(self, self.commander),
            ]
        )

        patrol_branch = py_trees.composites.Sequence(
            name="PATROL_BRANCH", memory=False
        )
        patrol_branch.add_children(
            [
                PatrolMode(self),
                MoveToHomeTarget(self, self.commander),
            ]
        )

        root = py_trees.composites.Selector(name="ROOT", memory=False)
        root.add_children(
            [
                talk_branch,
                tracking_branch,
                patrol_branch,
            ]
        )

        tree = py_trees.trees.BehaviourTree(root=root)
        tree.setup(timeout=15)
        return tree


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n killed with ctrl-c ")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    if not main():
        sys.exit(1)
