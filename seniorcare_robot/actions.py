import py_trees
from rclpy.node import Node

class TryTalk(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="TryTalk")
        self.node = node

    def initialise(self):
        self.node.get_logger().info("[BT] TryTalk initialise")

    def update(self):
        self.node.get_logger().info("[BT] TryTalk running")

        return py_trees.common.Status.RUNNING

#바로 llm으로 보내는거면 이동과 대화 동시에 이루어짐.
#llm에서 "따라오지마" 사용자 입력 받을 시 tracking mode=0 반대도 동일

class RunActionRecognition(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="RunActionRecognition")
        self.node = node

    def initialise(self):
        self.node.get_logger().info("[BT] RunActionRecognition initialise")

    def update(self):
        self.node.get_logger().info("[BT] RunActionRecognition running")
        pass
        return py_trees.common.Status.RUNNING


class MovingRobot(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="MovingRobot")
        self.node = node

    def initialise(self):
        self.node.get_logger().info("[BT] MovingRobot initialise")

    def update(self):
        if self.node.tracking_mode == 1:
            motor_command = [1.0, 1.0, 1.0]
            print("Running motors for tracking")
        else:
            motor_command = [0.0, 0.0, 0.0]
            print("Stopping motors")

        self.node.jointstate_publisher.publish(dxl(data=motor_command))
        return py_trees.common.Status.SUCCESS

import math
import py_trees
from std_msgs.msg import Float64MultiArray


class MoveToTarget(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__(name="MoveToTarget")
        self.node = node
        self.target_x = 0.0  # LiDAR 콜백에서 업데이트
        self.target_y = 0.0
        self.current_x = 0.0  # IMU/LiDAR에서 가져옴
        self.current_y = 0.0
        self.current_yaw = 0.0
        # PID 등 초기화 (필요 시 라이브러리 추가)

    def update(self):
        # 목표 도착 체크 (isArrive 비슷)
        if self.is_arrived(
            self.current_x, self.current_y, self.target_x, self.target_y
        ):
            # 정지 명령
            motor_command = [0.0, 0.0, 0.0]  # walkStop 비슷
            self.node.state_publisher_.publish(Float64MultiArray(data=motor_command))
            return py_trees.common.Status.SUCCESS

        # 이동 로직 (calcTargetAngle, PID 비슷)
        target_angle = self.calc_target_angle(
            self.target_x,
            self.target_y,
            self.current_x,
            self.current_y,
            self.current_yaw,
        )
        x_speed = self.calc_x_speed(
            self.current_x, self.current_y, self.target_x, self.target_y
        )
        yaw_speed = self.calc_yaw_speed(target_angle)  # PID 적용

        # walkStart 비슷: 모터 명령 퍼블리시
        motor_command = [x_speed, 0.0, yaw_speed]
        self.node.state_publisher_.publish(Float64MultiArray(data=motor_command))
        return py_trees.common.Status.RUNNING  # 아직 이동 중

    def is_arrived(self, cx, cy, tx, ty):
        distance = math.sqrt((tx - cx) ** 2 + (ty - cy) ** 2)
        return distance < 0.1  # 도착 임계값

    def calc_target_angle(self, tx, ty, cx, cy, yaw):
        # 목표 각도 계산 (C++ calcTargetAngle 비슷)
        angle = math.atan2(ty - cy, tx - cx) - yaw
        return math.degrees(angle)  # 라디안 to 도

    def calc_x_speed(self, cx, cy, tx, ty):
        # X 속도 계산 (C++ x 계산 비슷)
        distance = math.sqrt((tx - cx) ** 2 + (ty - cy) ** 2)
        speed = min(max(distance * 50, 10), 100)  # 예시 범위
        return speed

    def calc_yaw_speed(self, target_angle):
        # Yaw 속도 (PID 적용, C++ yawControl 비슷)
        # 간단 PID 구현 또는 라이브러리 사용
        error = target_angle
        yaw = max(min(error * 0.5, 30), -30)  # 예시
        return yaw
