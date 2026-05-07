import py_trees
from rclpy.node import Node

from senior_msg.msg import Master2Base
from seniorcare_robot.base_commander import BaseCommander


import math
from std_msgs.msg import Float64MultiArray



class TryTalk(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="TryTalk")
        self.node = node

    def initialise(self):
        self.node.get_logger().info("[BT] TryTalk initialise")

    def update(self):
        self.node.get_logger().info("[BT] TryTalk running")

        return py_trees.common.Status.RUNNING


# 바로 llm으로 보내는거면 이동과 대화 동시에 이루어짐.
# llm에서 "따라오지마" 사용자 입력 받을 시 tracking mode=0 반대도 동일



class TrackingPerson(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node, commander: BaseCommander):
        super().__init__(name="TrackingPerson")
        self.node = node
        self.commander = commander
        self.msg = Master2Base()
        self.motor_command = [0.0, 0.0, 0.0]

        self.target_x = 0.0  # LiDAR 콜백에서 업데이트
        self.target_y = 0.0
        self.current_x = 0.0  # IMU/LiDAR에서 가져옴
        self.current_y = 0.0
        self.current_yaw = 0.0
        # PID 등 초기화 (필요 시 라이브러리 추가)

    def initialise(self):
        self.node.get_logger().info("initialise")
        self.commander.reset_encoders()

    def update(self):
        # 목표 도착 체크 (isArrive 비슷)
        if self.node.person_visible:#yolo_vision
            if self.is_arrived(
                self.current_x, self.current_y, self.target_x, self.target_y
            ):
                # 정지 명령
                motor_command = [0.0, 0.0, 0.0]  # walkStop 비슷
                self.node.motor_pub.publish(Master2Base(data=motor_command))
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
        else:
            self.search_for_person()
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

    def search_for_person(self):
        motor_command = [0.0, 0.0, 20.0]
        self.node.state_publisher_.publish(Float64MultiArray(data=motor_command))


class MoveToHomeTarget(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node, commander: BaseCommander):
        super().__init__(name="MoveToHomeTarget")
        self.node = node
        self.commander = commander

    def initialise(self):
        self.node.get_logger().info("[BT] MoveToHomeTarget initialise")

    def update(self):
        self.node.get_logger().info("[BT] MoveToHomeTarget running")
        pass
        return py_trees.common.Status.RUNNING
