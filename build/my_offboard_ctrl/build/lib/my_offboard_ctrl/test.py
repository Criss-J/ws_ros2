#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus
)
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import json
from collections import deque


class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_mission')

        # QoS 분리: PX4로 보낼 때(Pub) vs PX4에서 받을 때(Sub)
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers (ROS2 → PX4)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_pub)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_pub)

        # Subscribers (PX4 → ROS2)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_sub)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_sub)

        # 외부 명령 / 카메라
        self.create_subscription(String, '/drone_command', self.command_callback, 10)
        self.cv_bridge = CvBridge()
        self.create_subscription(Image, '/camera', self.image_callback, 10)

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.position_received = False

        # 미션 큐
        self.mission_queue = deque()
        self.mission_state = "STANDBY"
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0

        self.ARRIVAL_THRESHOLD = 0.3
        self.arrival_count = 0  # 도착 유지 카운터 (10 = 1초)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('🦾 행동 대장(큐 모드) 켜짐!')

    # ──────────────────────────────────────────────
    # 명령 수신 & 미션 큐 관리
    # ──────────────────────────────────────────────

    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            plan = data.get("plan", [])
            if not plan:
                self.get_logger().warning("빈 플랜 수신")
                return

            if not self.position_received:
                self.get_logger().error(
                    "❌ PX4 위치 데이터 미수신! XRCE-DDS Agent 연결 확인 필요. 명령 거부.")
                return

            self.mission_queue.clear()
            self.arrival_count = 0
            for step in plan:
                self.mission_queue.append(step)

            self.get_logger().info(f'📋 플랜 수신: {len(plan)}단계')
            for i, step in enumerate(plan):
                self.get_logger().info(f'  {i+1}. {step}')

            self._start_next_mission()

        except json.JSONDecodeError:
            self.get_logger().error("JSON 파싱 오류")

    def _start_next_mission(self):
        if not self.mission_queue:
            self.get_logger().info('✅ 모든 임무 완료! HOVER 대기 중...')
            self.mission_state = "HOVER"
            return

        cmd = self.mission_queue.popleft()
        action = cmd.get("action")

        current_x = self.vehicle_local_position.x
        current_y = self.vehicle_local_position.y
        current_z = self.vehicle_local_position.z
        current_yaw = (self.vehicle_local_position.heading
                       if hasattr(self.vehicle_local_position, 'heading') else 0.0)

        # 🔥 디버그: 미션 시작 시점의 모든 상태
        self.get_logger().info(
            f'🔍 [DEBUG] action={action}, '
            f'current X:{current_x:.2f} Y:{current_y:.2f} Z:{current_z:.2f} '
            f'Yaw:{math.degrees(current_yaw):.1f}° | '
            f'prev target Z:{self.target_z:.2f}, state={self.mission_state}')

        self.arrival_count = 0  # 새 임무마다 초기화

        if action == "takeoff":
            alt = cmd.get("alt", 2.0)
            self.target_x = current_x
            self.target_y = current_y
            self.target_z = -float(alt)  # NED: 위가 음수
            self.target_yaw = current_yaw
            self.mission_state = "EXECUTING"
            self.engage_offboard_mode()
            self.arm()
            self.get_logger().info(
                f'🚀 이륙 → 목표 고도: {alt}m (target_z={self.target_z:.2f})')

        elif action == "land":
            self.mission_state = "LANDING"
            self.land()
            self.get_logger().info('🛬 착륙')

        elif action == "move":
            dx_body  = cmd.get("dx", 0.0)
            dy_body  = cmd.get("dy", 0.0)
            dz_up    = cmd.get("dz", 0.0)
            dyaw_deg = cmd.get("d_yaw", 0.0)

            # 🔥 핵심 픽스: 비행 중이면 이전 target_z 유지 (current_z 흔들림 방어)
            # mission_state가 STANDBY가 아니고 이전 target이 의미있는 고도면 그걸 base로
            if self.mission_state != "STANDBY" and abs(self.target_z) > 0.5:
                base_z = self.target_z  # 이전 목표 고도 유지
            else:
                base_z = current_z

            # body → world 변환 (NED 좌표계)
            delta_x = dx_body * math.cos(current_yaw) - dy_body * math.sin(current_yaw)
            delta_y = dx_body * math.sin(current_yaw) + dy_body * math.cos(current_yaw)

            self.target_x = current_x + delta_x
            self.target_y = current_y + delta_y
            self.target_z = base_z - dz_up   # base_z 사용 (NED: 위가 음수)
            self.target_yaw = current_yaw + math.radians(dyaw_deg)

            self.mission_state = "EXECUTING"
            self.get_logger().info(
                f'📍 이동 → target X:{self.target_x:.2f} Y:{self.target_y:.2f} '
                f'Z:{self.target_z:.2f} Yaw:{math.degrees(self.target_yaw):.1f}° '
                f'(base_z={base_z:.2f}, dz_up={dz_up})')

        elif action == "goto":
            self.target_x = cmd.get("x", current_x)
            self.target_y = cmd.get("y", current_y)
            self.target_z = -float(cmd.get("z", abs(current_z)))
            self.target_yaw = math.radians(cmd.get("yaw", math.degrees(current_yaw)))
            self.mission_state = "EXECUTING"
            self.get_logger().info(
                f'🎯 절대 이동 → X:{self.target_x:.2f} '
                f'Y:{self.target_y:.2f} Z:{self.target_z:.2f}')

        else:
            self.get_logger().warning(f'⚠️ 알 수 없는 action: {action}')
            self._start_next_mission()  # 다음으로 넘어감

    # ──────────────────────────────────────────────
    # 메인 타이머 루프 (10Hz)
    # ──────────────────────────────────────────────

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        self.publish_position_setpoint(
            self.target_x, self.target_y, self.target_z, self.target_yaw)

        if self.mission_state == "EXECUTING":
            dist = math.sqrt(
                (self.vehicle_local_position.x - self.target_x) ** 2 +
                (self.vehicle_local_position.y - self.target_y) ** 2 +
                (self.vehicle_local_position.z - self.target_z) ** 2
            )
            if dist < self.ARRIVAL_THRESHOLD:
                self.arrival_count += 1
                if self.arrival_count >= 10:  # 1초 유지 시 도달 판정
                    self.arrival_count = 0
                    self.get_logger().info(
                        f'✔ 목표 도달! (현재 X:{self.vehicle_local_position.x:.2f}, '
                        f'Y:{self.vehicle_local_position.y:.2f}, '
                        f'Z:{self.vehicle_local_position.z:.2f})')
                    self._start_next_mission()
            else:
                self.arrival_count = 0

    # ──────────────────────────────────────────────
    # PX4 콜백
    # ──────────────────────────────────────────────

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        if not self.position_received:
            self.position_received = True
            self.get_logger().info(
                f'📡 PX4 위치 수신 시작! X:{msg.x:.2f}, Y:{msg.y:.2f}, Z:{msg.z:.2f}')

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    # ──────────────────────────────────────────────
    # PX4 명령 헬퍼
    # ──────────────────────────────────────────────

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    # ──────────────────────────────────────────────
    # 카메라 화면
    # ──────────────────────────────────────────────

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pos_str = (f"Pos:({self.vehicle_local_position.x:.1f},"
                       f"{self.vehicle_local_position.y:.1f},"
                       f"{self.vehicle_local_position.z:.1f})")
            cv2.putText(cv_image,
                        f"State: {self.mission_state} | Queue: {len(self.mission_queue)} | "
                        f"Arrive: {self.arrival_count}/10",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(cv_image, pos_str,
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow('Drone Camera View', cv_image)
            cv2.waitKey(1)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()