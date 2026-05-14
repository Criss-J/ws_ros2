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

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_pub)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_pub)

        # Subscribers
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_sub)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_sub)
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
        self.arrival_count = 0

        # 역순 실행용
        self.last_executed_plan = []

        # 🆕 위치 history: 각 명령 시작 시점의 (x, y, z, yaw)
        # history[0]은 처음 명령 시작 시 위치 (= 시작점, P0)
        # history[-1]은 현재 명령 시작 시 위치 (= 현재 위치)
        # history[-2]는 직전 명령 시작 시 위치 (= "이전 위치")
        self.position_history = []

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('🦾 행동 대장(역순 실행 + 위치 history) 켜짐!')

    # ──────────────────────────────────────────────
    # 역순 plan 변환
    # ──────────────────────────────────────────────

    def reverse_plan(self, plan):
        if not plan:
            return []
        reversed_steps = []
        for step in reversed(plan):
            action = step.get("action")
            if action == "move":
                reversed_steps.append({
                    "action": "move",
                    "dx": -float(step.get("dx", 0.0)),
                    "dy": -float(step.get("dy", 0.0)),
                    "dz": -float(step.get("dz", 0.0)),
                    "d_yaw": -float(step.get("d_yaw", 0.0)),
                })
            elif action == "takeoff":
                reversed_steps.append({"action": "land"})
            elif action == "land":
                alt = 2.0
                for s in plan:
                    if s.get("action") == "takeoff":
                        alt = float(s.get("alt", 2.0))
                        break
                reversed_steps.append({"action": "takeoff", "alt": alt})
            elif action == "goto":
                self.get_logger().warning("⚠️ goto 역순 변환은 스킵.")
                continue
            else:
                continue
        return reversed_steps

    # ──────────────────────────────────────────────
    # 🆕 위치 history 관리
    # ──────────────────────────────────────────────

    def _record_current_position(self, label=""):
        """현재 위치를 history에 추가"""
        current_yaw = (self.vehicle_local_position.heading
                       if hasattr(self.vehicle_local_position, 'heading') else 0.0)
        pos = (
            float(self.vehicle_local_position.x),
            float(self.vehicle_local_position.y),
            float(self.vehicle_local_position.z),
            float(current_yaw),
        )
        self.position_history.append(pos)
        self.get_logger().info(
            f'📌 위치 기록 [{len(self.position_history)-1}]{label}: '
            f'X:{pos[0]:.2f}, Y:{pos[1]:.2f}, Z:{pos[2]:.2f}, '
            f'Yaw:{math.degrees(pos[3]):.1f}°')

    # ──────────────────────────────────────────────
    # 명령 수신 & 미션 큐 관리
    # ──────────────────────────────────────────────

    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)

            if not self.position_received:
                self.get_logger().error("❌ PX4 위치 데이터 미수신! 명령 거부.")
                return

            # 🆕 새 명령 받기 직전 현재 위치를 history에 기록
            self._record_current_position(label="(새 명령 시작 시점)")

            # 역순 실행 플래그 처리
            if data.get("reverse"):
                if not self.last_executed_plan:
                    self.get_logger().error("❌ 역순으로 실행할 이전 plan 없음")
                    return
                plan = self.reverse_plan(self.last_executed_plan)
                if not plan:
                    self.get_logger().error("❌ 역순 변환 결과가 비어있음")
                    return
                self.get_logger().info(f'🔄 역순 실행 모드 ({len(plan)}단계)')
            else:
                plan = data.get("plan", [])

            if not plan:
                self.get_logger().warning("빈 플랜 수신")
                return

            self.mission_queue.clear()
            self.arrival_count = 0
            for step in plan:
                self.mission_queue.append(step)

            # 역순이 아닐 때만 last_executed_plan 갱신
            if not data.get("reverse"):
                self.last_executed_plan = list(plan)

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

        self.get_logger().info(
            f'🔍 [DEBUG] action={action}, '
            f'current X:{current_x:.2f} Y:{current_y:.2f} Z:{current_z:.2f} '
            f'Yaw:{math.degrees(current_yaw):.1f}° | '
            f'prev target Z:{self.target_z:.2f}, state={self.mission_state}')

        self.arrival_count = 0

        if action == "takeoff":
            alt = cmd.get("alt", 2.0)
            self.target_x = current_x
            self.target_y = current_y
            self.target_z = -float(alt)
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

            if self.mission_state != "STANDBY" and abs(self.target_z) > 0.5:
                base_z = self.target_z
            else:
                base_z = current_z

            delta_x = dx_body * math.cos(current_yaw) - dy_body * math.sin(current_yaw)
            delta_y = dx_body * math.sin(current_yaw) + dy_body * math.cos(current_yaw)

            self.target_x = current_x + delta_x
            self.target_y = current_y + delta_y
            self.target_z = base_z - dz_up
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

        # 🆕 위치 history 기반 이동
        elif action == "goto_history":
            index = cmd.get("index", -2)  # 기본: 직전 위치
            # 음수 인덱스 처리 및 범위 체크
            try:
                target_pos = self.position_history[index]
            except (IndexError, TypeError):
                self.get_logger().error(
                    f'❌ history index {index} 범위 초과! '
                    f'history 크기: {len(self.position_history)}. 미션 스킵.')
                self._start_next_mission()
                return

            tx, ty, tz, t_yaw = target_pos
            self.target_x = tx
            self.target_y = ty
            self.target_z = tz
            self.target_yaw = t_yaw
            self.mission_state = "EXECUTING"
            
            # 친절한 라벨
            if index == 0:
                label = "처음 위치"
            elif index == -2 or index == len(self.position_history) - 2:
                label = "이전 위치"
            else:
                label = f"history[{index}]"
            
            self.get_logger().info(
                f'⏪ {label}로 복귀 → X:{tx:.2f} Y:{ty:.2f} Z:{tz:.2f} '
                f'Yaw:{math.degrees(t_yaw):.1f}°')

        else:
            self.get_logger().warning(f'⚠️ 알 수 없는 action: {action}')
            self._start_next_mission()

    # ──────────────────────────────────────────────
    # 메인 타이머 루프
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
                if self.arrival_count >= 10:
                    self.arrival_count = 0
                    self.get_logger().info(
                        f'✔ 목표 도달! (현재 X:{self.vehicle_local_position.x:.2f}, '
                        f'Y:{self.vehicle_local_position.y:.2f}, '
                        f'Z:{self.vehicle_local_position.z:.2f})')
                    self._start_next_mission()
            else:
                self.arrival_count = 0

    # ──────────────────────────────────────────────
    # PX4 콜백 / 헬퍼
    # ──────────────────────────────────────────────

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        if not self.position_received:
            self.position_received = True
            self.get_logger().info(
                f'📡 PX4 위치 수신 시작! X:{msg.x:.2f}, Y:{msg.y:.2f}, Z:{msg.z:.2f}')

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

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

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pos_str = (f"Pos:({self.vehicle_local_position.x:.1f},"
                       f"{self.vehicle_local_position.y:.1f},"
                       f"{self.vehicle_local_position.z:.1f})")
            cv2.putText(cv_image,
                        f"State: {self.mission_state} | Queue: {len(self.mission_queue)} | "
                        f"History: {len(self.position_history)}",
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