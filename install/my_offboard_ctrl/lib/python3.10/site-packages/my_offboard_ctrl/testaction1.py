#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import json

class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.cv_bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, '/camera', self.image_callback, 10)
        
        # AI 명령 수신 (JSON)
        self.command_subscriber = self.create_subscription(String, '/drone_command', self.command_callback, 10)

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        
        self.mission_state = "STANDBY"
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 0.0
        self.target_yaw = 0.0
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    # 🔥 AI의 JSON 명령을 해석하고 동적/절대 좌표를 계산하는 로직
    def command_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            action = cmd.get("action")
            
            # 현재 드론 상태 안전하게 가져오기
            current_yaw = self.vehicle_local_position.heading if hasattr(self.vehicle_local_position, 'heading') else 0.0
            current_x = self.vehicle_local_position.x
            current_y = self.vehicle_local_position.y
            current_z = self.vehicle_local_position.z

            if action == "takeoff":
                # JSON에서 alt 값을 꺼냄 (명시 안 되어 있으면 기본값 2.0m)
                target_alt = cmd.get("alt", 2.0) 
                self.get_logger().info(f'🚀 이륙 시작 (목표 고도: {target_alt}m)')
                
                # PX4 NED 좌표계: 위로 올라갈수록 Z값이 음수(-)
                self.target_z = -float(target_alt) 
                
                self.mission_state = "HOVER"
                self.engage_offboard_mode()
                self.arm()
                
            elif action == "land":
                self.get_logger().info('🛬 착륙 시작')
                self.mission_state = "LANDING"
                self.land()
                
            elif action == "move": # 상대 좌표 이동 (기수 방향 기준)
                dx_body = cmd.get("dx", 0.0)
                dy_body = cmd.get("dy", 0.0)
                dz_up = cmd.get("dz", 0.0)
                dyaw_deg = cmd.get("d_yaw", 0.0)

                # 현재 방향 기준으로 회전 행렬(삼각함수) 적용
                delta_x = dx_body * math.cos(current_yaw) - dy_body * math.sin(current_yaw)
                delta_y = dx_body * math.sin(current_yaw) + dy_body * math.cos(current_yaw)
                
                self.target_x = current_x + delta_x
                self.target_y = current_y + delta_y
                self.target_z = current_z - dz_up 
                self.target_yaw = current_yaw + math.radians(dyaw_deg)

                self.mission_state = "DYNAMIC_MOVE"
                self.get_logger().info(f'📍 동적 이동 갱신: X({self.target_x:.2f}), Y({self.target_y:.2f}), Z({self.target_z:.2f}), Yaw({math.degrees(self.target_yaw):.1f}도)')
                
                self.engage_offboard_mode()
                self.arm()

            elif action == "goto": # 절대 좌표 이동 (글로벌 원점 기준)
                self.target_x = cmd.get("x", current_x)
                self.target_y = cmd.get("y", current_y)
                
                # LLM이 양수 고도(예: 3m)를 주면 NED Z축(-3m)으로 변환
                if "z" in cmd:
                    self.target_z = -float(cmd["z"])
                else:
                    self.target_z = current_z
                    
                if "yaw" in cmd:
                    self.target_yaw = math.radians(float(cmd["yaw"]))
                else:
                    self.target_yaw = current_yaw

                self.mission_state = "DYNAMIC_MOVE"
                self.get_logger().info(f'🎯 절대 좌표 이동 갱신: X({self.target_x:.2f}), Y({self.target_y:.2f}), Z({self.target_z:.2f})')
                
                self.engage_offboard_mode()
                self.arm()
                
        except json.JSONDecodeError:
            self.get_logger().warning("잘못된 JSON 명령 형식입니다.")

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        
        # 오차 범위 도달 체크
        if self.mission_state == "DYNAMIC_MOVE":
            dist = math.sqrt((self.vehicle_local_position.x - self.target_x)**2 + (self.vehicle_local_position.y - self.target_y)**2 + (self.vehicle_local_position.z - self.target_z)**2)
            if dist < 0.3:
                self.mission_state = "HOVER"
                
        self.publish_position_setpoint(self.target_x, self.target_y, self.target_z, self.target_yaw)

    def vehicle_local_position_callback(self, msg): self.vehicle_local_position = msg
    def vehicle_status_callback(self, msg): self.vehicle_status = msg
    
    # 💡 여기서 400, 21 등의 명령어가 MAVLink 규격에 맞게 상수 이름으로 사용됨
    def arm(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    def engage_offboard_mode(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    def land(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

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
            cv2.putText(cv_image, f"State: {self.mission_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Drone Camera View', cv_image)
            cv2.waitKey(1)
        except Exception: pass

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    print('🦾 행동 대장 켜짐: 대뇌의 명령을 기다리는 중...')
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown(); cv2.destroyAllWindows()

if __name__ == '__main__':
    main()