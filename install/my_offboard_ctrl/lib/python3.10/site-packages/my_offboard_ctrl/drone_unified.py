#!/usr/bin/env python3
"""
drone_unified.py - LLM + 드론 제어 통합 + 위치 history (데코레이터) + 비홀로노믹(회전 후 전진) + 최단거리 정규화 완벽 적용
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus
)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import json
import threading
import functools
from collections import deque

from langchain_google_vertexai import ChatVertexAI
import vertexai


SYSTEM_PROMPT = """당신은 드론의 좌표 계산기입니다.
사용자의 자연어 명령을 받아서 PX4 NED 좌표계의 setpoint 시퀀스로 변환하세요.

[PX4 NED 좌표계]
- X: 북쪽 양수, Y: 동쪽 양수, Z: 아래쪽 양수 (고도 5m = z=-5)
- yaw: 라디안. 북쪽이 0, 시계방향 양수

[기수 기준 상대 이동 변환]
"앞으로 N미터":
  new_x = current_x + N * cos(current_yaw)
  new_y = current_y + N * sin(current_yaw)
"뒤로": cos/sin에 -N
"오른쪽 N미터": current_x - N*sin(yaw), current_y + N*cos(yaw)
"왼쪽 N미터": current_x + N*sin(yaw), current_y - N*cos(yaw)

[🆕 시계 방향(Clock-face) 이동 변환 - 최단거리 정규화 적용]
"M시 방향으로 N미터":
  1. M시에 따른 상대 회전각(offset_rad) 매핑:
     - 12시: 0.0
     - 1시: +0.52, 2시: +1.05, 3시: +1.57
     - 4시: +2.09, 5시: +2.62, 6시: +3.14
     - 7시: -2.62, 8시: -2.09, 9시: -1.57
     - 10시: -1.05, 11시: -0.52
  2. target_yaw = current_yaw + offset_rad
  3. [중요] target_yaw 정규화: 값이 3.14159(pi) 보다 크면 6.28318(2*pi) 을 빼고, -3.14159 보다 작으면 6.28318 을 더하세요. (반드시 -pi ~ pi 범위 유지)
  4. new_x = current_x + N * cos(target_yaw)
  5. new_y = current_y + N * sin(target_yaw)
* 주의사항: 기수가 먼저 이동 방향을 보도록 2개의 시퀀스로 분리.
  - 시퀀스 1 (제자리 회전): 현재 x, y, z 유지, yaw만 target_yaw
  - 시퀀스 2 (전진 이동): new_x, new_y, 현재 z, target_yaw
* 만약 사용자가 "N미터"라는 거리를 생략하고 "M시 방향으로 가"라고만 명령하면, 기본값 N=2.0 미터로 계산.

[회전]
"오른쪽 N도": new_yaw = current_yaw + radians(N)
"왼쪽 N도": new_yaw = current_yaw - radians(N)

[고도]
"N미터 이륙" → 새 z = -N
"N미터 올라가" → 새 z = current_z - N
"N미터 내려가" → 새 z = current_z + N
"착륙" → land=true

[위치 복귀 명령 - 특별 키워드 사용]
사용자가 "이전 위치로", "처음 위치로", "원점으로", "출발 지점으로" 같이 
과거 위치로 돌아가라는 요청을 하면, 좌표를 직접 계산하지 말고 
다음 특별 형식을 사용하세요:

- "이전 위치", "방금 전 위치", "한 단계 전 위치" → {"recall": "previous"}
- "처음 위치", "시작 위치", "출발 지점", "원점" → {"recall": "first"}

[출력 형식 - JSON만]
{"setpoints": [
  {"x":..., "y":..., "z":..., "yaw":..., "arm":true/false, "offboard":true/false, "land":true/false},
  또는
  {"recall": "previous" 또는 "first"}
]}

[규칙]
- 첫 이륙엔 arm=true, offboard=true 포함
- yaw는 라디안 (도 X)
- 좌표는 절대 좌표
- recall 키워드는 단독 setpoint로 사용 (다른 필드와 섞지 말 것)
- 마크다운, 설명 없이 JSON만

[예시 1] "5m 이륙해" (현재 x=0,y=0,z=0,yaw=0)
{"setpoints":[{"x":0.0,"y":0.0,"z":-5.0,"yaw":0.0,"arm":true,"offboard":true}]}

[예시 2] "앞으로 3m" (현재 x=0,y=0,z=-5,yaw=0)
{"setpoints":[{"x":3.0,"y":0.0,"z":-5.0,"yaw":0.0}]}

[예시 3] "이전 위치로 돌아가"
{"setpoints":[{"recall":"previous"}]}

[예시 4] "6시 방향으로 3m 이동해" (현재 x=0, y=0, z=-5, yaw=0)
{"setpoints":[
  {"x":0.0,"y":0.0,"z":-5.0,"yaw":3.14159},
  {"x":-3.0,"y":0.0,"z":-5.0,"yaw":3.14159}
]}

[예시 5] "9시 방향으로 가" (거리 생략 시 기본 2m, 현재 x=0, y=0, z=-5, yaw=0)
{"setpoints":[
  {"x":0.0,"y":0.0,"z":-5.0,"yaw":-1.5708},
  {"x":0.0,"y":-2.0,"z":-5.0,"yaw":-1.5708}
]}
"""


# ──────────────────────────────────────────────
# 데코레이터: 명령 처리 전후로 위치 history 자동 관리
# ──────────────────────────────────────────────

def track_position(func):
    """명령 처리 메서드를 감싸서:
    1. 명령 실행 전 현재 위치를 history에 자동 기록
    2. LLM 결과의 recall 키워드를 실제 좌표로 자동 치환
    """
    @functools.wraps(func)
    def wrapper(self, user_input):
        # [Before] 명령 실행 직전 현재 위치 기록
        if self.position_received:
            cur_yaw = (self.local_position.heading 
                       if hasattr(self.local_position, 'heading') else 0.0)
            pos = (
                float(self.local_position.x),
                float(self.local_position.y),
                float(self.local_position.z),
                float(cur_yaw),
            )
            self.position_history.append(pos)
            self.get_logger().info(
                f'📌 history[{len(self.position_history)-1}] 기록: '
                f'X:{pos[0]:.2f}, Y:{pos[1]:.2f}, Z:{pos[2]:.2f}, '
                f'Yaw:{math.degrees(pos[3]):.1f}°')
        
        # [Body] 원래 함수 실행 (LLM 호출 등)
        return func(self, user_input)
    return wrapper


def resolve_recall(func):
    """LLM이 만든 setpoint 리스트에 'recall' 키워드 있으면
    실제 좌표(history에서)로 치환하는 데코레이터."""
    @functools.wraps(func)
    def wrapper(self, setpoints):
        resolved = []
        for sp in setpoints:
            if "recall" not in sp:
                resolved.append(sp)
                continue
            
            kind = sp["recall"]
            target_pos = None
            label = ""

            if kind == "first":
                # 처음 위치 = history[0]
                if len(self.position_history) >= 1:
                    target_pos = self.position_history[0]
                    label = "처음 위치"
            elif kind == "previous":
                # 이전 위치 = 현재 명령 직전이 history[-1]이므로
                # 그 직전인 history[-2]가 "이전"
                if len(self.position_history) >= 2:
                    target_pos = self.position_history[-2]
                    label = "이전 위치"
            
            if target_pos is None:
                self.get_logger().warning(
                    f'⚠️ recall="{kind}" 실패: history 부족 '
                    f'({len(self.position_history)}개). 스킵.')
                continue
            
            tx, ty, tz, t_yaw = target_pos
            resolved.append({
                "x": tx, "y": ty, "z": tz, "yaw": t_yaw,
                "_label": label,  # 로그용
            })
            self.get_logger().info(
                f'⏪ recall="{kind}" → {label} 좌표로 치환: '
                f'X:{tx:.2f}, Y:{ty:.2f}, Z:{tz:.2f}')
        
        return func(self, resolved)
    return wrapper


# ──────────────────────────────────────────────
# 메인 노드
# ──────────────────────────────────────────────

class DroneUnified(Node):
    def __init__(self):
        super().__init__('drone_unified')
        self.callback_group = ReentrantCallbackGroup()

        # ── QoS ──
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        # ── Publishers ──
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_pub)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_pub)

        # ── Subscribers ──
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.position_callback, qos_sub, callback_group=self.callback_group)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.status_callback, qos_sub, callback_group=self.callback_group)
        self.cv_bridge = CvBridge()
        self.create_subscription(Image, '/camera', self.image_callback, 10,
                                 callback_group=self.callback_group)

        # ── 상태 ──
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.position_received = False

        self.setpoint_queue = deque()
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0
        self.has_target = False
        self.in_landing = False

        self.ARRIVAL_THRESHOLD = 0.3
        self.arrival_count = 0

        # 위치 history
        self.position_history = []

        # ── LLM ──
        vertexai.init(project="geonhui-494205", location="us-central1")
        self.llm = ChatVertexAI(model_name="gemini-2.5-flash", temperature=0.0)

        # ── 타이머 ──
        self.timer = self.create_timer(0.1, self.timer_callback,
                                        callback_group=self.callback_group)

        self.get_logger().info('🦾🧠 통합 드론 노드 시작')

        threading.Thread(target=self.user_input_loop, daemon=True).start()

    # ──────────────────────────────────────────────
    # 사용자 입력 루프
    # ──────────────────────────────────────────────

    def user_input_loop(self):
        print("\n🗣️ 명령 (exit으로 종료, '멈춰'로 정지)")
        print("   - '5m 이륙해', '앞으로 3m', '9시 방향으로 5m'")
        print("   - '이전 위치로', '처음 위치로'\n")
        
        while rclpy.ok():
            try:
                user_input = input("\n🗣️ 명령: ").strip()
            except EOFError:
                break
            
            if not user_input:
                continue
            if user_input.lower() in ['exit', 'quit']:
                rclpy.shutdown()
                break
            if user_input.lower() in ['멈춰', '정지', 'stop']:
                print("⏸ 큐 비움, 현재 위치 유지")
                self.setpoint_queue.clear()
                continue
            
            if not self.position_received:
                print("⚠️ PX4 위치 미수신")
                continue
            
            self.process_command(user_input)

    # ──────────────────────────────────────────────
    # 명령 처리 (데코레이터로 history 자동 관리)
    # ──────────────────────────────────────────────

    @track_position  # 명령 실행 전 현재 위치 자동 기록
    def process_command(self, user_input: str):
        cur_x = self.local_position.x
        cur_y = self.local_position.y
        cur_z = self.local_position.z
        cur_yaw = (self.local_position.heading 
                   if hasattr(self.local_position, 'heading') else 0.0)
        
        status = (
            f"[현재 상태 NED]\n"
            f"  x={cur_x:.4f}, y={cur_y:.4f}, z={cur_z:.4f}\n"
            f"  현재 고도={-cur_z:.2f}m\n"
            f"  yaw={cur_yaw:.4f} rad ({math.degrees(cur_yaw):.1f}°)\n"
            f"  history 크기={len(self.position_history)}"
        )

        try:
            print("\n🤔 LLM 호출 중...")
            messages = [
                ("system", SYSTEM_PROMPT),
                ("user", f"{status}\n사용자 명령: {user_input}")
            ]
            result = self.llm.invoke(messages)
            raw = self._clean_json(result.content)
            data = json.loads(raw)
            setpoints = data.get("setpoints", [])

            if not setpoints:
                print("❌ 빈 setpoint")
                return

            print(f"\n📋 LLM 결과 ({len(setpoints)}개):")
            for i, sp in enumerate(setpoints):
                print(f"  {i+1}. {sp}")

            # recall 키워드를 실제 좌표로 치환 후 큐에 추가
            self._enqueue_setpoints(setpoints)
            print("✅ 실행 시작")

        except json.JSONDecodeError as e:
            print(f"❌ JSON 파싱 오류: {e}")
            print(f"원본: {result.content[:200]}")
        except Exception as e:
            print(f"❌ 오류: {e}")

    @resolve_recall  # recall 키워드를 실제 좌표로 치환
    def _enqueue_setpoints(self, setpoints):
        """setpoint들을 큐에 넣고 첫 항목 시작.
        @resolve_recall 데코레이터가 recall 키워드를 실제 좌표로 미리 치환함."""
        self.setpoint_queue.clear()
        for sp in setpoints:
            self.setpoint_queue.append(sp)
        self._consume_next()

    def _clean_json(self, raw: str) -> str:
        """LLM이 반환한 마크다운을 제거하고 순수 JSON 텍스트만 추출합니다."""
        raw = raw.strip()
        if raw.startswith("```"):
            parts = raw.split("```")
            if len(parts) >= 2:
                raw = parts[1]
                if raw.lower().startswith("json"):
                    raw = raw[4:]
        return raw.strip()

    # ──────────────────────────────────────────────
    # setpoint 소비
    # ──────────────────────────────────────────────

    def _consume_next(self):
        if not self.setpoint_queue:
            self.get_logger().info('✅ 모든 setpoint 완료')
            return

        sp = self.setpoint_queue.popleft()
        self.arrival_count = 0

        if sp.get("arm"):
            self._arm()
        if sp.get("offboard"):
            self._engage_offboard()
        if sp.get("land"):
            self._land()
            self.in_landing = True
            return
        if sp.get("disarm"):
            self._disarm()

        if "x" in sp: self.target_x = float(sp["x"])
        if "y" in sp: self.target_y = float(sp["y"])
        if "z" in sp: self.target_z = float(sp["z"])
        if "yaw" in sp: self.target_yaw = float(sp["yaw"])
        
        self.has_target = True
        label = sp.get("_label", "")
        self.get_logger().info(
            f'🎯 {label} → X:{self.target_x:.2f} Y:{self.target_y:.2f} '
            f'Z:{self.target_z:.2f} Yaw:{math.degrees(self.target_yaw):.1f}°')

    # ──────────────────────────────────────────────
    # 타이머 (위치 & 기수 각도 동시 도달 판정 로직)
    # ──────────────────────────────────────────────

    def timer_callback(self):
        self._publish_offboard_heartbeat()
        
        if self.in_landing:
            return
        if not self.has_target:
            return
        
        self._publish_setpoint(self.target_x, self.target_y, 
                                self.target_z, self.target_yaw)
        
        if not self.setpoint_queue:
            return
        
        # 1. 3D 위치 오차 계산
        dist = math.sqrt(
            (self.local_position.x - self.target_x) ** 2 +
            (self.local_position.y - self.target_y) ** 2 +
            (self.local_position.z - self.target_z) ** 2
        )
        
        # 2. 기수(Yaw) 각도 오차 계산 (라디안)
        cur_yaw = (self.local_position.heading 
                   if hasattr(self.local_position, 'heading') else 0.0)
        yaw_diff = abs(cur_yaw - self.target_yaw)
        
        # (360도(2pi)를 넘어가는 오차 최단거리 보정)
        if yaw_diff > math.pi:
            yaw_diff = 2 * math.pi - yaw_diff
            
        # 3. 위치(0.3m 이내) AND 각도(약 5.7도 이내, 0.1 라디안) 모두 만족해야 도달로 인정
        if dist < self.ARRIVAL_THRESHOLD and yaw_diff < 0.1:
            self.arrival_count += 1
            if self.arrival_count >= 10:
                self.get_logger().info('✔ 위치 및 각도 도달 완료')
                self._consume_next()
        else:
            self.arrival_count = 0

    # ──────────────────────────────────────────────
    # 콜백
    # ──────────────────────────────────────────────

    def position_callback(self, msg):
        self.local_position = msg
        if not self.position_received:
            self.position_received = True
            self.get_logger().info(
                f'📡 PX4 위치 수신: X:{msg.x:.2f}, Y:{msg.y:.2f}, Z:{msg.z:.2f}')

    def status_callback(self, msg):
        self.vehicle_status = msg

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pos_str = (f"Pos:({self.local_position.x:.1f},"
                       f"{self.local_position.y:.1f},{self.local_position.z:.1f})")
            tgt_str = (f"Tgt:({self.target_x:.1f},"
                       f"{self.target_y:.1f},{self.target_z:.1f})")
            cv2.putText(cv_image, 
                        f"Q:{len(self.setpoint_queue)} | Hist:{len(self.position_history)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(cv_image, pos_str,
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            cv2.putText(cv_image, tgt_str,
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,255), 2)
            cv2.imshow('Drone View', cv_image)
            cv2.waitKey(1)
        except Exception:
            pass

    # ──────────────────────────────────────────────
    # PX4 저수준 (변경 없음)
    # ──────────────────────────────────────────────

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('⚡ ARM')

    def _disarm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('💤 DISARM')

    def _engage_offboard(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('🎮 OFFBOARD')

    def _land(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('🛬 LAND')

    def _publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def _publish_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command, **params):
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
        self.vehicle_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneUnified()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()