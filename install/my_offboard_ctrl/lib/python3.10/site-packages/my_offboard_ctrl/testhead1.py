#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import json

from langchain_google_vertexai import ChatVertexAI
import vertexai

command_pub = None
drone_telemetry = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
position_received = False

SYSTEM_PROMPT = """당신은 ROS 2 자율비행 드론 관제사입니다.
사용자의 명령을 분석해서 JSON 비행 계획(plan)으로 변환하세요.

[규칙]
- 반드시 아래 JSON 형식만 출력하세요. 다른 말은 절대 하지 마세요.
- JSON 외의 텍스트, 마크다운 백틱(```), 설명 등 절대 금지.
- 사용자가 고도 변경을 명시하지 않으면 dz는 무조건 0.0으로 설정하세요.
- "앞으로 가", "뒤로 가", "회전" 같은 수평 이동/회전 명령에는 항상 dz=0.

[JSON 형식]
{"plan": [{"action": "...", ...}, ...]}

[사용 가능한 action]
1. takeoff  → {"action": "takeoff", "alt": 고도(m, 기본 2.0)}
2. land     → {"action": "land"}
3. move     → {"action": "move", "dx": 앞뒤(m), "dy": 좌우(m), "dz": 위아래(m), "d_yaw": 회전각(도)}
              * dx 양수=앞, 음수=뒤
              * dy 양수=오른쪽, 음수=왼쪽
              * dz 양수=위, 음수=아래
              * d_yaw 양수=오른쪽회전(시계방향), 음수=왼쪽회전(반시계방향)
4. goto     → {"action": "goto", "x": X좌표, "y": Y좌표, "z": 고도(m, 양수), "yaw": 방위각(도)}

[예시]
사용자: "3m 이륙하고 앞으로 5m 가다가 왼쪽으로 90도 돌아서 4m 가고 착륙해"
출력:
{"plan": [
  {"action": "takeoff", "alt": 3.0},
  {"action": "move", "dx": 5.0, "dy": 0.0, "dz": 0.0, "d_yaw": 0.0},
  {"action": "move", "dx": 0.0, "dy": 0.0, "dz": 0.0, "d_yaw": -90.0},
  {"action": "move", "dx": 4.0, "dy": 0.0, "dz": 0.0, "d_yaw": 0.0},
  {"action": "land"}
]}
"""


class LangChainDroneNode(Node):
    def __init__(self):
        super().__init__('langchain_drone_commander')
        global command_pub
        command_pub = self.create_publisher(String, '/drone_command', 10)

        # 🔥 PX4 → ROS2 (/fmu/out/...) 토픽은 VOLATILE durability로 발행됨
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.pos_callback, qos_sub,
        )

    def pos_callback(self, msg):
        global drone_telemetry, position_received
        drone_telemetry["x"] = msg.x
        drone_telemetry["y"] = msg.y
        drone_telemetry["z"] = msg.z
        drone_telemetry["yaw"] = msg.heading if hasattr(msg, 'heading') else 0.0

        if not position_received:
            position_received = True
            self.get_logger().info(
                f'📡 PX4 위치 수신 시작! X:{msg.x:.2f}, Y:{msg.y:.2f}, Z:{msg.z:.2f}')


def clean_json_response(raw: str) -> str:
    """LLM이 가끔 ```json ... ```으로 감싸서 주는 경우 제거"""
    raw = raw.strip()
    if raw.startswith("```"):
        # ```json ... ``` 또는 ``` ... ``` 제거
        parts = raw.split("```")
        if len(parts) >= 2:
            raw = parts[1]
            if raw.lower().startswith("json"):
                raw = raw[4:]
    return raw.strip()


def run_agent():
    vertexai.init(project="geonhui-494205", location="us-central1")
    try:
        llm = ChatVertexAI(model_name="gemini-2.5-flash", temperature=0.0)
    except Exception as e:
        print(f"Vertex AI 오류: {e}")
        return

    print("\n🧠 [플랜 모드] 대뇌가 켜졌습니다.")
    print("   (PX4 위치 수신 대기 중... 첫 명령 전에 '📡 PX4 위치 수신 시작' 로그 확인하세요)\n")

    while rclpy.ok():
        user_input = input("\n🗣️ 명령: ")
        if user_input.lower() in ['exit', 'quit']:
            break
        if not user_input.strip():
            continue

        # 비상 정지: 빈 plan 발행해서 receiver가 거부하도록 (또는 별도 처리)
        if user_input.lower() in ['멈춰', '정지', 'stop', 'hold']:
            print("⏸ 현재 위치 유지 명령 (다음 플랜까지)")
            continue

        if not position_received:
            print("⚠️ 아직 PX4 위치 데이터를 받지 못했습니다. 잠시 후 다시 시도하세요.")
            continue

        status = (f"[현재 드론 위치] X={drone_telemetry['x']:.2f}, "
                  f"Y={drone_telemetry['y']:.2f}, Z={drone_telemetry['z']:.2f}")

        raw = ""
        try:
            messages = [
                ("system", SYSTEM_PROMPT),
                ("user", f"{status}\n사용자 명령: {user_input}")
            ]
            result = llm.invoke(messages)
            raw = clean_json_response(result.content)

            plan_data = json.loads(raw)
            plan = plan_data.get("plan", [])

            if not plan:
                print("❌ 빈 플랜입니다.")
                continue

            print(f"\n📋 비행 계획 ({len(plan)}단계):")
            for i, step in enumerate(plan):
                print(f"  {i+1}. {step}")

            payload = json.dumps(plan_data)
            command_pub.publish(String(data=payload))
            print("✅ 플랜 전송 완료!")

        except json.JSONDecodeError as e:
            print(f"❌ JSON 파싱 오류: {e}\n원본 응답: {raw}")
        except Exception as e:
            print(f"❌ 오류: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LangChainDroneNode()
    threading.Thread(target=run_agent, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()