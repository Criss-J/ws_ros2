#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import json
import functools

from langchain_google_vertexai import ChatVertexAI
import vertexai

command_pub = None
drone_telemetry = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
position_received = False

last_sent_plan = None


SYSTEM_PROMPT = """당신은 ROS 2 자율비행 드론 관제사입니다.
사용자의 명령을 분석해서 JSON 비행 계획(plan)으로 변환하세요.

[규칙]
- 반드시 아래 JSON 형식만 출력하세요. 다른 말은 절대 하지 마세요.
- JSON 외의 텍스트, 마크다운 백틱(```), 설명 등 절대 금지.

[중요: 고도 변경 vs 수평 이동 구분]
- "올라가", "올라와", "위로", "상승" → dz 양수 (위로 이동)
- "내려가", "내려와", "아래로", "하강", "낮춰" → dz 음수 (아래로 이동)
- "앞으로", "뒤로", "왼쪽", "오른쪽" → 수평 이동, dz=0
- "회전", "돌아" → 회전, dz=0
- 사용자가 고도 변경을 명시하지 않는 한 dz=0.0

[JSON 형식]
{"plan": [{"action": "...", ...}, ...]}

[사용 가능한 action]
1. takeoff       → {"action": "takeoff", "alt": 고도(m, 기본 2.0)}
                   * 땅에서 처음 이륙하는 명령 (예: "이륙해", "5m 이륙해")
2. land          → {"action": "land"}
                   * 완전히 땅에 착지 (예: "착륙해", "내려놔")
3. move          → {"action": "move", "dx": 앞뒤(m), "dy": 좌우(m), "dz": 위아래(m), "d_yaw": 회전각(도)}
                   * dx 양수=앞, 음수=뒤
                   * dy 양수=오른쪽, 음수=왼쪽
                   * dz 양수=위로 상승, 음수=아래로 하강 (단, 땅에 닿지 않는 범위)
                   * d_yaw 양수=오른쪽회전(시계방향), 음수=왼쪽회전(반시계방향)
                   * 비행 중 고도 변경은 land가 아니라 move의 dz로 처리
4. goto          → {"action": "goto", "x": X좌표, "y": Y좌표, "z": 고도(m, 양수), "yaw": 방위각(도)}
5. reverse_last  → {"action": "reverse_last"}
                   * 직전에 실행한 비행 plan을 거꾸로 실행
                   * "역순으로", "거꾸로", "왔던 길로" 표현
6. goto_history  → {"action": "goto_history", "index": -2}
                   * 과거 명령 시작 시점의 위치로 절대 좌표 이동
                   * index=-2: 직전 명령 시작 위치 ("이전 위치")
                   * index=0:  처음 명령 시작 위치 ("처음 위치", "원점")

[중요: land vs move(dz음수) 구분]
- "착륙해", "내려놔", "착지", "랜딩" → land (완전히 땅에 닿음)
- "1m 내려와", "조금만 내려가", "고도 낮춰" → move with dz 음수 (공중에서 하강)

[예시 1] 일반 비행
사용자: "3m 이륙하고 앞으로 5m 가서 착륙해"
출력:
{"plan": [
  {"action": "takeoff", "alt": 3.0},
  {"action": "move", "dx": 5.0, "dy": 0.0, "dz": 0.0, "d_yaw": 0.0},
  {"action": "land"}
]}

[예시 2] 하강 (땅에 닿지 않음)
사용자: "1m 내려와"
출력: {"plan": [{"action": "move", "dx": 0.0, "dy": 0.0, "dz": -1.0, "d_yaw": 0.0}]}

[예시 3] 상승
사용자: "2m 올라가"
출력: {"plan": [{"action": "move", "dx": 0.0, "dy": 0.0, "dz": 2.0, "d_yaw": 0.0}]}

[예시 4] 복합 이동
사용자: "앞으로 3m 가면서 1m 올라가"
출력: {"plan": [{"action": "move", "dx": 3.0, "dy": 0.0, "dz": 1.0, "d_yaw": 0.0}]}

[예시 5] 이전 위치
사용자: "이전 위치로 가줘"
출력: {"plan": [{"action": "goto_history", "index": -2}]}

[예시 6] 처음 위치
사용자: "처음 위치로 돌아가"
출력: {"plan": [{"action": "goto_history", "index": 0}]}

[예시 7] 역순 실행
사용자: "왔던 길로 거꾸로 돌아가"
출력: {"plan": [{"action": "reverse_last"}]}
"""


def remember_plan(func):
    @functools.wraps(func)
    def wrapper(plan_data):
        global last_sent_plan
        plan = plan_data.get("plan", [])
        is_reverse = plan_data.get("reverse", False)
        if plan and not is_reverse:
            # goto_history나 reverse_last 단독이면 history 저장 안 해도 됨
            # 일반 비행 plan만 저장
            actions = [s.get("action") for s in plan]
            is_history_only = all(a == "goto_history" for a in actions)
            if not is_history_only:
                last_sent_plan = plan
                print(f"💾 plan 저장됨 ({len(last_sent_plan)}단계) - 역순 실행 가능")
        return func(plan_data)
    return wrapper


@remember_plan
def publish_plan(plan_data):
    payload = json.dumps(plan_data)
    command_pub.publish(String(data=payload))


def expand_reverse_last(plan_data):
    plan = plan_data.get("plan", [])
    has_reverse = any(s.get("action") == "reverse_last" for s in plan)
    if not has_reverse:
        return plan_data, False
    if last_sent_plan is None:
        return None, True
    return {"reverse": True}, True


class LangChainDroneNode(Node):
    def __init__(self):
        super().__init__('langchain_drone_commander')
        global command_pub
        command_pub = self.create_publisher(String, '/drone_command', 10)
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
    raw = raw.strip()
    if raw.startswith("```"):
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

    print("\n🧠 [플랜 + 역순 + 위치 history] 대뇌가 켜졌습니다.")
    print("   💡 사용 가능한 명령:")
    print("      - 일반 비행: '5m 이륙해', '앞으로 3m'")
    print("      - 역순 실행: '왔던 길로 돌아가' (경로 되짚기)")
    print("      - 이전 위치: '이전 위치로 가줘' (직전 명령 시작점)")
    print("      - 처음 위치: '처음 위치로', '원점으로'\n")

    while rclpy.ok():
        user_input = input("\n🗣️ 명령: ")
        if user_input.lower() in ['exit', 'quit']:
            break
        if not user_input.strip():
            continue
        if user_input.lower() in ['멈춰', '정지', 'stop', 'hold']:
            print("⏸ 현재 위치 유지")
            continue
        if not position_received:
            print("⚠️ 아직 PX4 위치 데이터를 받지 못했습니다.")
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

            print(f"\n📋 LLM이 생성한 plan ({len(plan)}단계):")
            for i, step in enumerate(plan):
                print(f"  {i+1}. {step}")

            expanded, was_reverse = expand_reverse_last(plan_data)
            if was_reverse:
                if expanded is None:
                    print("❌ 이전에 실행한 plan이 없습니다. 먼저 일반 비행을 하세요.")
                    continue
                print(f"🔄 역순 실행으로 변환 → 저장된 plan({len(last_sent_plan)}단계)을 거꾸로 실행")
                publish_plan(expanded)
            else:
                publish_plan(plan_data)

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