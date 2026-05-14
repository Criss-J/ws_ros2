#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import json

# 🔥 구버전 호환 랭체인 모듈 임포트
from langchain_google_vertexai import ChatVertexAI
from langchain.tools import tool
from langchain.agents import initialize_agent, AgentType
import vertexai

command_pub = None
# 🌍 AI가 참고할 드론의 실시간 위치 저장소
drone_telemetry = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}

@tool
def command_takeoff(reason: str) -> str:
    """드론을 제자리에서 2m 이륙시킵니다."""
    if command_pub: command_pub.publish(String(data=json.dumps({"action": "takeoff"})))
    return "이륙 명령 전송 완료"

@tool
def command_land(reason: str) -> str:
    """드론을 현재 위치에서 착륙시킵니다."""
    if command_pub: command_pub.publish(String(data=json.dumps({"action": "land"})))
    return "착륙 명령 전송 완료"

@tool
def command_relative_move(dx: float, dy: float, dz: float, d_yaw: float) -> str:
    """드론을 '현재 기수 방향(앞)' 기준으로 이동시킵니다. (dx:앞/뒤, dy:우/좌, dz:위/아래, d_yaw:회전각)
    움직이지 않거나 언급이 없는 축은 0.0을 입력하세요.
    """
    payload = {"action": "move", "dx": float(dx), "dy": float(dy), "dz": float(dz), "d_yaw": float(d_yaw)}
    if command_pub: command_pub.publish(String(data=json.dumps(payload)))
    return f"이동 명령 전송 완료: 앞 {dx}m, 우 {dy}m, 상 {dz}m, 회전 {d_yaw}도"

class LangChainDroneNode(Node):
    def __init__(self):
        super().__init__('langchain_drone_commander')
        global command_pub
        command_pub = self.create_publisher(String, '/drone_command', 10)
        
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        # 👂 대뇌도 드론의 위치를 실시간으로 구독
        self.pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_callback, qos_profile)

    def pos_callback(self, msg):
        global drone_telemetry
        drone_telemetry["x"] = msg.x
        drone_telemetry["y"] = msg.y
        drone_telemetry["z"] = msg.z
        drone_telemetry["yaw"] = msg.heading if hasattr(msg, 'heading') else 0.0

def run_agent():
    vertexai.init(project="geonhui-494205", location="us-central1")
    try:
        llm = ChatVertexAI(model_name="gemini-1.5-flash-001", temperature=0.0)
    except Exception as e:
        print(f"Vertex AI 오류: {e}")
        return

    tools = [command_takeoff, command_land, command_relative_move]
    
    # 🔥 구버전 랭체인 환경에서도 여러 개의 변수(숫자)를 잘 추출하는 STRUCTURED_CHAT 사용
    agent_executor = initialize_agent(
        tools=tools,
        llm=llm,
        agent=AgentType.STRUCTURED_CHAT_ZERO_SHOT_REACT_DESCRIPTION,
        verbose=True,
        agent_kwargs={
            "prefix": "당신은 ROS 2 자율비행 관제사입니다. [시스템 알림]으로 주어지는 현재 드론의 위치를 파악하고 사용자의 명령을 수행하세요. '처음 위치'나 '원점'으로 돌아가라는 명령이 오면, 현재 X, Y 좌표를 0으로 만들기 위한 역방향 이동 거리(dx, dy)를 스스로 계산해서 이동 도구를 호출하세요. 모든 이동 수치는 미터(m) 단위입니다."
        }
    )

    print("\n🧠 [구버전 호환 및 상태 인지형] 랭체인 대뇌가 켜졌습니다.")
    while rclpy.ok():
        user_input = input("\n🗣️ 자연어 명령 입력 (예: 이륙하고 앞으로 5m 가줘): ")
        if user_input.lower() in ['exit', 'quit']: break
        
        # 사용자의 명령 앞에 현재 좌표 삽입 (Context Injection)
        current_status = f"[시스템 알림] 현재 드론 위치: X={drone_telemetry['x']:.2f}m, Y={drone_telemetry['y']:.2f}m, 고도={abs(drone_telemetry['z']):.2f}m. "
        enriched_input = current_status + "사용자 명령: " + user_input
        
        try:
            agent_executor.invoke({"input": enriched_input})
        except Exception as e:
            print(f"실행 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LangChainDroneNode()
    agent_thread = threading.Thread(target=run_agent)
    agent_thread.start()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown(); agent_thread.join()

if __name__ == '__main__':
    main()