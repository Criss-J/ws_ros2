# 🚁 LLM-Controlled Drone (Raspberry Pi Companion)

자연어 명령으로 PX4 드론을 제어하는 **라즈베리파이 컴패니언 컴퓨터 코드**.  
LangChain + Gemini가 명령을 해석하고, ROS 2로 로컬 PC의 PX4 SITL을 원격 제어합니다.

> "5m 이륙해" → 드론 이륙  
> "6시 방향으로 3m 가" → 회전 후 전진  
> "처음 위치로 돌아가" → 시작점 복귀  

---

## 🏗️ 시스템 구조 (분산)

```
┌─────────────────────────────┐         ┌──────────────────────────────┐
│      라즈베리파이 (RPi)         │   ROS 2 │       로컬 PC (Ubuntu)         │
│                             │  ←DDS→  │                              │
│  drone_unified.py (이 코드)   │         │  ・PX4 SITL (v1.15.x)         │
│  ├─ LangChain + Gemini      │         │  ・Gazebo (gz_x500_mono_cam)  │
│  ├─ 자연어 → 좌표 계산        │         │  ・MicroXRCEAgent (udp4:8888)│
│  ├─ 위치 history (데코레이터) │         │  ・QGroundControl (선택)       │
│  └─ ROS 2 setpoint 발행      │         │                              │
└─────────────────────────────┘         └──────────────────────────────┘
         ▲                                          │
         │           VehicleLocalPosition           │
         └──────────────────────────────────────────┘
                       (PX4 → ROS 2)
```

이 레포지토리는 **라즈베리파이에서 돌아가는 컨트롤러 코드만** 포함합니다.  
로컬 PC의 PX4 시뮬 환경은 별도 구축 필요 (아래 참고).

---

## ✨ 주요 기능

- 🧠 **자연어 명령 처리**: Gemini 2.5 Flash가 자연어를 PX4 NED 좌표로 직접 계산
- 📌 **위치 history 자동 관리**: 데코레이터로 모든 명령의 시작 위치 자동 기록
- ⏪ **위치 복귀**: "이전 위치로", "처음 위치로" 같은 추상 명령 지원
- 🕐 **시계 방향 이동**: "9시 방향으로 5m" 같은 직관적 명령 (비홀로노믹: 회전 후 전진)
- 🎯 **위치 + 각도 동시 도달 판정**: 좌표뿐 아니라 기수 방향까지 정확히 맞춤
- 🦾 **단일 노드 구조**: LLM 호출과 드론 제어가 하나의 ROS 2 노드 안에서 동작

---

## 📦 환경 요구 사항

### 라즈베리파이 (이 코드가 도는 곳)
- **OS**: Ubuntu 22.04 (64-bit, Raspberry Pi 4 권장)
- **ROS 2**: Humble
- **px4_msgs**: `release/1.15` 브랜치 (PX4 버전과 반드시 일치)
- **Python 패키지**:
```bash
  pip install langchain-google-vertexai google-cloud-aiplatform
```

### 로컬 PC (별도 구축 필요)
https://github.com/youngmo123/PX4-ROS2-Gazebo-Drone-Simulation-Template 참고
- **OS**: Ubuntu 22.04
- **ROS 2**: Humble
- **PX4**: v1.15.x (release/1.15 권장)
- **Gazebo**: Harmonic (Gz Sim)
- **MicroXRCE-DDS Agent**: 최신 버전
- **px4_msgs**: 라즈베리파이와 **동일한 브랜치 (release/1.15)** 필수

### 네트워크
- 라즈베리파이와 로컬 PC가 **같은 네트워크**에 있어야 함
- `ROS_DOMAIN_ID` 양쪽 동일해야 함 (기본값 0)

### Google Cloud
Vertex AI 사용을 위해 GCP 프로젝트 + 인증 필요:
```bash
gcloud auth application-default login
```
`drone_unified.py` 안의 `project="your_name"`를 본인 프로젝트 ID로 변경.

---

## 🛠️ 라즈베리파이 설정 (이 레포 사용법)

### 1. px4_msgs 설치 (가장 중요!)

PX4 시뮬 버전과 정확히 일치하는 브랜치를 받아야 합니다.

```bash
cd ~/ws_ros2/src
git clone -b release/1.15 https://github.com/PX4/px4_msgs.git
```

### 2. 이 레포 클론

```bash
cd ~/ws_ros2/src
git clone <YOUR_GITHUB_REPO_URL>
```

### 3. 빌드

```bash
cd ~/ws_ros2
rm -rf build install log    # 클린 빌드 권장
colcon build --symlink-install
source install/setup.bash
```

### 4. 환경 자동 소싱 (선택)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ws_ros2/install/setup.bash" >> ~/.bashrc
```

---

## 🚀 실행 절차

### [로컬 PC] PX4 시뮬 환경 띄우기

**터미널 A — MicroXRCE-DDS Agent**:
```bash
MicroXRCEAgent udp4 -p 8888
```

**터미널 B — PX4 SITL (카메라 모델)**:
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4010 \
PX4_SIM_MODEL=gz_x500_mono_cam \
PX4_GZ_WORLD=baylands \
PX4_GZ_MODEL_POSE="1,1,0.1,0,0,0.9" \
build/px4_sitl_default/bin/px4
```

**터미널 C — QGroundControl (선택)**:
```bash
./QGroundControl-x86_64.AppImage
```

### [라즈베리파이] 통신 확인

```bash
# PX4 토픽이 보이는지
ros2 topic list | grep fmu

# 데이터가 실제로 흐르는지
ros2 topic hz /fmu/out/vehicle_local_position

# 메시지 정의 호환 확인
ros2 interface show px4_msgs/msg/VehicleLocalPosition | head -5
```

토픽이 보이고 hz가 30Hz 정도 찍히면 정상.

### [라즈베리파이] 통합 노드 실행

```bash
ros2 run my_offboard_ctrl drone_unified
```

`📡 PX4 위치 수신` 로그가 뜨면 명령 입력 준비 완료.

---

## 🗣️ 사용 가능한 명령 예시

| 카테고리 | 명령 예시 | 동작 |
|---|---|---|
| **이륙/착륙** | `5m 이륙해` / `착륙` | 5m로 이륙 / 자동 착륙 |
| **수평 이동** | `앞으로 3m` / `뒤로 2m` | 기수 기준 상대 이동 |
| **좌우 이동** | `왼쪽으로 1m` / `오른쪽으로 2m` | 기수 기준 옆 이동 |
| **고도 변경** | `1m 내려와` / `2m 올라가` | 현재 고도 기준 상하 |
| **회전** | `오른쪽 90도 회전` | 시계방향 yaw 변경 |
| **시계 방향** | `6시 방향으로 3m` / `9시 방향으로 가` | 절대 방위로 회전 + 전진 |
| **위치 복귀** | `이전 위치로` / `처음 위치로` | history 기반 복귀 |
| **정지** | `멈춰` / `정지` | 현재 위치 hover |

---

## 🧭 좌표계 (PX4 NED)

PX4 표준 **NED (North-East-Down)** 사용:
- **X**: 북쪽 양수
- **Y**: 동쪽 양수
- **Z**: 아래쪽 양수 → 고도 5m = Z = **-5**
- **yaw**: 라디안, 북쪽이 0, 시계방향 양수

---

## 🧩 핵심 설계 — 데코레이터 패턴

명령 처리 로직에서 **관심사 분리**를 위해 두 개의 데코레이터를 사용:

### `@track_position`
명령 실행 전에 현재 위치를 자동으로 history에 기록.
```python
@track_position
def process_command(self, user_input):
    # 이 함수는 history 코드 한 줄도 없음
    # 데코레이터가 자동 처리
    ...
```

### `@resolve_recall`
LLM이 출력한 `{"recall": "first"}` 같은 추상 키워드를 실제 좌표로 자동 치환.
```python
@resolve_recall
def _enqueue_setpoints(self, setpoints):
    # recall 키워드는 이미 좌표로 변환된 상태로 들어옴
    ...
```

덕분에:
- `process_command`는 LLM 호출만 신경 씀
- `_enqueue_setpoints`는 큐 관리만 신경 씀
- history 로직은 데코레이터 한 곳에 집중

---

## 🎯 LLM 좌표 계산의 철학

이 프로젝트의 핵심 설계 결정:

> **코드에는 takeoff/move/land 같은 의미 함수가 없습니다.**  
> **모든 의미 해석과 좌표 계산은 LLM이 담당합니다.**

코드는 그저 좌표를 받아서 PX4에 전달할 뿐. 사용자가 "앞으로 3m"라고 하면 LLM이 직접:

```
new_x = current_x + 3 * cos(current_yaw)
new_y = current_y + 3 * sin(current_yaw)
```

이 값을 JSON으로 출력하면 코드는 그대로 PX4에 전달.

### 장점
- 자연어 표현의 무한한 다양성에 대응 가능
- 코드는 단순하고 안정적
- LLM 모델을 교체하면 즉시 다른 해석 가능

### 단점
- LLM의 삼각함수 계산 오류 가능성 존재
- LLM 호출 응답 시간 (~2-3초)
- 결정론적이지 않을 수 있음 (`temperature=0.0`으로 완화)

---

## 🐛 트러블슈팅 (자주 발생한 이슈)

| 이슈 | 원인 | 해결 |
|---|---|---|
| `ros2 topic hz`는 되는데 `echo`가 안 됨 | px4_msgs 버전 미스매치 | px4_msgs를 PX4와 동일한 `release/1.15`로 클린 빌드 |
| 콜백이 호출 안 됨 | QoS Durability 미스매치 | PX4 → ROS 2 subscriber는 `VOLATILE`, ROS 2 → PX4 publisher는 `TRANSIENT_LOCAL` |
| 토픽 보이는데 메시지 안 옴 | DDS 통신 끊김 | `ROS_DOMAIN_ID` 양쪽 일치, 같은 네트워크 확인 |
| 이륙 후 이동 시 고도 손실 | `current_z`가 흔들려 잘못 읽힘 | 비행 중엔 이전 `target_z` 유지 (`base_z` 패턴) |
| `initialize_agent` import 실패 | LangChain 0.3+에서 제거 | 단일 LLM 호출 방식으로 단순화 |
| 회전 후 도달 판정 빠름 | 위치만 보고 yaw 무시 | 위치 + yaw 동시 도달 조건 추가 |

---

## 📁 파일 구조

```
my_offboard_ctrl/                # ROS 2 패키지
├── my_offboard_ctrl/
│   └── drone_unified.py         # ← 메인 통합 노드
├── resource/
├── package.xml
├── setup.py
└── README.md
```

---

## 🔮 향후 계획

- [ ] 카메라 영상 + Gemini Vision (`뭐가 보여?` 같은 비전 분석)
- [ ] 음성 입력 (Whisper STT)
- [ ] 안전 한계 (geofence, 최대 고도) 추가
- [ ] 실기 이전 (Pixhawk + 라즈베리파이 직접 시리얼 연결)
- [ ] 비행 데이터 로깅 및 시각화 대시보드

---

## 📚 참고 자료

- [PX4 ROS 2 Offboard Control Guide](https://docs.px4.io/main/en/ros2/offboard_control.html)
- [PX4-Autopilot GitHub](https://github.com/PX4/PX4-Autopilot)
- [px4_msgs GitHub](https://github.com/PX4/px4_msgs)
- [LangChain Documentation](https://docs.langchain.com/)
- 원본 PX4 시뮬 템플릿: [PX4-ROS2-Gazebo-Drone-Simulation-Template](https://github.com/youngmo123/PX4-ROS2-Gazebo-Drone-Simulation-Template)

---

## ⚠️ 주의사항

- 본 시스템은 **시뮬레이션 검증용**으로 개발되었습니다.
- 실기 드론 적용 시 추가 안전 장치(geofence, failsafe, 비상 정지 등) 필수.
- LLM의 응답은 결정론적이지 않을 수 있으므로 안전 critical 시스템에는 별도 검증 필요.
- 이 레포의 코드는 라즈베리파이용입니다. 로컬 PC의 PX4 시뮬 환경은 별도로 구축해야 합니다.

---

## 📝 라이선스

학습/연구 목적의 프로젝트입니다.  
PX4는 BSD 3-Clause, ROS 2는 Apache 2.0 라이선스를 따릅니다.
