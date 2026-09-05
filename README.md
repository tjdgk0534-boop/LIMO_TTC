<div align="center">

# 🚙 LiMO Pro TTC Unprotected Left-Turn Decision
### 2D LiDAR · Ego-motion Compensation · TTC-based STOP / KEEP

![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)
![LiDAR](https://img.shields.io/badge/Sensor-2D%20LiDAR-6A5ACD)
![Control](https://img.shields.io/badge/Decision-TTC%20STOP%20%2F%20KEEP-orange)
![Research](https://img.shields.io/badge/Project-Personally%20Led-success)

**2D LiDAR만으로 상대 차량의 접근 상태를 추정하고, TTC 기반으로 비보호 좌회전의 정지/주행유지를 판단한 ROS2 연구 프로젝트**

</div>

---

## 🔎 30초 요약

| 항목 | 내용 |
|---|---|
| **플랫폼** | LiMO Pro Ackermann mobile robot |
| **환경** | ROS2 Humble, Cartographer, Nav2 기반 주행환경 |
| **센서** | EAI T-Mini Pro 2D LiDAR |
| **핵심 구현** | ROI Target Selection, Ego-motion 보정, 상대거리/상대속도 계산, Filtering, TTC, Activation Zone, STOP/KEEP 제어 |
| **제어 구조** | `/cmd_vel_smooth → TTC Filter → /cmd_vel` |
| **검증** | 4개 대표 시나리오에서 충돌위험 조건 STOP / 안전 조건 KEEP 동작 확인 |
| **프로젝트 성격** | 개인 주도 연구, 공동저자는 상대 로봇 운용 등 실험 진행 보조 |

> Cartographer, Nav2, AMCL, DWB, MPPI는 기존 ROS Package/Plugin입니다. 이 저장소는 해당 Package를 직접 개발했다고 주장하지 않으며, **환경 구성·연동·문제분석·파라미터 조정·실험 검증**을 수행한 내용을 구분해 기록합니다.

---

## 🧩 System Architecture

```mermaid
flowchart LR
    SCAN[/scan\n2D LiDAR] --> ROI[ROI / Target Selection]
    POSE[Pose / Odometry] --> EGO[Ego-motion Compensation]
    ROI --> EGO
    EGO --> REL[Relative Motion]
    REL --> TTC[TTC Calculation]
    POSE --> ZONE[TTC Activation Zone]
    TTC --> ZONE

    NAV[Nav2] --> SM[Velocity Smoother]
    SM -->|/cmd_vel_smooth| FIL[TTC Speed Filter]
    ZONE --> FIL
    FIL -->|KEEP: pass-through| CMD[/cmd_vel]
    FIL -->|STOP: zero Twist| CMD
```

### 핵심 아이디어

```text
LiDAR 측정
   ↓
ROI 내 상대 로봇 선택
   ↓
Ego-motion 좌표계 보정
   ↓
상대거리 / 상대속도
   ↓
TTC 계산
   ↓
Activation Zone에서만 판단 활성화
   ↓
STOP / KEEP
```

---

## 🛠 Tech Stack

| 영역 | 구성 |
|---|---|
| Robotics | ROS2 Humble, LiMO Pro |
| Mapping / Navigation | Cartographer, Nav2, AMCL 기반 구성 |
| Sensor | 2D LiDAR `/scan` |
| State / Motion | `/amcl_pose`, `/odometry/filtered` 등 |
| Perception Logic | ROI, nearest/local-window target selection |
| Motion Estimation | Ego-motion coordinate compensation |
| Filtering | Local median, temporal smoothing |
| Risk Metric | Relative closing speed, TTC |
| Control | TTC Activation Zone, `STOP / KEEP` velocity filtering |
| Validation | CSV logging, graph analysis, 2-robot scenario test |

---

## 💡 Key Engineering Stories

### 01. 정지한 벽이 움직이는 것처럼 계산됨 — Ego-motion Compensation

초기에는 연속 LiDAR 프레임의 Target 좌표를 바로 빼서 속도를 계산했습니다. 하지만 로봇 자체가 이동하면 두 Target 좌표는 **서로 다른 `base_link` 좌표계에서 측정된 값**이므로, 정지한 벽조차 속도를 가진 것처럼 계산될 수 있었습니다.

이를 해결하기 위해 이전 시점 Target을:

```text
previous local frame
      ↓
 world / map frame
      ↓
current local frame
      ↓
compare with current target
```

순서로 변환한 뒤 차분하도록 수정했습니다.

> 핵심: 센서 노이즈로 보기 전에 **좌표계가 동일한 기준에서 비교되고 있는지**부터 확인

---

### 02. Nav2 경로는 나오는데 차량이 움직이지 않음 — Interface Mismatch

RViz에는 경로가 생성되고 `/cmd_vel_nav`도 정상 발행됐지만 차량은 움직이지 않았습니다.

확인 결과:

- Nav2 출력: `geometry_msgs/Twist`
- LiMO Ackermann driver 입력: `ackermann_msgs/AckermannDrive`
- 기대 Topic도 서로 다름

Ackermann command topic에 직접 명령을 발행해 Hardware/Driver가 정상임을 먼저 확인한 뒤, **Twist → Ackermann 변환 구조**를 연결했습니다.

> 문제를 알고리즘이 아니라 **Message Type / Topic Interface 계층**에서 찾아낸 사례

---

### 03. 상대 로봇이 없는데 STOP — Wall False Positive

상대 로봇 없이 주행해도 LiDAR Target이 남아 STOP되는 현상이 발생했습니다.

Target 존재/부재 상황의 Range를 비교하니:

- 상대 로봇 존재 시 약 `1.8 m` 수준
- 상대 로봇 부재 시 벽이 약 `2.1 m` 수준에서 Target으로 선택

개발 당시 최대 탐지거리가 `4 m`여서 벽이 ROI에 포함되는 것을 확인했고, 전방 탐지범위를 약 `2 m` 수준으로 줄인 뒤 다시 주행하여 불필요한 정지 없이 경로를 수행하는 것을 확인했습니다.

---

### 04. STOP 후 다시 출발하지 않음 — ProgressChecker Root Cause

처음에는 TTC Filter가 `0 velocity`를 계속 내보낸다고 의심했습니다. 하지만 Topic과 CSV를 순서대로 확인하니 Filter가 해제된 뒤에도 **상위 Nav2에서 명령 자체가 나오지 않는 것**을 확인했습니다.

원인은 Nav2 `ProgressChecker`와 의도적 정지의 상호작용이었습니다.

```text
movement_time_allowance
2.0 s  →  4.0 s
```

으로 조정한 뒤 STOP 이후 경로주행이 다시 이어지는 것을 확인했습니다.

> 핵심: 증상이 나타난 마지막 Node만 의심하지 않고 **Upstream까지 신호를 역추적**

---

### 05. LiDAR 거리·속도 값 흔들림 — Target Estimator 개선

Target 대표값은 개발 과정에서 단계적으로 변경했습니다.

`ROI 평균/중심값` → `최근접점` → `최근접점 주변 Local Window` → `Local Median + Temporal Filtering`

5-point/3-point, mean/median 조합을 비교했고 최종 논문에서는 속도 Filtering에 **5-sample median**을 사용했습니다.

근거 없는 RMSE/정확도 개선율은 제시하지 않습니다.

---

## 🎯 Final Experiment

| Scenario | Ego Robot | Opposing Robot | 확인 동작 |
|:---:|---:|---:|---|
| 01 | `0.5 m/s` | `0.5 m/s` | 🛑 **STOP** — 충돌위험 조건 |
| 02 | `0.8 m/s` | `0.8 m/s` | 🛑 **STOP** — 충돌위험 조건 |
| 03 | `0.5 m/s` | `0.8 m/s` | ✅ **KEEP** — 상대 로봇 선통과 |
| 04 | `0.8 m/s` | `0.5 m/s` | ✅ **KEEP** — Ego Robot 선통과 |

### Final reference

- TTC threshold: **3.0 s**
- Decision: **STOP / KEEP**
- Spatial activation: 좌회전 정지선 이전 구간에서 TTC 판단 활성화

> 최종 결과는 **“4개 대표 시나리오에서 충돌위험 조건의 STOP과 안전 조건의 KEEP 동작을 확인했다”** 수준으로만 표현합니다. 통계적 성공률, 정확도, 오탐률을 측정한 실험은 아닙니다.

---

## 📂 Repository Guide

```text
.
├── README.md
├── CONTRIBUTION.md
├── NOTICE.md
├── src/
│   ├── speed_only_from_lidar.py
│   └── ttc_speed_filter.py
├── config/
│   ├── development_risk_areas.yaml
│   ├── development_ttc_areas.yaml
│   └── final_experiment_reference.yaml
├── docs/
│   ├── troubleshooting.md
│   └── version-history.md
└── results/
```

### 주요 파일

- [`src/speed_only_from_lidar.py`](src/speed_only_from_lidar.py)  
  LiDAR ROI / Target Selection / Ego-motion 보정 / 속도·TTC / Logging
- [`src/ttc_speed_filter.py`](src/ttc_speed_filter.py)  
  Nav2 속도 명령에 TTC 기반 STOP/KEEP 적용
- [`config/final_experiment_reference.yaml`](config/final_experiment_reference.yaml)  
  개발 중간값과 분리한 최종 논문 기준 Reference
- [`docs/troubleshooting.md`](docs/troubleshooting.md)  
  주요 문제의 증상 → 가설 → 확인 → 수정 과정
- [`docs/version-history.md`](docs/version-history.md)  
  개발 중간 파라미터와 최종값 혼동 방지

---

## ⚠️ Development Snapshot vs Final Result

이 저장소의 Python 코드는 **개발 후기 snapshot**을 보존합니다. 따라서 코드 내부 기본값과 최종 논문 실험값이 항상 동일하지 않습니다.

예:

```text
Development code default TTC STOP threshold : 5.0 s
Final paper reference TTC threshold         : 3.0 s
```

최종 결과에 사용할 값은 [`config/final_experiment_reference.yaml`](config/final_experiment_reference.yaml)을 기준으로 구분해두었습니다.

---

## 🧠 What This Project Shows

이 프로젝트의 핵심은 TTC 공식 자체보다 **센서 데이터가 잘못 보이는 원인을 좌표계·Interface·Navigation·Filtering 계층으로 분해하고 실험으로 검증한 과정**입니다.

```text
Sensor Data
   ↓
Coordinate Frame
   ↓
Target Estimation
   ↓
Relative Motion
   ↓
TTC Decision
   ↓
Velocity Interface
   ↓
Nav2 / Vehicle Behavior
```

각 단계에서 Topic, Message, CSV, 실제 차량 동작을 함께 확인하며 문제 발생 계층을 좁혀갔습니다.

---

## ⚠️ Limitations

- 4개 대표 시나리오 기반 검증으로 통계적 성공률은 산출하지 않았습니다.
- 단순화된 교차로 및 상대 접근 환경입니다.
- Nearest Target 방식은 다중 객체에서 Target Switching 가능성이 있습니다.
- 상대속도 모델은 해당 실험환경에 맞춘 단순화가 포함됩니다.
- 2D LiDAR만으로 객체 Class/Identity를 구분하지 않습니다.
- Activation Zone은 실험환경의 Map Geometry에 의존합니다.
- Cartographer/Nav2/AMCL/MPPI 등 기존 Package를 직접 개발했다고 주장하지 않습니다.

자세한 역할 범위는 [`CONTRIBUTION.md`](CONTRIBUTION.md), 개발 이력은 [`docs/version-history.md`](docs/version-history.md)를 참고해주세요.
