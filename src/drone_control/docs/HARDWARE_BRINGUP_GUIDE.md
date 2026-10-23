# 실기체 브링업 가이드 — Holybro X500 V2 + Pixhawk 6C + Jetson Orin Nano

시뮬레이션(Gazebo + PX4 SITL)에서 검증한 `drone_control` 스택을 **실제 기체에 올려서 날리기까지**의
전체 절차입니다. 실기체를 처음 다루는 사람이 혼자 따라갈 수 있도록, 순서·배선·명령·확인 기준을 한 단계씩 적었습니다.

> 이 문서에서 **[확인 필요]** 표시는 제가 로컬 PX4 소스나 시뮬로 직접 검증하지 못한 항목입니다.
> 그 항목은 실제 부품의 매뉴얼/보드 실크 표기로 반드시 확인한 뒤 진행하세요.
> 반대로 표시가 없는 PX4 파라미터·포트·명령은 이 워크스페이스의 PX4 소스(`v1.18.0-beta1-696-g3de9e4d`)에서 확인한 것입니다.

---

## 0. 먼저 읽기: 원칙, 일정, 혼자 할 때의 전략

### 절대 원칙 (어기면 사람이 다친다)

1. **프로펠러는 비행 직전에만 장착한다.** 조립·설정·캘리브레이션·벤치 테스트·오프보드 암 테스트 전부 프롭 없이 한다.
2. **첫 전원 인가는 항상 멀티미터와 함께.** 각 전원 분기의 출력 전압을 *부하를 연결하기 전에* 잰다. 스모크 스토퍼가 있으면 첫 인가 때 쓴다.
3. **한 번에 한 가지만 바꾼다.** 배선 하나, 파라미터 하나, 노드 하나. 바꾸기 전 상태로 돌아갈 수 있어야 한다.
4. **순서는 벤치 → 지상 수동 비행 → 저고도 자동 → 임무.** 단계를 건너뛰지 않는다.
5. **레이저는 사람 근처에서 절대 켜지 않는다.** 물리 킬스위치가 소프트웨어와 무관하게 전체 레이저 전원을 끊어야 한다.
6. **RC 조종기는 언제나 켜져 있고, 엄지는 모드 스위치 위에.** 오프보드 비행 중 이상하면 RC로 Position/Land 전환 — 그 순간 오프보드는 끝난다.

### 현실적인 일정 (혼자, 처음)

| 단계 | 내용 | 소요 |
|---|---|---|
| A | 조립·배선·전원 트리 (3장) | 2~3일 |
| B | PX4 펌웨어·QGC 설정·캘리브레이션 (4장) | 1~2일 |
| C | Jetson 세팅·ROS 2·카메라·UART (5장) | 2~3일 |
| D | 지상 PC·Wi-Fi·DDS·웹앱 (6장) | 1일 |
| E | 벤치 테스트 체크리스트 (7장) | 1~2일 |
| F | 비행 테스트 F1~F5 (8장) | 날씨 따라 여러 날 |

### 혼자 하되, 두 가지는 빌리세요

- **첫 수동 비행(F1)은 조종 경험자가 조종하거나 옆에 있어야 합니다.** 동아리 조종기를 빌리는 김에 사람도 빌리세요. 그 전에 시뮬레이터(QGC 가상조이스틱 + SITL, 또는 RC 시뮬)로 스틱 감각을 익히세요.
- **법규 확인**은 2장의 체크리스트대로 공식 사이트에서 직접 확인하세요. 규정은 바뀝니다.

---

## 1. 시뮬레이션 ↔ 실기체: 무엇이 어디서 돌아가는가

### 1.1 대응표

| 시뮬레이션 | 실기체 | 돌아가는 곳 | 코드 변경 |
|---|---|---|---|
| PX4 SITL (`make px4_sitl gz_x500_survey`) | Pixhawk 6C에 **같은 커밋의 PX4** (`make px4_fmu-v6c_default upload`) | Pixhawk | 없음 (파라미터만) |
| `MicroXRCEAgent udp4 -p 8888` | `MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600` | Jetson | 없음 |
| Gazebo 카메라 → ros_gz 브리지 → `/survey/camera/image` | Arducam IMX219(CSI) → `csi_camera` 노드 → 같은 토픽 | Jetson | 새 노드 (제공) |
| Gazebo gpu_lidar → 브리지 → 노드가 `/fmu/in/distance_sensor` 발행 | **TF-Luna → Pixhawk TELEM3 (PX4 `tfmini` 드라이버)** — PX4가 직접 융합 | Pixhawk | 없음 (노드의 범위 전달 경로는 그냥 비활성) |
| `waypoint_navigator` (노트북) | `waypoint_navigator` (`drone_onboard.launch.py`) | Jetson | 없음 |
| RViz 레이저 마커 | `laser_control` 노드 → MOSFET → 레이저 ×3, 서보 ×3 (Pixhawk AUX) | Jetson + Pixhawk | 새 노드 (제공) |
| `ground_station` + 스티칭 웹앱 (같은 노트북) | 같은 것, **지상 PC** — `CapturedFrame`이 Wi-Fi(ROS 2/DDS)로 넘어옴 | 지상 PC | 없음 (DDS 설정만) |
| QGC (UDP, 같은 PC) | QGC ↔ SiK 라디오 (TELEM1) | 지상 PC | 없음 |
| `GZ_IP`, ros_gz, 텍스처 바닥 | 없음 — 실제 지면은 **질감이 있어야** 함 (잔디·자갈·낙엽 OK, 균일한 포장·물 NG) | — | — |

### 1.2 시스템 그림

```
                 ┌──────────────── 기체 ────────────────────────────────────────┐
  RC 조종기 ≈≈≈▶│ 수신기 ─RC IN─▶ Pixhawk 6C ◀─POWER1─ PM02 ◀─XT60─ 4S LiHV     │
                 │                  │  │  │                       │  보조 분배기 │
  QGC(지상PC) ≈≈▶│ SiK 공중 ─TELEM1─┘  │  └─MAIN 1‑4─▶ ESC×4 ▶ 모터  │  ├─ 12V DC‑DC ─▶ Jetson
                 │                     │                            │  ├─ 6V UBEC ──▶ 서보 레일
                 │         TF‑Luna ─TELEM3┤   AUX 1‑3 ─▶ 서보×3        │  └─ 5V Buck ─▶ 킬스위치 ─▶ MOSFET ─▶ 레이저×3
                 │                     │                                            ▲
                 │  M10 GPS ───GPS1────┤                                            │ GPIO 3개
                 │                     └──TELEM2 (UART 921600)──▶ Jetson Orin Nano ─┘
                 │                                                   │  CSI ◀─ IMX219 (하향)
                 └───────────────────────────────────────────────────┼──────────────┘
                                                                     │ Wi‑Fi (ROS 2 / CycloneDDS, 도메인 7)
                                                        ┌────────────▼────────────┐
                                                        │ 지상 PC: ground_station  │
                                                        │   + 스티칭 웹앱(:8000)   │
                                                        │   + QGC (SiK USB)        │
                                                        └─────────────────────────┘
```

### 1.3 데이터 흐름 (어떤 토픽이 어느 링크를 타나)

| 토픽 / 링크 | 경로 | 비고 |
|---|---|---|
| `/fmu/in/*`, `/fmu/out/*` | Pixhawk ⇄ Jetson (TELEM2 UART, uXRCE-DDS) | 921600 bps. 카메라 이미지는 **절대** 이 링크를 타지 않음 |
| `/survey/camera/image` | Jetson 내부 (csi_camera → navigator) | 1280×720 rgb8 ≈ 2.8 MB/프레임. Wi-Fi로 내보내지 않음 |
| `/survey/frames` (`CapturedFrame`) | Jetson → 지상 PC (Wi-Fi) | 임무당 9장만. RELIABLE+TRANSIENT_LOCAL depth 9 |
| `/survey/mosaic/map` | 지상 PC 내부 (RViz) | |
| `/laser/command`, `/laser/state` | Jetson 내부 | |
| MAVLink | Pixhawk ⇄ 지상 PC (SiK, TELEM1, 57600) | QGC: 상태·프리플라이트·파라미터·로그 |

Wi-Fi가 끊겨도 **임무는 Jetson 안에서 계속 돕니다**(내비게이터는 Jetson에 있음). 지상국은 늦게 붙어도 TRANSIENT_LOCAL 덕에 9장을 다 받습니다. Pixhawk↔Jetson UART가 끊기면 PX4가 `COM_OF_LOSS_T`(1 s) 후 `COM_OBL_RC_ACT`(Hold)로 넘어갑니다.

---

## 2. 안전과 법규 (조립 전에)

### 2.1 한국 법규 체크리스트 **[확인 필요 — 공식 사이트에서 최신 기준 확인]**

| 항목 | 확인할 것 | 어디서 |
|---|---|---|
| 기체 신고 | 최대이륙중량 기준(2 kg 초과 시 신고). X500 V2 + 4S 4000 mAh + Jetson/페이로드면 **2 kg을 넘길 가능성이 큼** → 완성 후 저울로 재고 신고 여부 판단 | 드론원스톱 민원서비스 (drone.onestop.go.kr) |
| 조종자 증명 | 250 g 초과 기체: 무게 구간별 자격(온라인 교육/필기/실기) | 한국교통안전공단(TS) 드론 자격 |
| 비행 승인·공역 | 비행 장소가 관제권/비행금지·제한구역인지, 150 m 이하, 주간·가시권 내 | 드론원스톱 지도 |
| 보험 | 학교/동아리 보험 또는 개인 배상책임 | 소속 기관 |
| 촬영 | 항공촬영 신고 대상 여부 | 드론원스톱 |

### 2.2 LiPo / LiHV

- 셀당 만충 **4.35 V(LiHV)**, 비행 종료 3.5 V 이하로 내려가지 않게, 보관 3.8 V. 충전은 LiHV 모드 확인된 충전기로, 자리를 비우지 않는다.
- 셀 체커로 비행 전후 셀별 전압. 셀 편차 0.05 V 이상이면 밸런스 충전.
- 부풀었거나 뜨거워진 팩은 쓰지 않는다. 운반·보관은 LiPo 백.
- XT60 극성, 처음 꽂을 때 스파크(대용량 커패시터) — 스모크 스토퍼 또는 안티스파크 커넥터.

### 2.3 프로펠러

- 벤치 작업 중에는 **빼 둔다**. 장착 시 CW/CCW 표기와 모터 회전 방향을 맞춘다 (4.6절).
- 시동(arm) 전 사람과 10 m 거리. 프롭 교체 후엔 다시 밸런스/조임 확인.

### 2.4 레이저

- 모듈의 **레이저 등급**을 사양서로 확인. 테스트 단계에서는 Class 2 이하 권장 **[확인 필요]**. 5 V "라인 레이저" 모듈은 Class 3R/3B인 경우가 많고, 이 등급은 눈에 직접 들어오면 영구 손상.
- 킬스위치는 레이저 전원 분기(5 V Buck 출력 또는 배터리 분기)에 **직렬로**, 손이 쉽게 닿는 위치에. 인디케이터 LED는 킬스위치 *뒤*(레이저 전원이 실제로 살아 있는지 표시).
- 소프트웨어 인터락(`laser_control`): `hardware_enable` 파라미터 + 임무 노드의 요청 + **armed** + **공중(z < −0.5 m)** 모두 만족해야 켜짐. 벤치에선 `interlock_bypass:=true`로만 켤 수 있고, 그때도 벽을 향하게.
- 비행 중 레이저 방향에는 사람·동물·차량·창문·항공기가 없어야 함. 지면 투영 거리 5 m 기준 삼각형 한 변 ≈ 2.4 m.

### 2.5 비상 절차 (외우기)

| 상황 | 조치 |
|---|---|
| 오프보드 중 이상 거동 | RC 모드 스위치 → **Position**(멈춰서 호버) 또는 **Land**. 오프보드는 즉시 해제됨 |
| 제어 불능·추락 임박·사람 접근 | RC **킬스위치** (모터 즉시 정지 = 추락. 고도 낮고 사람 없을 때만) |
| Wi-Fi/지상국 끊김 | 아무 것도 안 해도 됨 (임무는 Jetson에서 계속). 결과는 Jetson 로컬에도 저장 |
| Jetson 멈춤/UART 끊김 | PX4가 1 s 후 Hold → RC로 Land |
| RC 끊김 | PX4가 RTL (`NAV_RCL_ACT=2`) — **이륙 전 Home 위치와 RTL 고도 확인** |
| 배터리 경고 | QGC 경고 시 즉시 Land/RTL. `BAT1_V_EMPTY` 이하로 비행하지 않기 |

---

## 3. 조립과 배선

### 3.1 조립 순서

1. **프레임·모터·ESC·PDB·랜딩기어** — Holybro X500 V2 ARF 조립 가이드(PX4 docs "Holybro X500 V2 (Pixhawk 6C) Build") 순서대로. ARF는 모터·ESC가 암에 사전 조립된 경우가 많음 **[확인 필요: 키트 구성]**.
2. **Pixhawk 6C 마운트** — 상판 중앙, **화살표가 기체 전방**, 키트의 진동 댐퍼 사용. 케이블이 FC를 당기지 않게.
3. **PM02 V3** — 배터리 XT60 → PM02 입력, PM02 출력 → PDB. PM02의 6핀 JST-GH → Pixhawk **POWER1**.
4. **GPS/컴퍼스 마스트** — 상판, ESC·전원선·배터리에서 최대한 멀리(자기 간섭). 모듈 화살표도 전방.
5. **수신기** — SBUS면 Pixhawk **RC IN(SBUS/PPM)** 포트 **[확인 필요: 수신기 모델]**. 안테나는 탄소 프레임에서 떨어뜨려 서로 90°.
6. **SiK 공중 유닛** — TELEM1.
7. **페이로드 플레이트(하부)** — Jetson(스탠드오프, 냉각 공기 흐름), 카메라 브래킷(광축 수직 하향), TF-Luna 브래킷(수직 하향), 레이저 3개 + 서보 메커니즘. 무게중심이 프레임 중앙에 오도록 배터리 위치로 보정.
8. **Jetson UART ↔ TELEM2**, GPIO ↔ MOSFET, 카메라 리본.

### 3.2 전원 트리와 첫 전원 인가

```
배터리 4S (XT60)
 └─ PM02 V3 ──(6핀)──▶ Pixhawk POWER1 (5.2 V + 전압/전류 센싱)
     └─ PDB ──▶ ESC ×4 ──▶ 모터
         └─ 보조 분배기 (XT30 ×3, 각 분기에 퓨즈)
             ├─ Jetson 분기: 18AWG ─▶ DC‑DC 12 V / 5 A ─(퓨즈 3~5 A)─▶ 배럴 5.5×2.5 (센터 +) ─▶ Jetson
             ├─ 서보 분기: 20AWG ─▶ UBEC 6 V / 5 A ─▶ 서보 전원 버스 (+470~1000 µF) ─▶ 서보 ×3 (+6 V, GND)
             └─ 레이저 분기: ─▶ Buck 5 V / ≥3 A ─▶ [물리 킬스위치] ─▶ MOSFET 보드 V+ ─▶ 레이저 ×3
GND: 모든 분기·Jetson·Pixhawk·MOSFET 보드의 GND를 하나로 (공통 접지). 신호선은 GND 없이는 의미 없음.
```

**첫 전원 인가 절차 (프롭 없음, Jetson/서보/레이저 미연결 상태에서 시작):**

1. 각 DC-DC의 출력 전압을 **무부하**로 측정: 12 V ±0.3, 6 V ±0.3, 5 V ±0.25. 극성(배럴 센터 +) 확인.
2. 이상 없으면 전원을 끄고 Jetson → 서보 → 레이저 순으로 하나씩 연결하며 매번 전압 재측정.
3. 배터리 연결 직후 PM02/ESC가 뜨거워지는지 30초 손으로 확인. 냄새·연기 즉시 분리.
4. QGC에 USB로 연결해 **배터리 전압 표시**가 멀티미터와 ±0.2 V 안에 있는지 (4.5절에서 보정).

### 3.3 Pixhawk 6C 포트 배선

| Pixhawk 6C 포트 | 연결 | PX4 설정 | 비고 |
|---|---|---|---|
| POWER1 | PM02 V3 | `BAT1_*` (4.5절) | |
| **MAIN(I/O) PWM OUT 1~4** | ESC 1~4 신호 | 4.6절 모터 매핑 | PX4 Quad X: 1 = 앞오른쪽(CCW), 2 = 뒤왼쪽(CCW), 3 = 앞왼쪽(CW), 4 = 뒤오른쪽(CW) |
| **FMU(AUX) PWM OUT 1~3** | 서보 1~3 신호 | "Peripheral via Actuator Set 1~3" | 서보 레일 +6 V/GND는 UBEC에서. 레일 허용 전압 **[확인 필요: Holybro 6C 문서, 보통 ≤10 V]** |
| GPS1 | M10 GPS/컴퍼스 (세이프티 스위치·부저 포함) | 자동 | |
| TELEM1 | SiK 공중 유닛 | `MAV_0_CONFIG` 기본 TELEM1, 57600 | |
| **TELEM2** | Jetson UART (아래 3.4) | `UXRCE_DDS_CFG=102`, `SER_TEL2_BAUD=921600` | **VCC(5 V) 핀은 연결하지 않음** |
| **TELEM3** | TF-Luna (UART 모드) | `SENS_TFMINI_CFG=103` | 대안: GPS2 (enum 202) |
| RC IN | 수신기 SBUS **[확인 필요]** | QGC RC 캘리브레이션 | |
| USB-C | 설정·펌웨어 | | 비행 중 분리 |

PX4가 보는 6C의 UART 이름은 이 워크스페이스 소스 기준 `GPS1, GPS2, TELEM1, TELEM2, TELEM3`입니다. 시리얼 포트 enum: **TELEM1=101, TELEM2=102, TELEM3=103, GPS1=201, GPS2=202**.

TELEM JST-GH 6핀 핀아웃(Pixhawk 표준): **1 VCC 5 V, 2 TX(출력), 3 RX(입력), 4 CTS, 5 RTS, 6 GND**. 케이블 쪽 핀 1(빨간 선)이 VCC.

### 3.4 Jetson Orin Nano 40핀 헤더 배선 **[핀 번호는 보드 실크/NVIDIA 핀아웃으로 재확인]**

| Jetson 핀(BOARD) | 기능 | 연결 |
|---|---|---|
| 6 | GND | Pixhawk TELEM2 핀 6 (GND) |
| 8 | UART1 TXD (`/dev/ttyTHS1`) | Pixhawk TELEM2 핀 3 (RX) |
| 10 | UART1 RXD | Pixhawk TELEM2 핀 2 (TX) |
| 7, 29, 31 | GPIO 출력 (3.3 V) | MOSFET 보드 IN1~3 (각각 10 kΩ 풀다운 → GND) |
| 9 | GND | MOSFET 보드 논리 GND |
| 1 | 3.3 V | MOSFET 보드 논리 전원(보드가 요구할 때만) |

- UART는 3.3 V 로직으로 양쪽이 같습니다. **TX↔RX 교차**, GND 공통, 5 V는 연결 금지.
- MOSFET 보드가 3.3 V HIGH로 켜지지 않으면(5 V 로직 전용) 레벨시프터(BOM 조건부 품목)를 사이에.
- 카메라: **CAM0** 22핀 CSI 커넥터. 리본 접점면 방향(커넥터 래치 쪽)을 매뉴얼로 확인. 카메라 보드를 브래킷에 고정할 때 **리본이 기체 후방을 향하면 이미지 위쪽이 전방**이 되는 경우가 많지만 센서마다 다르므로 5.6절의 방향 검증으로 확정.

### 3.5 TF-Luna

| TF-Luna 핀 | 연결 |
|---|---|
| 1 (+5 V) | Pixhawk TELEM3 핀 1 (5 V) 또는 별도 5 V |
| 2 (RXD/SDA) | TELEM3 핀 2 (Pixhawk TX) |
| 3 (TXD/SCL) | TELEM3 핀 3 (Pixhawk RX) |
| 4 (GND) | TELEM3 핀 6 |
| 5 (모드 선택) | **UART 모드**로 두는 배선 **[확인 필요: TF-Luna 매뉴얼의 핀 5 표 — GND면 I²C, 개방/3.3 V면 UART인 것으로 알려짐]** |

- PX4에는 TF-Luna 전용 I²C 드라이버가 없습니다(`src/drivers/distance_sensor/`에 `tfmini`(UART)와 `tf02pro`(I²C)만 있음). TF-Luna의 UART 프레임은 TFmini와 호환이라 `tfmini` 드라이버를 씁니다 **[확인 필요: docs.px4.io → Benewake TFmini 페이지에서 TF-Luna 언급 확인]**.
- 대안(시뮬과 동일한 경로): TF-Luna를 Jetson I²C(핀 3/5)에 붙이고 작은 ROS 노드가 `sensor_msgs/LaserScan`을 `/survey/range/scan`에 발행 → 내비게이터가 지금처럼 `/fmu/in/distance_sensor`로 전달. 노드는 제공하지 않았으니 UART 경로를 권장합니다.
- 마운트: 수직 하향, **빔 아래에 기체 부품이 없게**(시뮬에서 카메라 하우징이 빔 0.15 m 아래 들어가 AGL이 0.19 m로 읽힌 사고가 있었습니다). 최소 거리 0.2 m. 장착 오프셋을 `EKF2_RNG_POS_X/Y/Z`(기체 원점 기준, m, Z는 아래 양수)에.

### 3.6 레이저·서보 메커니즘

- MOSFET 보드: 3.3 V 로직 호환형. 입력마다 10 kΩ 풀다운(부팅 중 Jetson 핀이 떠서 레이저가 켜지는 것 방지). 부하 측 V+는 킬스위치 뒤의 5 V.
- 서보 3개: 신호 AUX1~3, 전원은 **Pixhawk가 아니라 UBEC**(6 V/5 A). 서보 버스에 470~1000 µF 전해 커패시터. 서보 연장선은 페이로드까지.
- 기계: 삼각형/일자 두 위치에 하드스톱을 두면 서보 반복 정밀도가 좋아집니다. `laser_control`의 `servo_triangle`/`servo_line` 값(−1~1)이 두 위치의 PWM입니다 — 서보를 실제로 돌려 보며 맞춥니다(7장 7단계).

### 3.7 배선 품질 체크리스트

- [ ] 모든 GND가 하나로 연결됨 (멀티미터 도통)
- [ ] 고전류선 18AWG, 서보 20~22AWG, 신호 24~28AWG
- [ ] 커넥터 극성 3회 확인(배럴 센터 +, XT30 방향, JST-GH 핀 1)
- [ ] 납땜부 열수축, 케이블 타이, 진동으로 흔들리는 선 없음
- [ ] GPS/컴퍼스에서 배터리·ESC·전원선·Jetson 전원선 거리 최대화
- [ ] Jetson 방열판 위에 공기 흐름(덮지 않기), 카메라·Jetson에 진동 댐퍼
- [ ] 전체 무게 측정 → 신고 기준(2.1절) 확인, 무게중심이 프레임 중앙

---

## 4. Pixhawk / PX4 설정

### 4.1 QGroundControl

지상 PC에 이미 설치되어 있습니다(`~/QGroundControl.AppImage`). Pixhawk를 **USB-C**로 연결하면 자동 연결됩니다. 시뮬에서 쓰던 QGC와 같은 프로그램입니다.

### 4.2 펌웨어: 시뮬레이션과 **같은 커밋**을 올린다

px4_msgs(토픽 이름의 `_v1`/`_v4` 접미사, 메시지 필드)는 PX4 펌웨어 버전에 묶여 있습니다. 시뮬에서 한 번 겪었듯 버전이 다르면 토픽 이름이 달라져 노드가 조용히 멈춥니다. 그래서 QGC의 "Stable" 대신 **이 워크스페이스의 PX4 트리를 빌드해 올립니다**:

```bash
cd ~/PX4-Autopilot
git rev-parse --short HEAD          # 3de9e4d2d3  (= px4_msgs 가 맞춰진 커밋)
# QGC를 완전히 종료한 뒤 (USB 포트를 잡고 있으면 업로드 실패)
make px4_fmu-v6c_default upload     # "Erase / Program / Verify" 후 재부팅
```

처음 빌드는 10분 이상 걸립니다. 업로드 후 QGC를 켜면 "PX4 v1.18.0 beta…" 버전이 보여야 합니다. 나중에 PX4를 업데이트하면 **px4_msgs도 같은 커밋으로 체크아웃해 다시 빌드**하고, `ros2 topic list | grep fmu`로 접미사를 재확인하세요(README "PX4 topic names").

### 4.3 에어프레임

Vehicle Setup → Airframe → **Holybro X500 V2** (PX4 에어프레임 `4019_x500_v2`) → Apply and Restart.

### 4.4 센서·RC·모드·세이프티

QGC Vehicle Setup 순서대로:

1. **Sensors**: Compass → Gyroscope → Accelerometer → Level Horizon. 컴퍼스는 야외, 금속·전원선 멀리. GPS 모듈의 컴퍼스가 "primary"로 잡히는지 확인.
2. **Radio**: 조종기 전 채널 캘리브레이션. 스틱 모드(Mode 2) 확인.
3. **Flight Modes**: 모드 채널에 3단 스위치 → **Stabilized / Position / Hold(또는 Land)**. 별도 2단 스위치에 **Kill switch**. (오프보드는 ROS가 명령으로 진입하므로 스위치 불필요. 오프보드 중 모드 스위치를 움직이면 그 모드로 빠져나옵니다.)
4. **Safety**: RC 신호 상실 → Return(RTL), 데이터링크 상실 → Hold, 배터리 경고/비상 → Return/Land, RTL 고도(장애물보다 높게), **지오펜스**(테스트 장소 반경·최대 고도, 위반 시 Hold/RTL).
5. **세이프티 스위치**(GPS 모듈 버튼): 시동 전에 길게 눌러 해제해야 모터가 돕니다. 귀찮으면 `CBRK_IO_SAFETY` 등으로 끌 수 있지만 처음엔 켜 두세요.

### 4.5 전원 모듈·배터리

Vehicle Setup → Power: 셀 수 **4**, Full **4.35 V**, Empty **3.5 V**(LiHV). "Calculate" 버튼에 멀티미터로 잰 배터리 전압을 넣어 전압 보정. 전류 보정은 호버 후 로그로 다듬어도 됩니다. (`config/px4_params_x500_real.params`에도 `BAT1_*`가 들어 있습니다.)

### 4.6 모터·ESC (프롭 없음!)

1. Vehicle Setup → **Actuators**: 출력 MAIN 1~4 ← Motor 1~4. (에어프레임 선택으로 기본 매핑이 들어와 있음.)
2. **ESC 캘리브레이션**: Power 탭의 "Calibrate ESCs" 절차대로(배터리 분리 → 버튼 → 배터리 연결 → 비프음). BLHeli_S는 PWM 캘리브레이션을 받습니다.
3. Actuators 탭의 **슬라이더로 모터 하나씩** 돌려 (a) 번호 위치, (b) 회전 방향(화살표)을 확인. 방향이 반대면 그 모터의 **ESC-모터 3선 중 2선을 교차**.
4. 모터 번호가 틀리면 Actuators 탭에서 출력 채널 재지정.

### 4.7 서보 (레이저 배열 전환)

Actuators 탭 → AUX(FMU) 1~3 ← **"Peripheral via Actuator Set 1 / 2 / 3"** (이 이름이 PX4 소스의 출력 함수 `Peripheral_via_Actuator_Set`입니다). 각 채널 Min/Max PWM(보통 1000/2000), Disarmed 값은 삼각형 위치의 PWM으로. `laser_control`은 `VEHICLE_CMD_DO_SET_ACTUATOR`(187)로 Set 1~3에 −1~1 값을 보냅니다.

### 4.8 우리 스택용 파라미터 — 파일로 한 번에

`src/drone_control/config/px4_params_x500_real.params`를 QGC **Parameters → Tools → Load from file**로 올린 뒤 재부팅. 내용:

| 파라미터 | 값 | 이유 |
|---|---|---|
| `UXRCE_DDS_CFG` | 102 (TELEM2) | uXRCE-DDS 클라이언트가 TELEM2로 Jetson과 통신 |
| `SER_TEL2_BAUD` | 921600 | 에이전트 `-b 921600`과 일치 |
| `UXRCE_DDS_DOM_ID` | 7 | 노드들의 `ROS_DOMAIN_ID=7` |
| `UXRCE_DDS_SYNCT` | 1 | 노드가 자기 시계로 타임스탬프를 찍어도 PX4가 보정 |
| `MAV_1_CONFIG` | 0 | TELEM2를 MAVLink가 점유하지 않게 (QGC는 TELEM1=`MAV_0_CONFIG`) |
| `SENS_TFMINI_CFG` | 103 (TELEM3) | TF-Luna UART → `distance_sensor` (Jetson 경로를 쓰면 삭제) |
| `EKF2_RNG_CTRL` | 2 | 임무 고도에서도 항상 융합 → `dist_bottom` 유효 |
| `EKF2_HGT_REF` | 1 (GPS) | **시뮬의 2(Range)와 다름.** 실기체는 GPS 기준이 안전. 평지 아닌 곳에서 Range 기준은 고도가 지면 따라 출렁임 |
| `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` | 1.0 s / 5 (Hold) | 오프보드 스트림 끊기면 1초 후 Hold |
| `COM_RCL_EXCEPT` | 0 | 오프보드 중에도 RC 상실은 페일세이프 |
| `NAV_RCL_ACT` | 2 (Return) | RC 상실 → RTL |
| `COM_DISARM_LAND` | 2.0 | 착륙 2초 후 자동 디스암 |
| `BAT1_N_CELLS/V_CHARGED/V_EMPTY` | 4 / 4.35 / 3.5 | LiHV |

직접 추가로 넣을 것: `EKF2_RNG_POS_X/Y/Z`(TF-Luna 장착 오프셋), 지오펜스 `GF_*`(QGC Safety 탭), 필요하면 `NAV_DLL_ACT`(시뮬 에어프레임은 2라 QGC 없이는 arming이 안 됐음 — 실기체 X500 V2 에어프레임의 기본값을 QGC에서 확인).

### 4.9 설정 끝 확인

- QGC 상단에 프리플라이트 경고 없음, GPS **3D fix**(야외), 위성 ≥ 10, 컴퍼스 "ok".
- Parameters 검색창에 `UXRCE`를 쳐서 값이 들어갔는지. `MAVLink Inspector`(Analyze Tools)에서 `DISTANCE_SENSOR`가 들어오는지(TF-Luna).

---

## 5. Jetson Orin Nano 설정

### 5.1 JetPack

1. **JetPack 6.x** (Ubuntu 22.04 기반 → ROS 2 **Humble**과 그대로 맞음). Orin Nano Developer Kit는 NVIDIA "Getting Started" 페이지의 **SD 카드 이미지** 방식이 가장 쉽습니다. 128 GB microSD(A2)에 굽고 첫 부팅 때 사용자 생성. **[확인 필요: JetPack 6 이미지가 SD 부팅을 지원하는 Dev Kit 리비전인지 — 구형 리비전은 SDK Manager로 QSPI 업데이트 필요]**
2. 모니터·키보드로 1회 설정 후엔 **헤드리스(SSH)**로. 전원 모드는 처음엔 15 W(발열·전류 여유), 최종 25 W(Super) 전환은 비행 전 온도 확인 후.
3. 고정 IP(6.1절), SSH 키 로그인, `sudo apt update && sudo apt upgrade`.

### 5.2 ROS 2 Humble

공식 apt 설치(docs.ros.org Humble "Ubuntu (deb packages)") → `ros-humble-ros-base` + `ros-humble-rmw-cyclonedds-cpp`, `python3-colcon-common-extensions`, `python3-pip`. `.bashrc`에:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/drone_ws/install/drone_control/share/drone_control/config/cyclonedds_jetson.xml
```

### 5.3 워크스페이스

```bash
mkdir -p ~/drone_ws/src && cd ~/drone_ws/src
git clone <당신의 GitHub 저장소>  # px4_msgs, px4_ros_com, drone_control_msgs, drone_control 포함
# Jetson에는 필요 없는 것 제외: drone-stitching(지상 PC용), ros_gz(Gazebo용)
touch drone-stitching/COLCON_IGNORE ros_gz/COLCON_IGNORE 2>/dev/null
cd ~/drone_ws && colcon build --packages-up-to drone_control    # px4_msgs 빌드 10~20분
source install/setup.bash
```

`px4_msgs`는 **4.2절에서 올린 PX4 커밋과 같은 것**이어야 합니다(이 저장소의 px4_msgs는 `75cb93a`/`3de9e4d` 계열에 맞춰져 있음). Jetson의 python3-opencv는 numpy와 충돌하지 않으므로 `csi_camera` 노드가 그대로 돕니다.

### 5.4 Micro XRCE-DDS Agent

PX4 docs "uXRCE-DDS (PX4-ROS 2/DDS Bridge)"의 "Micro XRCE-DDS Agent Installation"(소스 빌드, v2.4.x) 그대로:

```bash
cd ~ && git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j4 && sudo make install && sudo ldconfig /usr/local/lib/
```

실행(5.5절의 UART 준비 후):

```bash
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
```

Pixhawk가 켜져 있으면 몇 초 안에 `session established`, `create_topic`… 로그가 줄줄이 뜹니다. 안 뜨면 TX/RX 교차·보레이트·`UXRCE_DDS_CFG`를 의심.

### 5.5 UART(`/dev/ttyTHS1`) 준비

```bash
sudo systemctl stop nvgetty && sudo systemctl disable nvgetty   # 콘솔이 UART1을 잡고 있음
sudo usermod -aG dialout $USER && sudo reboot
ls -l /dev/ttyTHS1                                              # crw-rw---- root dialout
```

### 5.6 카메라

```bash
# 1) 커널이 카메라를 봤는지
dmesg | grep -i imx219 ; ls /dev/video*
# 2) 프리뷰(모니터 있을 때) 또는 1장 캡처
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=1 ! 'video/x-raw(memory:NVMM),width=1280,height=720' ! nvvidconv ! jpegenc ! filesink location=/tmp/cam.jpg
# 3) ROS 노드
ros2 run drone_control csi_camera --ros-args -p fps:=5 -p width:=1280 -p height:=720
ros2 topic hz /survey/camera/image        # ≈ 5 Hz
```

**캘리브레이션(한 번):** 체스보드(예: 8×6 내부 코너, 25 mm)를 인쇄해 평평하게 붙이고

```bash
sudo apt install ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
    image:=/survey/camera/image camera:=/survey/camera
```

Calibrate → Save → 나온 `ost.yaml`을 `~/camera_calibration/imx219.yaml`로. 이후 `csi_camera`에 `camera_info_url:=~/camera_calibration/imx219.yaml`을 주면 `/survey/camera/camera_info`가 발행되고 지상국이 manifest의 `K`/왜곡계수를 그걸로 채웁니다. 내비게이터의 격자 간격용 화각은 `camera_hfov_rad = 2·atan(W/(2·fx))`로 계산해 launch 인자로.

**방향 검증(시뮬과 동일한 가정 확인):** 기체(또는 카메라 브래킷)를 북쪽으로 향하게 바닥에 놓고 `ros2 run image_tools showimage --ros-args -r image:=/survey/camera/image`(또는 저장한 사진)로 보며 기체 **앞쪽에 둔 물체가 이미지 위쪽**, **오른쪽 물체가 이미지 오른쪽**이어야 합니다. 아니면 `csi_camera`의 `flip_method`(0~7)로 맞추거나 지상국 `image_top_yaw_offset_rad`로 보정.

### 5.7 레이저 GPIO

```bash
sudo pip3 install Jetson.GPIO             # JetPack에 포함된 경우도 있음
sudo groupadd -f -r gpio && sudo usermod -aG gpio $USER
sudo cp /usr/local/lib/python3*/dist-packages/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/ 2>/dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger && sudo reboot
```

레이저 대신 **LED**를 MOSFET 출력에 달고 먼저 테스트(7장 8단계).

### 5.8 자동 시작 (나중에)

처음엔 SSH 터미널 두 개(에이전트, launch)로 수동 실행하세요. 안정되면 systemd 서비스(`MicroXRCEAgent`, `drone_onboard.launch.py`)로 부팅 시 자동 실행 — 단 `laser_hardware_enable`은 서비스 파일에서도 기본 false로 두고 비행 당일 명시적으로 켜세요.

---

## 6. 지상 PC와 네트워크

### 6.1 Wi-Fi

- 가장 쉬운 구성: **휴대용 Wi-Fi 라우터**(또는 노트북 핫스팟). 고정 IP: 지상 PC `192.168.4.10`, Jetson `192.168.4.20` (라우터 DHCP 예약 또는 정적).
- 2.4 GHz는 RC 조종기(2.4 GHz)와 간섭할 수 있어 **5 GHz**를 우선, 거리가 멀면 2.4 GHz. 임무 중 Wi-Fi는 9장 전송에만 쓰이므로 끊겨도 비행엔 영향 없음.
- 확인: 양쪽에서 `ping` → `ssh jetson@192.168.4.20`.

### 6.2 ROS 2 (CycloneDDS) 설정

Wi-Fi 멀티캐스트는 믿을 수 없으므로 **유니캐스트 피어 지정**:

| 머신 | 파일 | 수정할 곳 |
|---|---|---|
| Jetson | `config/cyclonedds_jetson.xml` | `NetworkInterface name="wlan0"`(`ip a`로 확인), `Peer address` = 지상 PC IP |
| 지상 PC | `config/cyclonedds_groundpc.xml` | `NetworkInterface name="wlp0s20f3"`(`ip a`), `Peer address` = Jetson IP |

각 머신의 `.bashrc`에 `CYCLONEDDS_URI=file://…/해당 파일`, `ROS_DOMAIN_ID=7`. 확인: Jetson에서 `ros2 run demo_nodes_cpp talker`, 지상 PC에서 `ros2 topic echo /chatter`.

### 6.3 QGC

SiK 지상 유닛을 USB에 꽂으면 QGC가 자동(57600)으로 붙습니다. 지상 PC의 QGC는 **항상 켜 두세요**(프리플라이트 메시지, 배터리, 위치, 로그 다운로드).

### 6.4 지상국 소프트웨어

```bash
# 터미널 G1: 스티칭 웹앱 (대시보드)
cd ~/drone_ws/src/drone-stitching && python3 scripts/start_webapp.py
# 터미널 G2: 지상국
source ~/drone_ws/install/setup.bash
ros2 run drone_control ground_station --ros-args -p camera_info_topic:=/survey/camera/camera_info \
    -p fallback_agl_m:=5.0 -p stitch_backend:=auto
```

(시뮬 launch의 `ground_station` 부분만 떼어 쓰는 것과 같습니다. `camera_info_topic`은 Jetson의 `csi_camera`가 발행하는 토픽 — CapturedFrame과 마찬가지로 Wi-Fi를 넘어옵니다.)

---

## 7. 벤치 테스트 체크리스트 (프롭 없음, 기체는 테이블에 묶어 두기)

각 단계는 **통과 기준**을 만족해야 다음으로 갑니다.

| # | 테스트 | 명령 / 방법 | 통과 기준 |
|---|---|---|---|
| 1 | 전원 트리 | 3.2절 | 각 DC-DC 전압 정상, 30초 후 발열·냄새 없음 |
| 2 | PX4 부팅·QGC | USB 또는 SiK | 프리플라이트 경고 없음(실내라 GPS 제외), 배터리 전압 ±0.2 V |
| 3 | Pixhawk ↔ Jetson | Jetson: `MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600` 그리고 `ros2 topic list \| grep fmu` | `session established`, `/fmu/out/vehicle_status_v4` 등 60여 개 토픽 |
| 4 | Wi-Fi DDS | 지상 PC: `ros2 topic echo /fmu/out/vehicle_status_v4 --once` | 메시지 1개 수신(nav_state, arming_state) |
| 5 | 카메라 | 5.6절 `hz` + 방향 검증 | 5 Hz, 방향 맞음 |
| 6 | TF-Luna | 기체를 들어 0.3~2 m 높이에서 `ros2 run drone_control check_range_fusion --seconds 20`(Jetson) | `dist_bottom` ≈ 실제 높이, `valid=True range_bit=True` |
| 7 | 서보 | Jetson: `ros2 run drone_control laser_control` (mock) + 다른 터미널 `ros2 topic pub -1 /laser/command drone_control_msgs/msg/LaserCommand "{enable: false, pattern: line}"` → `triangle` | 서보가 두 위치로 이동. 안 움직이면 4.7절의 함수 할당·서보 레일 전원 확인 |
| 8 | 레이저(LED로) | 킬스위치 OFF 상태로 LED 연결 → `laser_control --ros-args -p backend:=jetson -p hardware_enable:=true -p interlock_bypass:=true` + `/laser/command {enable: true}` | LED 3개 ON, `enable:false`로 OFF. 그 다음에만 실제 레이저, 벽을 향해, 킬스위치 손에 |
| 9 | 인터락 | 위 상태에서 `interlock_bypass`를 빼고 실행 | 레이저가 **켜지지 않아야** 정상(`vehicle not armed`) |
| 10 | 오프보드 암 | Jetson: `ros2 launch drone_control drone_onboard.launch.py mission:=hover target_altitude:=1.0 camera:=false` | 로그 `PX4 mode -> OFFBOARD`, `arming_state -> ARMED`, 모터가 프롭 없이 저속 회전 → `STATE IDLE -> TAKEOFF` → Ctrl+C → QGC로 Disarm. 암이 거부되면 QGC 메시지(“Preflight Fail: …”)가 이유 |
| 11 | 페일세이프 | 10번 상태에서 Jetson의 launch를 Ctrl+C | 1초 후 PX4가 Hold로 전환(QGC 모드 표시) |

10번은 **모터가 실제로 돕니다** — 프롭이 없는지 세 번 확인하고, 기체를 고정하고, 킬스위치에 손을 두고 하세요.

---

## 8. 비행 테스트 (단계별 go / no-go)

장소: 사람·차 없는 넓은 잔디/흙 바닥(질감 필요), 바람 5 m/s 이하, GPS 3D fix 위성 ≥ 12, 배터리 만충, 지오펜스 설정, 조종기 켜짐, QGC 연결.

| 단계 | 내용 | 고도 | go 기준 | no-go면 |
|---|---|---|---|---|
| F1 | **수동 Stabilized 이륙·호버·착륙** (경험자) | 1~2 m | 떨림 없음, 모터 온도 정상, 10초 호버 유지 | 프롭 방향/밸런스, PID(기본값으로 X500은 보통 OK), 진동(Flight Review) |
| F2 | **Position 모드 호버** 30초, 전후좌우 1 m 이동 | 2 m | 위치 유지 ±0.5 m, 고도 ±0.3 m | 컴퍼스 간섭(회전 시 흔들림=toilet-bowl), GPS 품질 |
| F3 | **오프보드 호버** `mission:=hover target_altitude:=1.5 target_x:=0 target_y:=0` → 중간에 RC로 Position 전환 | 1.5 m | 자동 이륙·호버, RC 전환 즉시 응답 | `COM_OBL_RC_ACT`, 오프보드 스트림(10 Hz) 확인 |
| F4 | **survey** 1장 촬영 + 레이저(짧게, 사람 없는 방향) `mission:=survey target_altitude:=3 laser_hardware_enable:=true laser_backend:=jetson` | 3 m | 사진 저장, `dist_bottom` 유효, 레이저 `LASERS ON` 로그 후 착륙 시 OFF | AGL 경고(`range AGL … disagrees`)면 TF-Luna 마운트/빔 아래 부품 |
| F5 | **mosaic** 9장, `after_mosaic:=land` | 5 m | 9장 전송, 대시보드 n/9 → 지도, 지도에서 지면 질감이 이어짐 | 스티치 실패면 `diagnostics.json`: `insufficient_features`(지면 질감 부족·흔들림), `implausible footprint`(AGL) |

각 비행 후: QGC → Analyze → Log Download → logs.px4.io(Flight Review)에서 진동·컴퍼스·배터리 확인. 배터리는 **용량의 70% 이상 쓰지 않기**(4000 mAh면 2800 mAh).

**레이저 비행 규칙**: 레이저가 켜지는 F4/F5에서는 비행 구역 전체와 그 너머(투영 방향)에 사람이 없고, 킬스위치 담당(본인)이 레이저를 끌 수 있는 거리에 있어야 합니다.

---

## 9. 시뮬 → 실기체 변경 요약

| 항목 | 시뮬 | 실기체 |
|---|---|---|
| PX4 | SITL, 에어프레임 `gz_x500_survey`(`SIM_GZ_EN_LIDAR 0`, `EKF2_HGT_REF 2`) | `px4_fmu-v6c`, 에어프레임 X500 V2, `config/px4_params_x500_real.params` (`EKF2_HGT_REF 1`) |
| 에이전트 | `udp4 -p 8888` | `serial --dev /dev/ttyTHS1 -b 921600` |
| 카메라 | ros_gz 브리지, hfov 1.2 rad 이상적 핀홀 | `csi_camera` + 캘리브레이션 YAML → `camera_info`; `camera_hfov_rad = 2·atan(W/2fx)` |
| 레인지 | 노드가 LaserScan→DistanceSensor 전달 | TF-Luna → Pixhawk TELEM3 (`SENS_TFMINI_CFG 103`) |
| 레이저 | RViz 마커만 | `laser_control` + MOSFET + 서보(“Peripheral via Actuator Set”) |
| launch | `mosaic_mission.launch.py` (전부 한 PC) | Jetson: `drone_onboard.launch.py` / 지상 PC: 웹앱 + `ground_station` |
| DDS | 로컬 | `cyclonedds_jetson.xml` / `cyclonedds_groundpc.xml` (유니캐스트 피어) |
| 바닥 | `textured` 월드 | 질감 있는 실제 지면 |
| 허용오차 | `position_tolerance 0.3`, `reach_hold_time 1.0` | 0.5 / 2.0, `capture_settle_time 3.0` (GPS 위치 노이즈) — `drone_onboard.launch.py` 기본값 |

---

## 10. 트러블슈팅

| 증상 | 원인 | 조치 |
|---|---|---|
| 에이전트에 `session established`가 안 뜸 | TX/RX 미교차, 보레이트, `UXRCE_DDS_CFG`가 TELEM2가 아님, nvgetty가 포트 점유 | 3.4절, 4.8절, 5.5절 |
| `ros2 topic list`에 `/fmu/out/…`는 있는데 노드가 “No /fmu/out/vehicle_status_v4 received” | 펌웨어·px4_msgs 버전 불일치(접미사 다름) | `ros2 topic list \| grep vehicle_status`로 실제 이름 확인 → 4.2절대로 같은 커밋 맞추기 |
| 지상 PC에서 Jetson 토픽이 안 보임 | 도메인 ID, CycloneDDS 인터페이스/피어 주소, 방화벽 | 6.2절. `ros2 doctor --report` |
| 암 거부 “No connection to the GCS” | `NAV_DLL_ACT`>0인데 QGC 미연결 | QGC(SiK) 켜기 또는 `NAV_DLL_ACT 0` |
| 암 거부 “Preflight Fail: …” 기타 | 컴퍼스/가속도 캘리브레이션, 세이프티 스위치, RC 미캘리브 | QGC 메시지가 정확한 이유 |
| 오프보드 진입 직후 Hold/Position으로 빠짐 | 셋포인트 10 Hz 스트림 끊김(UART 과부하·Jetson CPU) | Jetson 전원 모드↑, 카메라 fps↓, `COM_OF_LOSS_T` |
| `dist_bottom` 무효 | TF-Luna 포트/모드, `EKF2_RNG_CTRL`, 빔 아래 부품(0.19 m 같은 값) | 7장 6단계, 3.5절 |
| 사진이 안 찍힘(“Waiting for a camera frame”) | 카메라 노드 죽음, 토픽 이름 | `ros2 topic hz /survey/camera/image` |
| 스티치 `insufficient_features` | 지면 질감 부족·모션 블러·너무 어두움 | 질감 있는 장소, `capture_settle_time`↑, 밝은 시간 |
| 스티치 `implausible footprint scale` | 어떤 프레임의 AGL이 엉뚱함 | 지상국 로그의 AGL 경고 → 센서 마운트 |
| 지도가 90°/180° 돌아감·거울상 | 카메라 방향 가정 | 5.6절 방향 검증, `flip_method`/`image_top_yaw_offset_rad` |
| 레이저가 안 켜짐 | 인터락(`laser request blocked: …` 로그가 이유를 말함), 킬스위치, 풀다운/레벨 | 2.4절, 7장 8·9단계 |

---

## 11. 참고 문서 (읽는 순서대로)

1. PX4 — Holybro X500 V2 (Pixhawk 6C) Build: docs.px4.io/main/en/frames_multicopter/holybro_x500v2_pixhawk6c.html
2. PX4 — Pixhawk 6C 하드웨어/핀아웃: docs.px4.io/main/en/flight_controller/pixhawk6c.html (Holybro docs의 6C Pinout 페이지 포함)
3. PX4 — Basic Configuration (센서·RC·모드·세이프티·ESC): docs.px4.io/main/en/config/
4. PX4 — Actuator Configuration(“Peripheral via Actuator Set”): docs.px4.io/main/en/config/actuators.html
5. PX4 — uXRCE-DDS / ROS 2 사용자 가이드: docs.px4.io/main/en/middleware/uxrce_dds.html, docs.px4.io/main/en/ros2/user_guide.html
6. PX4 — Offboard 모드·페일세이프: docs.px4.io/main/en/flight_modes/offboard.html, docs.px4.io/main/en/config/safety.html
7. PX4 — Benewake TFmini(UART) 드라이버: docs.px4.io/main/en/sensor/tfmini.html
8. PX4 — 파라미터 레퍼런스: docs.px4.io/main/en/advanced_config/parameter_reference.html
9. NVIDIA — Jetson Orin Nano Developer Kit Getting Started, 40-pin 헤더 핀아웃, Jetson.GPIO (github.com/NVIDIA/jetson-gpio)
10. eProsima — Micro XRCE-DDS Agent
11. ROS 2 Humble — 설치, `camera_calibration`, CycloneDDS 네트워크 설정
12. 드론원스톱 민원서비스 — 기체 신고·비행 승인·공역

이 저장소 안: `README.md`(시뮬·토픽·QoS·모자이크), `config/px4_params_x500_real.params`, `config/cyclonedds_*.xml`, `launch/drone_onboard.launch.py`, `drone_control/laser_control.py`, `drone_control/csi_camera_node.py`.
