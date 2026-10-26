# 실기체 브링업 가이드 — Holybro X500 V2 + Pixhawk 6C + Raspberry Pi 5 + Arducam B0353

시뮬레이션(Gazebo + PX4 SITL)에서 검증한 `drone_control` 스택을 **실제 기체에 올려서 날리기까지**의
전체 절차입니다. 실기체를 처음 다루는 사람이 혼자 따라갈 수 있도록, 순서·배선·명령·확인 기준을 한 단계씩 적었습니다.

> 온보드 컴퓨터는 **Raspberry Pi 5 (8 GB)**, 카메라는 **Arducam B0353**(AR0234 2.3 MP 글로벌 셔터, 3.6 mm M12 렌즈, Pivariety)입니다.
> 임무 목표는 **20 m × 10 m 스티칭 지도**이며, 그에 맞는 고도·격자 간격은 1.4절에 있습니다.
>
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
| C | Raspberry Pi 5 세팅·ROS 2·카메라·UART (5장) | 2~3일 |
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
| `MicroXRCEAgent udp4 -p 8888` | `MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600` | Pi 5 | 없음 |
| Gazebo 카메라 → ros_gz 브리지 → `/survey/camera/image` | Arducam B0353(CSI, libcamera) → `survey_camera` 노드(Picamera2) → 같은 토픽 | Pi 5 | 새 노드 (제공) |
| Gazebo gpu_lidar → 브리지 → 노드가 `/fmu/in/distance_sensor` 발행 | **TF-Luna → Pixhawk TELEM3 (PX4 `tfmini` 드라이버)** — PX4가 직접 융합 | Pixhawk | 없음 (노드의 범위 전달 경로는 그냥 비활성) |
| `waypoint_navigator` (노트북, launch가 `arm_approved:=true`) | `waypoint_navigator` (`drone_onboard.launch.py`, **ARM 게이트**: `ros2 param set … arm_approved true` 전까지 시동 안 걺) | Pi 5 | 없음 (격자는 20×10 m 기준으로 자동 계산) |
| RViz 레이저 마커 | `laser_control` 노드 → MOSFET → 레이저 ×3, 서보 ×3 (Pixhawk AUX). **launch에서 `laser:=true`를 줘야 뜸(기본 꺼짐)** | Pi 5 + Pixhawk | 새 노드 (제공) |
| `ground_station` + 스티칭 웹앱 (같은 노트북) | 같은 것, **지상 PC** — `CapturedFrame`이 Wi-Fi(ROS 2/DDS)로 넘어옴 | 지상 PC | 없음 (DDS 설정만) |
| QGC (UDP, 같은 PC) | QGC ↔ SiK 라디오 (TELEM1) | 지상 PC | 없음 |
| `GZ_IP`, ros_gz, 텍스처 바닥 | 없음 — 실제 지면은 **질감이 있어야** 함 (잔디·자갈·낙엽 OK, 균일한 포장·물 NG) | — | — |

### 1.2 시스템 그림

```
                 ┌──────────────── 기체 ────────────────────────────────────────┐
  RC 조종기 ≈≈≈▶│ 수신기 ─RC IN─▶ Pixhawk 6C ◀─POWER1─ PM02 ◀─XT60─ 4S LiHV     │
                 │                  │  │  │                       │  보조 분배기 │
  QGC(지상PC) ≈≈▶│ SiK 공중 ─TELEM1─┘  │  └─MAIN 1‑4─▶ ESC×4 ▶ 모터  │  ├─ 5V/5A Buck ─▶ Pi 5 (USB‑C)
                 │                     │                            │  ├─ 6V UBEC ──▶ 서보 레일
                 │         TF‑Luna ─TELEM3┤   AUX 1‑3 ─▶ 서보×3        │  └─ 5V Buck ─▶ 킬스위치 ─▶ MOSFET ─▶ 레이저×3
                 │                     │                                            ▲
                 │  M10 GPS ───GPS1────┤                                            │ GPIO 3개
                 │                     └──TELEM2 (UART 921600)──▶ Raspberry Pi 5 ────┘
                 │                                                   │  CSI (CAM/DISP 0) ◀─ Arducam B0353 (하향)
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
| `/fmu/in/*`, `/fmu/out/*` | Pixhawk ⇄ Pi 5 (TELEM2 UART, uXRCE-DDS) | 921600 bps. 카메라 이미지는 **절대** 이 링크를 타지 않음 |
| `/survey/camera/image` | Pi 5 내부 (survey_camera → navigator) | 1920×1200 rgb8 ≈ **6.9 MB/프레임**, 3 Hz, 발행·구독 모두 **RELIABLE**. Wi-Fi로 내보내지 않음 — RViz의 카메라 디스플레이는 `mosaic.rviz`에서 기본 꺼져 있음(켜면 Wi-Fi가 막힘). Pi에서 `ros2 topic info -v /survey/camera/image`로 양쪽 QoS 확인 |
| `/survey/camera/camera_info` | Pi 5 → 지상 PC (Wi-Fi, 작음) | 캘리브레이션 YAML을 줬을 때만. 지상국이 manifest의 K/왜곡계수로 사용 |
| `/survey/frames` (`CapturedFrame`) | Pi 5 → 지상 PC (Wi-Fi) | 임무당 9장만(각 6.9 MB). RELIABLE+TRANSIENT_LOCAL depth 9 |
| `/survey/mosaic/map` | 지상 PC 내부 (RViz) | |
| `/laser/command`, `/laser/state` | Pi 5 내부 | `laser:=true`일 때만 수신자가 있음 |
| MAVLink | Pixhawk ⇄ 지상 PC (SiK, TELEM1, 57600) | QGC: 상태·프리플라이트·파라미터·로그 |

Wi-Fi가 끊겨도 **임무는 Pi 5 안에서 계속 돕니다**(내비게이터는 Pi에 있음). 지상국은 늦게 붙어도 TRANSIENT_LOCAL 덕에 9장을 다 받습니다. Pixhawk↔Pi UART가 끊기면 PX4가 `COM_OF_LOSS_T`(1 s) 후 `COM_OBL_RC_ACT`(Hold)로 넘어갑니다.

### 1.4 임무 기하: 20 × 10 m 모자이크와 고도

스티처는 3×3 = 9장을 요구하므로 스티칭 결과의 크기는 `발자국 + 2 × 격자 간격`(축마다)입니다. 카메라 발자국은 고도에 비례하니, **원하는 지도 크기(20 × 10 m)를 주면 고도가 격자 간격을 정합니다**. 내비게이터가 `mosaic_extent_x/y`(기본 20/10)로 이 계산을 하고, 겹침이 `mosaic_min_overlap`(30 %) 아래로 떨어지거나 한 장이 이미 지도보다 크면 **시동 전에 거부**하며 가능한 고도 범위를 로그에 적습니다.

B0353을 핀홀로 보면(AR0234 1920×1200, 3 µm 픽셀, 3.6 mm 렌즈): `fx = 3.6 mm / 3 µm = 1200 px`, 수평 화각 `2·atan(960/1200) = 77.3°`(1.3495 rad), 발자국 `1.6 h × 1.0 h`. Arducam 카탈로그의 90°(H)는 광각 렌즈의 배럴 왜곡을 포함한 값으로 보여, 핀홀 값이 **보수적**(실제 화각이 더 넓으면 겹침만 늘어남)입니다. 캘리브레이션 뒤 `camera_hfov_rad = 2·atan(W/(2·fx))`로 바꿔 주세요(5.7절).

```bash
ros2 run drone_control mosaic_plan                 # 또는 python3 -m drone_control.mosaic_plan (ROS 없이도 됨)
ros2 run drone_control mosaic_plan --fx 1187.4     # 캘리브레이션 fx로 다시 계산
```

| AGL | 발자국 (m) | 격자 간격 (m) | 겹침 X / Y | GSD | 비고 |
|---|---|---|---|---|---|
| 5.0 | 8.0 × 5.0 | — | X 25 % | 4.2 mm | 거부(30 % 미만) |
| 5.5 | 8.8 × 5.5 | 5.60 × 2.25 | 36 % / 59 % | 4.6 mm | 가능한 최저 |
| **6.25** | **10.0 × 6.25** | **5.00 × 1.875** | **50 % / 70 %** | **5.2 mm** | **기본값** (`drone_onboard.launch.py`) |
| 7.0 | 11.2 × 7.0 | 4.40 × 1.50 | 61 % / 79 % | 5.8 mm | |
| 9.5 | 15.2 × 9.5 | 2.40 × 0.25 | 84 % / 97 % | 7.9 mm | 가능한 최고 |
| 10.0 | 16.0 × 10.0 | — | — | — | 거부(한 장이 이미 10 m) |

- 비행 고도 **6.25 m**(이륙 지점 기준), 셀 간격 기체 **우측으로 5.0 m, 전방으로 1.875 m**, 순서 5→4→1→2→3→6→9→8→7. 20 m 변은 **기체 우측(이미지 가로)** 방향 — `target_yaw:=0`(북향)이면 동서로 20 m, 남북으로 10 m.
- 지상에는 **발자국 여유를 포함해 약 25 × 15 m**의 질감 있는 평지가 필요합니다(잔디·자갈·낙엽 OK, 균일한 포장·물 NG). 중앙(셀 5)이 `target_x/target_y`.
- 지도 해상도: GSD 5.2 mm → `stitch_config.json`의 150 px/m에서 3000 × 1500 px(원 해상도는 ~190 px/m).
- 캘리브레이션 결과 화각이 카탈로그처럼 90°라면 같은 6.25 m에서 발자국 12.5 × 7.8 m, 간격 3.75 × 1.09 m, 겹침 70 % / 86 % — 지도는 그대로 20 × 10 m.
- 셀에서 사진을 못 찍으면(`capture_timeout` 안에 프레임 없음) 다시 안정화 후 재시도, `capture_attempts`(3)회 모두 실패하면 **그 셀에서 HOLD**(스티처가 9장을 다 요구하므로 빈 채로 계속 날지 않음). RC로 Position/Land 전환해 회수.

---

## 2. 안전과 법규 (조립 전에)

### 2.1 한국 법규 체크리스트 **[확인 필요 — 공식 사이트에서 최신 기준 확인]**

| 항목 | 확인할 것 | 어디서 |
|---|---|---|
| 기체 신고 | 최대이륙중량 기준(2 kg 초과 시 신고). X500 V2 + 4S 4000 mAh + Pi 5/페이로드면 **2 kg을 넘길 가능성이 큼** → 완성 후 저울로 재고 신고 여부 판단 | 드론원스톱 민원서비스 (drone.onestop.go.kr) |
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
| Wi-Fi/지상국 끊김 | 아무 것도 안 해도 됨 (임무는 Pi에서 계속). 사진은 Pi 로컬(`~/drone_survey_photos`)에도 저장 |
| Pi 멈춤/UART 끊김 | PX4가 1 s 후 Hold → RC로 Land |
| 셀에서 촬영 3회 실패 → 내비게이터가 그 셀에서 HOLD | 로그 `No photo at gridN after 3 attempt(s)`. RC로 Position/Land, 카메라 점검 후 재시동 |
| launch는 떴는데 시동이 안 걸림 | 정상: ARM 게이트. 기체 옆에서 확인 후 `ros2 param set /waypoint_navigator arm_approved true` (7장 10번) |
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
7. **페이로드 플레이트(하부)** — Pi 5(공식 액티브 쿨러 장착, 스탠드오프, 공기 흐름), 카메라 브래킷(광축 수직 하향, **이미지 위 = 기체 전방**이 되게 커넥터 방향 고정), TF-Luna 브래킷(수직 하향), 레이저 3개 + 서보 메커니즘. 무게중심이 프레임 중앙에 오도록 배터리 위치로 보정.
8. **Pi UART ↔ TELEM2**, GPIO ↔ MOSFET, 카메라 리본(Pi 5용 22핀↔15핀 케이블, 3.4절).

### 3.2 전원 트리와 첫 전원 인가

```
배터리 4S (XT60)
 └─ PM02 V3 ──(6핀)──▶ Pixhawk POWER1 (5.2 V + 전압/전류 센싱)
     └─ PDB ──▶ ESC ×4 ──▶ 모터
         └─ 보조 분배기 (XT30 ×3, 각 분기에 퓨즈)
             ├─ Pi 분기: 18AWG ─▶ Buck 5.1 V / 5 A ─(퓨즈 5 A)─▶ USB‑C (또는 헤더 핀 2/4 +5 V, 6 GND) ─▶ Raspberry Pi 5
             ├─ 서보 분기: 20AWG ─▶ UBEC 6 V / 5 A ─▶ 서보 전원 버스 (+470~1000 µF) ─▶ 서보 ×3 (+6 V, GND)
             └─ 레이저 분기: ─▶ Buck 5 V / ≥3 A ─▶ [물리 킬스위치] ─▶ MOSFET 보드 V+ ─▶ 레이저 ×3
GND: 모든 분기·Pi·Pixhawk·MOSFET 보드의 GND를 하나로 (공통 접지). 신호선은 GND 없이는 의미 없음.

Pi 5 전원 메모: 공식 어댑터는 5.1 V/5 A(USB-PD)이지만 우리 부하(카메라·Wi-Fi·UART, USB 주변기기 없음)는 2~3 A면 충분합니다. PD 협상이 없는 5 V 소스로 켜면 Pi가 USB 포트 전류를 600 mA로 제한하고 "low voltage" 경고를 낼 수 있으므로 `config.txt`에 `usb_max_current_enable=1`을 넣고, 벅의 출력을 **5.1 V**로 맞추고 선을 짧게(전압 강하) **[확인 필요: 부하 시 Pi 입력 전압 ≥ 4.9 V]**.
```

**첫 전원 인가 절차 (프롭 없음, Pi/서보/레이저 미연결 상태에서 시작):**

1. 각 DC-DC의 출력 전압을 **무부하**로 측정: 5.1 V ±0.1(Pi), 6 V ±0.3, 5 V ±0.25. USB-C/헤더 극성 확인.
2. 이상 없으면 전원을 끄고 Pi → 서보 → 레이저 순으로 하나씩 연결하며 매번 전압 재측정.
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
| **TELEM2** | Pi 5 UART (아래 3.4) | `UXRCE_DDS_CFG=102`, `SER_TEL2_BAUD=921600` | **VCC(5 V) 핀은 연결하지 않음** |
| **TELEM3** | TF-Luna (UART 모드) | `SENS_TFMINI_CFG=103` | 대안: GPS2 (enum 202) |
| RC IN | 수신기 SBUS **[확인 필요]** | QGC RC 캘리브레이션 | |
| USB-C | 설정·펌웨어 | | 비행 중 분리 |

PX4가 보는 6C의 UART 이름은 이 워크스페이스 소스 기준 `GPS1, GPS2, TELEM1, TELEM2, TELEM3`입니다. 시리얼 포트 enum: **TELEM1=101, TELEM2=102, TELEM3=103, GPS1=201, GPS2=202**.

TELEM JST-GH 6핀 핀아웃(Pixhawk 표준): **1 VCC 5 V, 2 TX(출력), 3 RX(입력), 4 CTS, 5 RTS, 6 GND**. 케이블 쪽 핀 1(빨간 선)이 VCC.

### 3.4 Raspberry Pi 5 40핀 헤더 배선 **[핀 번호는 보드 실크/pinout.xyz로 재확인]**

| Pi 핀(물리) | BCM | 기능 | 연결 |
|---|---|---|---|
| 6 | — | GND | Pixhawk TELEM2 핀 6 (GND) |
| 8 | GPIO14 | UART0 TXD (`/dev/ttyAMA0`) | Pixhawk TELEM2 핀 3 (RX) |
| 10 | GPIO15 | UART0 RXD | Pixhawk TELEM2 핀 2 (TX) |
| 11, 13, 15 | GPIO17, 27, 22 | GPIO 출력 (3.3 V) — `laser_control`의 `laser_pins` 기본값 | MOSFET 보드 IN1~3 (각각 10 kΩ 풀다운 → GND) |
| 9 | — | GND | MOSFET 보드 논리 GND |
| 1 | — | 3.3 V | MOSFET 보드 논리 전원(보드가 요구할 때만) |
| 2, 4 / 6 | — | +5 V / GND | (USB-C 대신 헤더로 급전할 때) 5.1 V 벅 출력 |

- UART는 3.3 V 로직으로 양쪽이 같습니다. **TX↔RX 교차**, GND 공통, 5 V는 연결 금지. Pi 5의 GPIO14/15 UART는 `config.txt`에 `dtparam=uart0=on`을 넣어야 `/dev/ttyAMA0`로 나타납니다(5.5절). Pi 5의 3핀 디버그 UART 커넥터(`/dev/ttyAMA10`)는 콘솔용이니 쓰지 않습니다.
- GPIO17/27/22는 부팅 시 입력+내부 풀다운이라 뜨지 않지만, 외부 10 kΩ 풀다운은 그대로 둡니다(커널이 올라오기 전·재부팅 중 보호).
- MOSFET 보드가 3.3 V HIGH로 켜지지 않으면(5 V 로직 전용) 레벨시프터(BOM 조건부 품목)를 사이에.
- 카메라: Pi 5의 **CAM/DISP 0** 커넥터(22핀 0.5 mm). B0353에 들어 있는 15핀(1 mm) 리본은 Pi 4용이므로 **Pi 5용 22핀↔15핀 카메라 케이블**(Raspberry Pi "Camera Cable Standard–Mini" 200/300 mm 또는 Arducam 동급)이 따로 필요 **[확인 필요: 구매 키트 구성]**. 리본 접점면 방향은 커넥터 래치 쪽 표기대로. 카메라 보드를 브래킷에 고정할 때 **이미지 위쪽이 기체 전방**이 되게 — 센서/케이블 방향에 따라 다르니 5.6절 방향 검증으로 확정하고, 90° 단위로 틀어졌으면 `camera_rotate_deg`로 맞춥니다.

### 3.5 TF-Luna

| TF-Luna 핀 | 연결 |
|---|---|
| 1 (+5 V) | Pixhawk TELEM3 핀 1 (5 V) 또는 별도 5 V |
| 2 (RXD/SDA) | TELEM3 핀 2 (Pixhawk TX) |
| 3 (TXD/SCL) | TELEM3 핀 3 (Pixhawk RX) |
| 4 (GND) | TELEM3 핀 6 |
| 5 (모드 선택) | **UART 모드**로 두는 배선 **[확인 필요: TF-Luna 매뉴얼의 핀 5 표 — GND면 I²C, 개방/3.3 V면 UART인 것으로 알려짐]** |

- PX4에는 TF-Luna 전용 I²C 드라이버가 없습니다(`src/drivers/distance_sensor/`에 `tfmini`(UART)와 `tf02pro`(I²C)만 있음). TF-Luna의 UART 프레임은 TFmini와 호환이라 `tfmini` 드라이버를 씁니다 **[확인 필요: docs.px4.io → Benewake TFmini 페이지에서 TF-Luna 언급 확인]**.
- 대안(시뮬과 동일한 경로): TF-Luna를 Pi I²C(핀 3/5)에 붙이고 작은 ROS 노드가 `sensor_msgs/LaserScan`을 `/survey/range/scan`에 발행 → 내비게이터가 지금처럼 `/fmu/in/distance_sensor`로 전달. 노드는 제공하지 않았으니 UART 경로를 권장합니다.
- 마운트: 수직 하향, **빔 아래에 기체 부품이 없게**(시뮬에서 카메라 하우징이 빔 0.15 m 아래 들어가 AGL이 0.19 m로 읽힌 사고가 있었습니다). 최소 거리 0.2 m. 장착 오프셋을 `EKF2_RNG_POS_X/Y/Z`(기체 원점 기준, m, Z는 아래 양수)에.

### 3.6 레이저·서보 메커니즘

- MOSFET 보드: 3.3 V 로직 호환형. 입력마다 10 kΩ 풀다운(부팅 중 Pi 핀이 떠서 레이저가 켜지는 것 방지). 부하 측 V+는 킬스위치 뒤의 5 V.
- 서보 3개: 신호 AUX1~3, 전원은 **Pixhawk가 아니라 UBEC**(6 V/5 A). 서보 버스에 470~1000 µF 전해 커패시터. 서보 연장선은 페이로드까지.
- 기계: 삼각형/일자 두 위치에 하드스톱을 두면 서보 반복 정밀도가 좋아집니다. `laser_control`의 `servo_triangle`/`servo_line` 값(−1~1)이 두 위치의 PWM입니다 — 서보를 실제로 돌려 보며 맞춥니다(7장 7단계).

### 3.7 배선 품질 체크리스트

- [ ] 모든 GND가 하나로 연결됨 (멀티미터 도통)
- [ ] 고전류선 18AWG, 서보 20~22AWG, 신호 24~28AWG
- [ ] 커넥터 극성 3회 확인(배럴 센터 +, XT30 방향, JST-GH 핀 1)
- [ ] 납땜부 열수축, 케이블 타이, 진동으로 흔들리는 선 없음
- [ ] GPS/컴퍼스에서 배터리·ESC·전원선·Pi 전원선 거리 최대화
- [ ] Pi 5 액티브 쿨러 위에 공기 흐름(덮지 않기), 카메라·Pi에 진동 댐퍼
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
| `UXRCE_DDS_CFG` | 102 (TELEM2) | uXRCE-DDS 클라이언트가 TELEM2로 Pi와 통신 |
| `SER_TEL2_BAUD` | 921600 | 에이전트 `-b 921600`과 일치 |
| `UXRCE_DDS_DOM_ID` | 7 | 노드들의 `ROS_DOMAIN_ID=7` |
| `UXRCE_DDS_SYNCT` | 1 | 노드가 자기 시계로 타임스탬프를 찍어도 PX4가 보정 |
| `MAV_1_CONFIG` | 0 | TELEM2를 MAVLink가 점유하지 않게 (QGC는 TELEM1=`MAV_0_CONFIG`) |
| `SENS_TFMINI_CFG` | 103 (TELEM3) | TF-Luna UART → `distance_sensor` (Pi 경로를 쓰면 삭제) |
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

## 5. Raspberry Pi 5 설정

### 5.1 OS와 ROS 2 — 플랫폼 결정 **[확인 필요 — 가장 먼저 검증할 항목]**

문제: B0353(Pivariety)은 **Raspberry Pi OS의 libcamera 스택**에서만 공식 지원되고(Arducam: libcamera 전용, raspistill/V4L2-ISP 불가), ROS 2 **Humble**은 Ubuntu 22.04용인데 Ubuntu 22.04는 Pi 5를 지원하지 않습니다. 따라서 Pi에서는 지상 PC(Humble)와 다른 조합이 됩니다.

| 조합 | 카메라 | ROS 2 | 평가 |
|---|---|---|---|
| **A. Raspberry Pi OS Bookworm 64-bit Lite + ROS 2 Jazzy 소스 빌드** | Arducam 공식 경로, `python3-picamera2`가 시스템 Python(3.11)에 설치됨 → `survey_camera` 노드가 그대로 돈다 | Jazzy는 Debian Bookworm을 Tier 3(소스 빌드)로 지원. `ros_base` 빌드 1~2시간(1회) | **권장**. 지상 PC Humble ↔ Pi Jazzy 통신은 같은 메시지 정의(px4_msgs·drone_control_msgs 같은 소스, sensor_msgs 동일)와 **양쪽 CycloneDDS**로 동작하는 것으로 알려져 있으나 공식 지원 조합은 아님 → 7장 4번 벤치 테스트로 확정 |
| B. Raspberry Pi OS + ROS 2 Jazzy Docker(`ros:jazzy`) | 호스트 커널 드라이버는 OK지만 컨테이너 안에 Pi용 libcamera·picamera2를 다시 깔아야 함(Ubuntu 베이스엔 패키지 없음) | 설치는 쉬움 | 카메라 때문에 비추천 |
| C. Ubuntu 24.04 (Pi) + ROS 2 Jazzy apt | Arducam이 지원하지 않음. Ubuntu의 libcamera에 Pivariety 튜닝/picamera2가 있는지 미확인 | 가장 쉬움 | `rpicam-hello`/`picamera2`가 B0353을 잡는 걸 확인한 경우에만 |

A로 진행합니다. 코드는 Humble/Jazzy 양쪽 rclpy API만 사용합니다. 지상 PC와 Pi 모두 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`(Jazzy 기본은 Fast DDS이므로 반드시 지정).

**검증(2026-10-27, 노트북 SITL):** 같은 분리를 노트북에서 재현했습니다 — 내비게이터는 RoboStack Jazzy 환경(x86, rmw_cyclonedds)에서, PX4 SITL·에이전트·Gazebo 브리지·지상국은 Humble에서. PX4 토픽(Humble 에이전트 → Jazzy), 카메라/레인지(Humble 브리지 → Jazzy), ARM 승인 서비스(Humble → Jazzy `set_parameters`), `CapturedFrame` 9장(Jazzy → Humble 지상국, 각 3.7 MB)과 스티칭까지 전부 통과했습니다. 따라서 **메시지 정의가 같고 양쪽이 CycloneDDS이면 Humble↔Jazzy는 동작**합니다. 남은 미확인은 Pi(arm64, Debian 소스 빌드)라는 플랫폼 자체뿐이며, 7장 4번 벤치 테스트로 확인합니다. 참고: Humble 쪽 `ros2 param set` CLI가 `ros2 daemon`이 멈춰 먹통이 될 수 있으니, 그러면 `ros2 daemon stop` 후 재시도하거나 README "Verified"의 rclpy 2줄 클라이언트를 쓰세요.

### 5.2 OS 설치

1. Raspberry Pi Imager → **Raspberry Pi OS Lite (64-bit, Bookworm)**. 고급 설정에서 호스트명, 사용자, SSH 키, Wi-Fi(우리 라우터 SSID) 미리 설정. 64 GB 이상 A2 microSD(또는 NVMe HAT).
2. 첫 부팅 후 `sudo apt update && sudo apt full-upgrade && sudo reboot`(커널 ≥ 6.6이면 Pivariety 드라이버가 커널에 들어 있음).
3. 고정 IP(6.1절), `sudo raspi-config` → Interface Options → Serial Port → "login shell over serial: **No**", "serial hardware: **Yes**".
4. `/boot/firmware/config.txt`에 추가 후 재부팅:

```ini
[all]
dtparam=uart0=on                 # GPIO14/15 -> /dev/ttyAMA0 (Pixhawk TELEM2)
camera_auto_detect=0
dtoverlay=arducam-pivariety,cam0  # B0353 on CAM/DISP 0 (CAM/DISP 1 이면 ,cam1) [확인 필요]
usb_max_current_enable=1          # PD 협상 없는 5 V 전원
```

5. 커널 UDP 수신 버퍼(카메라 6.9 MB 프레임이 Pi 안에서 DDS를 타므로 **필수**):

```bash
printf 'net.core.rmem_max=2147483647\nnet.core.rmem_default=2147483647\n' | sudo tee /etc/sysctl.d/60-ros2-dds.conf
sudo sysctl --system
```

### 5.3 카메라 스택 (Arducam Pivariety)

Arducam Quick Start(docs.arducam.com → Raspberry Pi Camera → Pivariety Camera)대로:

```bash
wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
chmod +x install_pivariety_pkgs.sh
./install_pivariety_pkgs.sh -p libcamera_dev
./install_pivariety_pkgs.sh -p libcamera_apps
sudo apt install -y python3-picamera2 python3-numpy python3-gpiozero python3-lgpio
sudo reboot
rpicam-hello --list-cameras        # "arducam-pivariety" 가 보여야 함 (안 보이면 리본/커넥터/cam0·cam1)
rpicam-still -o /tmp/cam.jpg --width 1920 --height 1200 --immediate
python3 -c "from picamera2 import Picamera2; c=Picamera2(); print(c.sensor_modes)"
```

Pivariety 드라이버는 **ISP를 거친 컬러 프레임을 libcamera로만** 내보냅니다(`/dev/video0`을 OpenCV로 열면 RAW10만 나옴). 그래서 `survey_camera`의 기본 백엔드가 `picamera2`입니다. `backend:=v4l2`는 USB 웹캠으로 벤치 테스트할 때만.

### 5.4 ROS 2 Jazzy 소스 빌드

ROS 2 공식 문서 "Jazzy → Installation → Alternatives → Ubuntu/Debian (source)"의 Debian Bookworm 절차. 요약:

```bash
sudo apt install -y python3-colcon-common-extensions python3-vcstool python3-rosdep git cmake build-essential
sudo rosdep init && rosdep update
mkdir -p ~/ros2_jazzy/src && cd ~/ros2_jazzy
vcs import --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos src
# 데스크톱·시뮬·rviz 제외: ros_base + 우리가 쓰는 것(tf2_ros, sensor_msgs, visualization_msgs, rmw_cyclonedds_cpp, launch_ros, ros2topic/ros2launch 등)
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
colcon build --merge-install --packages-up-to ros_base rmw_cyclonedds_cpp tf2_ros sensor_msgs visualization_msgs launch_ros ros2launch ros2topic ros2run ros2param ros2node   # 1~2시간, 메모리 부족 시 --parallel-workers 2
```

`.bashrc`에:

```bash
source ~/ros2_jazzy/install/setup.bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/drone_ws/install/drone_control/share/drone_control/config/cyclonedds_onboard.xml
```

### 5.5 워크스페이스

```bash
mkdir -p ~/drone_ws/src && cd ~/drone_ws/src
git clone <당신의 GitHub 저장소>  # px4_msgs, drone_control_msgs, drone_control 포함
# Pi에는 필요 없는 것 제외: drone-stitching(지상 PC용), ros_gz(Gazebo용), px4_ros_com(예제)
touch drone-stitching/COLCON_IGNORE ros_gz/COLCON_IGNORE px4_ros_com/COLCON_IGNORE 2>/dev/null
cd ~/drone_ws && colcon build --packages-up-to drone_control    # px4_msgs 빌드 20~40분 (Pi 5)
source install/setup.bash
ros2 run drone_control mosaic_plan                             # 1.4절 표가 그대로 나오면 OK
```

`px4_msgs`는 **4.2절에서 올린 PX4 커밋과 같은 것**이어야 합니다(이 저장소의 px4_msgs는 `75cb93a`/`3de9e4d` 계열에 맞춰져 있음). 지상 PC(Humble)도 같은 소스의 px4_msgs·drone_control_msgs를 빌드해야 양쪽 메시지 정의가 일치합니다.

### 5.6 Micro XRCE-DDS Agent와 UART

PX4 docs "uXRCE-DDS (PX4-ROS 2/DDS Bridge)"의 "Micro XRCE-DDS Agent Installation"(소스 빌드, v2.4.x) 그대로:

```bash
cd ~ && git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j4 && sudo make install && sudo ldconfig /usr/local/lib/
```

UART 준비와 실행:

```bash
sudo usermod -aG dialout,video,gpio $USER && sudo reboot
ls -l /dev/ttyAMA0                                   # crw-rw---- root dialout (5.2절 config.txt 적용 후)
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

Pixhawk가 켜져 있으면 몇 초 안에 `session established`, `create_topic`… 로그가 줄줄이 뜹니다. 안 뜨면 TX/RX 교차·보레이트·`UXRCE_DDS_CFG`·시리얼 콘솔이 꺼졌는지(`/boot/firmware/cmdline.txt`에 `console=serial0` 없어야 함)를 의심.

### 5.7 카메라 노드, 캘리브레이션, 방향 검증

```bash
ros2 run drone_control survey_camera --ros-args -p fps:=3            # 1920x1200 rgb8, RELIABLE
ros2 topic hz /survey/camera/image                                   # ≈ 3 Hz
ros2 topic info -v /survey/camera/image                              # Publisher: Reliability RELIABLE
```

**캘리브레이션(한 번):** 체스보드(예: 8×6 내부 코너, 25 mm)를 인쇄해 평평하게 붙입니다. GUI가 필요하므로 지상 PC에서 돌립니다(Wi-Fi로 6.9 MB 프레임을 받으니 `fps:=1`로 낮추고 이때만):

```bash
# Pi
ros2 run drone_control survey_camera --ros-args -p fps:=1
# 지상 PC
sudo apt install ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
    image:=/survey/camera/image camera:=/survey/camera --no-service-check
```

Calibrate → Save → 나온 `ost.yaml`을 Pi의 `~/camera_calibration/b0353.yaml`로 복사. 이후 launch에 `camera_info_url:=~/camera_calibration/b0353.yaml`을 주면 `/survey/camera/camera_info`가 발행되고 지상국이 manifest의 `K`/왜곡계수를 그걸로 채웁니다. 격자 간격용 화각은 `camera_hfov_rad = 2·atan(W/(2·fx))`(fx = `camera_matrix` 첫 값)로 계산해 launch 인자로 — `ros2 run drone_control mosaic_plan --fx <fx>`가 값과 launch 인자를 그대로 출력합니다. 내비게이터는 CameraInfo를 받으면 설정값과 2 % 이상 다를 때 경고합니다.

**방향 검증(시뮬과 동일한 가정 확인):** 기체(또는 카메라 브래킷)를 북쪽으로 향하게 바닥에 놓고 `ros2 run image_tools showimage --ros-args -r image:=/survey/camera/image`(지상 PC, 잠깐만) 또는 Pi에 저장된 사진으로 보며 기체 **앞쪽에 둔 물체가 이미지 위쪽**, **오른쪽 물체가 이미지 오른쪽**이어야 합니다. 90° 단위로 돌아가 있으면 `camera_rotate_deg:=90/180/270`, 미세한 각도 차이는 지상국 `image_top_yaw_offset_rad`. 빨간 물체가 파랗게 보이면 `swap_rb:=true`.

**노출:** 호버 중 흔들림으로 모션 블러가 생기면 `camera_exposure_us:=3000`(주광 2~4 ms, 글로벌 셔터라 롤링 왜곡은 없음) + `camera_analogue_gain`으로 밝기 보정.

### 5.8 레이저 GPIO

gpiozero/lgpio(5.3절에서 설치)가 Pi 5의 RP1 GPIO를 처리합니다. 핀 번호는 **BCM**(기본 17, 27, 22 = 물리 11, 13, 15). 레이저 대신 **LED**를 MOSFET 출력에 달고 먼저 테스트(7장 8단계):

```bash
ros2 run drone_control laser_control --ros-args -p backend:=rpi -p hardware_enable:=true -p interlock_bypass:=true
```

### 5.9 자동 시작 (나중에)

처음엔 SSH 터미널 두 개(에이전트, launch)로 수동 실행하세요. 안정되면 systemd 서비스(`MicroXRCEAgent`, `drone_onboard.launch.py`)로 부팅 시 자동 실행 — 단 `laser`/`laser_hardware_enable`은 서비스 파일에서도 기본 false로 두고 비행 당일 명시적으로 켜세요. ARM 게이트(`arm_approved`)는 launch 인자가 아니라 **런타임 파라미터**라 서비스로 자동 시작해도 기체가 스스로 시동을 걸지 않습니다. 비행 직전 기체 옆에서 `ros2 param set /waypoint_navigator arm_approved true`.

---

## 6. 지상 PC와 네트워크

### 6.1 Wi-Fi

- 가장 쉬운 구성: **휴대용 Wi-Fi 라우터**(또는 노트북 핫스팟). 고정 IP: 지상 PC `192.168.4.10`, Pi 5 `192.168.4.20` (라우터 DHCP 예약 또는 정적).
- 2.4 GHz는 RC 조종기(2.4 GHz)와 간섭할 수 있어 **5 GHz**를 우선, 거리가 멀면 2.4 GHz. 임무 중 Wi-Fi는 9장 전송에만 쓰이므로 끊겨도 비행엔 영향 없음.
- 확인: 양쪽에서 `ping` → `ssh pi@192.168.4.20`.

### 6.2 ROS 2 (CycloneDDS) 설정

Wi-Fi 멀티캐스트는 믿을 수 없으므로 **유니캐스트 피어 지정**:

| 머신 | 파일 | 수정할 곳 |
|---|---|---|
| Pi 5 | `config/cyclonedds_onboard.xml` | `NetworkInterface name="wlan0"`(`ip a`로 확인), `Peer address` = 지상 PC IP |
| 지상 PC | `config/cyclonedds_groundpc.xml` | `NetworkInterface name="wlp0s20f3"`(`ip a`), `Peer address` = Pi IP |

각 머신의 `.bashrc`에 `CYCLONEDDS_URI=file://…/해당 파일`, `ROS_DOMAIN_ID=7`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. 확인: Pi에서 `ros2 run demo_nodes_cpp talker`(또는 `ros2 topic pub /chatter std_msgs/msg/String "{data: hi}"`), 지상 PC에서 `ros2 topic echo /chatter` — Jazzy↔Humble 통신 확인이기도 합니다.

### 6.3 QGC

SiK 지상 유닛을 USB에 꽂으면 QGC가 자동(57600)으로 붙습니다. 지상 PC의 QGC는 **항상 켜 두세요**(프리플라이트 메시지, 배터리, 위치, 로그 다운로드).

### 6.4 지상국 소프트웨어

```bash
# 터미널 G1: 스티칭 웹앱 (대시보드)
cd ~/drone_ws/src/drone-stitching && python3 scripts/start_webapp.py
# 터미널 G2: 지상국
source ~/drone_ws/install/setup.bash
ros2 run drone_control ground_station --ros-args -p camera_info_topic:=/survey/camera/camera_info \
    -p camera_hfov_rad:=1.3495 -p fallback_agl_m:=6.25 -p stitch_backend:=auto
```

(시뮬 launch의 `ground_station` 부분만 떼어 쓰는 것과 같습니다. `camera_info_topic`은 Pi의 `survey_camera`가 캘리브레이션 YAML을 받았을 때 발행하는 작은 토픽 — CapturedFrame과 마찬가지로 Wi-Fi를 넘어옵니다. `fallback_agl_m`은 비행 고도(6.25)로.)

---

## 7. 벤치 테스트 체크리스트 (프롭 없음, 기체는 테이블에 묶어 두기)

각 단계는 **통과 기준**을 만족해야 다음으로 갑니다.

| # | 테스트 | 명령 / 방법 | 통과 기준 |
|---|---|---|---|
| 1 | 전원 트리 | 3.2절 | 각 DC-DC 전압 정상, 30초 후 발열·냄새 없음 |
| 2 | PX4 부팅·QGC | USB 또는 SiK | 프리플라이트 경고 없음(실내라 GPS 제외), 배터리 전압 ±0.2 V |
| 3 | Pixhawk ↔ Pi | Pi: `MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600` 그리고 `ros2 topic list \| grep fmu` | `session established`, `/fmu/out/vehicle_status_v4` 등 60여 개 토픽 |
| 4 | Wi-Fi DDS (Humble ↔ Jazzy) | 지상 PC: `ros2 topic echo /fmu/out/vehicle_status_v4 --once` | 메시지 1개 수신(nav_state, arming_state). 안 되면 5.1절 플랫폼 결정 재검토 |
| 5 | 카메라 | 5.7절 `hz` + `topic info -v` + 방향 검증 | 3 Hz, 발행·구독 모두 RELIABLE, 방향 맞음 |
| 6 | TF-Luna | 기체를 들어 0.3~2 m 높이에서 `ros2 run drone_control check_range_fusion --seconds 20`(Pi) | `dist_bottom` ≈ 실제 높이, `valid=True range_bit=True` |
| 7 | 서보 | Pi: `ros2 run drone_control laser_control --ros-args -p servo_enable:=true` (mock) + 다른 터미널 `ros2 topic pub -1 /laser/command drone_control_msgs/msg/LaserCommand "{enable: false, pattern: line}"` → `triangle` | 서보가 두 위치로 이동. 안 움직이면 4.7절의 함수 할당·서보 레일 전원 확인. (`servo_enable` 기본값은 false) |
| 8 | 레이저(LED로) | 킬스위치 OFF 상태로 LED 연결 → `laser_control --ros-args -p backend:=rpi -p hardware_enable:=true -p interlock_bypass:=true` + `/laser/command {enable: true}` | LED 3개 ON, `enable:false`로 OFF. 그 다음에만 실제 레이저, 벽을 향해, 킬스위치 손에 |
| 9 | 인터락 | 위 상태에서 `interlock_bypass`를 빼고 실행 | 레이저가 **켜지지 않아야** 정상(`vehicle not armed`) |
| 10 | 오프보드 암 (ARM 게이트) | Pi: `ros2 launch drone_control drone_onboard.launch.py mission:=hover target_altitude:=1.0 camera:=false` → 로그에 `ARM gate closed … waiting for ros2 param set` 이 10초마다 뜨고 **아무 일도 안 일어나야** 정상. 그 다음 다른 터미널(Pi 또는 지상 PC)에서 `ros2 param set /waypoint_navigator arm_approved true` | 승인 전: 모터 정지, QGC 모드 변화 없음. 승인 후: 로그 `ARM APPROVED`, `PX4 mode -> OFFBOARD`, `arming_state -> ARMED`, 모터가 프롭 없이 저속 회전 → `STATE IDLE -> TAKEOFF` → Ctrl+C → QGC로 Disarm. 암이 거부되면 QGC 메시지(“Preflight Fail: …”)가 이유 |
| 10b | 촬영 재시도·HOLD (SITL) | 노트북 SITL에서 `ros2 launch drone_control mosaic_mission.launch.py bridge:=false capture_timeout:=10` (카메라 프레임이 전혀 없는 상황) | 로그 `Capture attempt 1/3 … failed`, `2/3 … failed` → `STATE CAPTURE -> HOLD (capture failed at grid5)`, 그 자리에서 호버 유지(날아가지 않음) |
| 11 | 페일세이프 | 10번 상태에서 Pi의 launch를 Ctrl+C | 1초 후 PX4가 Hold로 전환(QGC 모드 표시) |

10번은 **모터가 실제로 돕니다** — 프롭이 없는지 세 번 확인하고, 기체를 고정하고, 킬스위치에 손을 두고 하세요.

---

## 8. 비행 테스트 (단계별 go / no-go)

장소: 사람·차 없는 넓은 잔디/흙 바닥(질감 필요), 바람 5 m/s 이하, GPS 3D fix 위성 ≥ 12, 배터리 만충, 지오펜스 설정, 조종기 켜짐, QGC 연결.

| 단계 | 내용 | 고도 | go 기준 | no-go면 |
|---|---|---|---|---|
| F1 | **수동 Stabilized 이륙·호버·착륙** (경험자) | 1~2 m | 떨림 없음, 모터 온도 정상, 10초 호버 유지 | 프롭 방향/밸런스, PID(기본값으로 X500은 보통 OK), 진동(Flight Review) |
| F2 | **Position 모드 호버** 30초, 전후좌우 1 m 이동 | 2 m | 위치 유지 ±0.5 m, 고도 ±0.3 m | 컴퍼스 간섭(회전 시 흔들림=toilet-bowl), GPS 품질 |
| F3 | **오프보드 호버** `mission:=hover target_altitude:=1.5 target_x:=0 target_y:=0` → 사람·프롭 확인 후 `ros2 param set /waypoint_navigator arm_approved true` → 중간에 RC로 Position 전환 | 1.5 m | 자동 이륙·호버, RC 전환 즉시 응답 | `COM_OBL_RC_ACT`, 오프보드 스트림(10 Hz) 확인 |
| F4 | **survey** 1장 촬영 `mission:=survey target_altitude:=3` (레이저는 별도 회차: `laser:=true laser_hardware_enable:=true laser_backend:=rpi`, 짧게, 사람 없는 방향) | 3 m | 사진 저장, `dist_bottom` 유효; 레이저 회차엔 `LASERS ON` 로그 후 착륙 시 OFF | AGL 경고(`range AGL … disagrees`)면 TF-Luna 마운트/빔 아래 부품 |
| F5 | **mosaic** 9장 = 20 × 10 m, `after_mosaic:=land` (기본 고도 6.25 m, 셀 간격 5.0 × 1.875 m; 지상 25 × 15 m 확보, 중간 발표 런은 `laser` 기본 false) | 6.25 m | 로그 `mosaic plan: altitude 6.25 m … overlap 50 % / 70 %`, 9장 전송, 대시보드 n/9 → 지도 20 × 10 m(3000 × 1500 px), 지면 질감이 이어짐 | 스티치 실패면 `diagnostics.json`: `insufficient_features`(지면 질감 부족·흔들림 → `camera_exposure_us`), `implausible footprint`(AGL). 셀에서 HOLD면 카메라(`ros2 topic hz`) |

각 비행 후: QGC → Analyze → Log Download → logs.px4.io(Flight Review)에서 진동·컴퍼스·배터리 확인. 배터리는 **용량의 70% 이상 쓰지 않기**(4000 mAh면 2800 mAh).

**레이저 비행 규칙**: 레이저가 켜지는 F4/F5에서는 비행 구역 전체와 그 너머(투영 방향)에 사람이 없고, 킬스위치 담당(본인)이 레이저를 끌 수 있는 거리에 있어야 합니다.

---

## 9. 시뮬 → 실기체 변경 요약

| 항목 | 시뮬 | 실기체 |
|---|---|---|
| PX4 | SITL, 에어프레임 `gz_x500_survey`(`SIM_GZ_EN_LIDAR 0`, `EKF2_HGT_REF 2`) | `px4_fmu-v6c`, 에어프레임 X500 V2, `config/px4_params_x500_real.params` (`EKF2_HGT_REF 1`) |
| 에이전트 | `udp4 -p 8888` | `serial --dev /dev/ttyAMA0 -b 921600` |
| 카메라 | ros_gz 브리지, hfov 1.2 rad 1280×960 | `survey_camera`(Picamera2, B0353 1920×1200, RELIABLE) + 캘리브레이션 YAML → `camera_info`; `camera_hfov_rad = 2·atan(W/2fx)` (기본 1.3495) |
| 격자 | 20×10 m, 고도 7.5 m(SITL 카메라), 간격 4.87×1.15 m | 20×10 m, 고도 **6.25 m**, 간격 5.0×1.875 m, 겹침 50/70 % (`mosaic_plan`) |
| 촬영 실패 | 3회 재시도 → HOLD (SITL에서 검증) | 같음 |
| 레인지 | 노드가 LaserScan→DistanceSensor 전달 | TF-Luna → Pixhawk TELEM3 (`SENS_TFMINI_CFG 103`) |
| 레이저 | RViz 마커만 | `laser_control`(`laser:=true`일 때만 실행, 서보는 `laser_servo_enable:=true`) + MOSFET + 서보(“Peripheral via Actuator Set”) |
| launch | `mosaic_mission.launch.py` (전부 한 PC) | Pi 5: `drone_onboard.launch.py` / 지상 PC: 웹앱 + `ground_station` |
| ROS 2 | Humble (Ubuntu 22.04) | Pi: Jazzy 소스 빌드 on Raspberry Pi OS Bookworm (5.1절) / 지상 PC: Humble |
| DDS | 로컬 | `cyclonedds_onboard.xml` / `cyclonedds_groundpc.xml` (유니캐스트 피어), 양쪽 CycloneDDS |
| 바닥 | `textured` 월드 | 질감 있는 실제 지면 |
| 허용오차 | `position_tolerance 0.3`, `reach_hold_time 1.0` | 0.5 / 2.0, `capture_settle_time 3.0` (GPS 위치 노이즈) — `drone_onboard.launch.py` 기본값 |

---

## 10. 트러블슈팅

| 증상 | 원인 | 조치 |
|---|---|---|
| 에이전트에 `session established`가 안 뜸 | TX/RX 미교차, 보레이트, `UXRCE_DDS_CFG`가 TELEM2가 아님, `dtparam=uart0=on` 누락, 시리얼 콘솔이 포트 점유 | 3.4절, 4.8절, 5.2/5.6절 |
| `ros2 topic list`에 `/fmu/out/…`는 있는데 노드가 “No /fmu/out/vehicle_status_v4 received” | 펌웨어·px4_msgs 버전 불일치(접미사 다름) | `ros2 topic list \| grep vehicle_status`로 실제 이름 확인 → 4.2절대로 같은 커밋 맞추기 |
| 지상 PC에서 Pi 토픽이 안 보임 | 도메인 ID, RMW가 양쪽 CycloneDDS인지, 인터페이스/피어 주소, 방화벽, (Humble↔Jazzy) 메시지 정의 불일치 | 6.2절. `ros2 doctor --report` |
| 암 거부 “No connection to the GCS” | `NAV_DLL_ACT`>0인데 QGC 미연결 | QGC(SiK) 켜기 또는 `NAV_DLL_ACT 0` |
| 암 거부 “Preflight Fail: …” 기타 | 컴퍼스/가속도 캘리브레이션, 세이프티 스위치, RC 미캘리브 | QGC 메시지가 정확한 이유 |
| 오프보드 진입 직후 Hold/Position으로 빠짐 | 셋포인트 10 Hz 스트림 끊김(UART 과부하·Pi CPU/발열 스로틀링) | 액티브 쿨러, 카메라 fps↓(`camera_fps:=2`), `COM_OF_LOSS_T` |
| `dist_bottom` 무효 | TF-Luna 포트/모드, `EKF2_RNG_CTRL`, 빔 아래 부품(0.19 m 같은 값) | 7장 6단계, 3.5절 |
| 사진이 안 찍힘(“Waiting for a camera frame” → `Capture attempt n/3 failed` → HOLD) | 카메라 노드 죽음, 토픽 이름, QoS 불일치, `rmem_max` 기본값(프레임 유실) | `ros2 topic hz`, `ros2 topic info -v /survey/camera/image`, 5.2절 5번 sysctl |
| 시동 전 `altitude … is too low/high for a 20 x 10 m mosaic` | 고도가 1.4절 범위(5.21~10 m) 밖 | `target_altitude` 조정 또는 `mosaic_plan`으로 확인 |
| `camera calibration … says hfov=… but the grid was planned with …` 경고 | 캘리브레이션 fx와 `camera_hfov_rad` 불일치 | 경고에 적힌 값으로 `camera_hfov_rad:=` 재실행 |
| 스티치 `insufficient_features` | 지면 질감 부족·모션 블러·너무 어두움 | 질감 있는 장소, `capture_settle_time`↑, 밝은 시간 |
| 스티치 `implausible footprint scale` | 어떤 프레임의 AGL이 엉뚱함 | 지상국 로그의 AGL 경고 → 센서 마운트 |
| 지도가 90°/180° 돌아감·거울상·색이 뒤바뀜 | 카메라 방향/채널 순서 가정 | 5.7절 방향 검증, `camera_rotate_deg`/`image_top_yaw_offset_rad`, `swap_rb` |
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
9. Raspberry Pi — Pi 5 문서(전원·UART·config.txt·카메라 커넥터), pinout.xyz, gpiozero 문서; Arducam — Pivariety Quick Start, AR0234/B0353 페이지(docs.arducam.com, 3.6 mm F/3.0, 120°(D)/90°(H)/75°(V)), Picamera2 매뉴얼
9b. ROS 2 Jazzy — Debian Bookworm 소스 설치, "ROS 2 on Raspberry Pi"
10. eProsima — Micro XRCE-DDS Agent
11. ROS 2 Humble — 설치, `camera_calibration`, CycloneDDS 네트워크 설정
12. 드론원스톱 민원서비스 — 기체 신고·비행 승인·공역

이 저장소 안: `README.md`(시뮬·토픽·QoS·모자이크·B0353 기하), `config/px4_params_x500_real.params`, `config/cyclonedds_*.xml`, `launch/drone_onboard.launch.py`, `drone_control/mosaic_plan.py`, `drone_control/camera_node.py`, `drone_control/laser_control.py`.
