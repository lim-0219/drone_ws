# 2026-11-03 비행 체크리스트 (3 m 버전)

4 m 버전과 다른 곳은 비행 2·3의 고도, 9장 격자 크기, 비행 구역뿐입니다.

순서: **수동 조종 → 자동 3 m 호버 → 자동 3 m 9장 촬영 + 영상 저장**

기체: Holybro X500 V2, Pixhawk 6C, Raspberry Pi 5, Arducam B0353, ELRS(RadioMaster Pocket).
포트: TELEM1 텔레메트리, TELEM2 Pi, TELEM3 ELRS 수신기, GPS1 GPS. 거리센서는 오늘 꺼 둠.

명령의 `[노트북]`, `[Pi]`는 어디서 치는지를 뜻합니다. Pi 명령은 노트북에서 `ssh pi@drone-pi.local`(비밀번호 0219)로 접속해서 칩니다.

---

## A. 출발 전 (집, 프롭 없음)

### A1. 코드를 Pi로 보내기
오늘 코드가 바뀌었습니다. 제자리 호버, 영상 녹화, `forward_target.sh`가 새로 들어갔습니다. Pi와 노트북이 같은 Wi-Fi에 있어야 합니다.

```bash
# [노트북]
rsync -av --exclude build --exclude install --exclude log --exclude .git \
  --exclude src/drone-stitching --exclude src/ros_gz --exclude src/px4_ros_com \
  ~/drone_ws/ pi@drone-pi.local:~/drone_ws/
```
```bash
# [Pi]
cd ~/drone_ws && colcon build --packages-select drone_control && source install/setup.bash
grep -q i2c-dev /etc/modules || echo i2c-dev | sudo tee -a /etc/modules
python3 -c "import av, picamera2; print('pyav', av.__version__)"   # 에러면: sudo apt install -y python3-av
```

### A2. 노트북 수신 버퍼 (한 번만, 사진 전송용)
```bash
# [노트북]
printf 'net.core.rmem_max=2147483647\nnet.core.rmem_default=2147483647\n' | sudo tee /etc/sysctl.d/60-ros2-dds.conf
sudo sysctl --system | grep rmem
```

### A3. 영상 녹화 시험 (10초, 비행 없음)
**영상 기능은 Pi에서 아직 한 번도 돌려 보지 않았습니다. 꼭 집에서 먼저 확인하세요.**

```bash
# [노트북] 노트북 주소 확인 후 수신 시작
ip -br addr show wlp0s20f3
mkdir -p ~/flight_videos
ffmpeg -fflags nobuffer -i udp://@:5000 -c copy ~/flight_videos/test_$(date +%H%M%S).ts
```
```bash
# [Pi] <노트북IP>는 위에서 본 주소
ros2 run drone_control survey_camera --ros-args -p fps:=3 \
  -p video_dir:=~/flight_videos -p video_udp:=udp://<노트북IP>:5000
```
- [ ] Pi 로그에 `video: 960x600 @ 15 fps H.264 ... -> /home/pi/flight_videos/flight_....ts, udp://...` 가 나온다
- [ ] 노트북 ffmpeg에 `frame=` 숫자가 올라간다
- [ ] 10초 뒤 Pi에서 Ctrl+C, 노트북에서 `q`. 노트북의 `.ts` 파일이 재생된다 (`ffplay 파일명`)

`video recording NOT started` 에러가 나면 A1의 `python3-av` 줄을 확인하세요. 이 경우에도 9장 촬영 자체는 됩니다.

### A4. QGC 설정 (배터리 연결, 프롭 없음)
`~/QGroundControl.AppImage`
- [ ] **Radio**: 스틱을 움직이면 막대가 따라 움직임 → Calibrate
- [ ] **Flight Modes**: SB(3단) = Mode 채널 → Flight Mode 1 **Stabilized** / 4 **Position** / 6 **Land**. SD(2단) = **Kill switch**
- [ ] SB를 움직일 때 왼쪽 위 모드 표시가 바뀐다. SD를 켜면 킬 스위치 경고가 뜬다
- [ ] **Power**: 셀 4, 실제 전압과 0.2 V 이내. 일반 LiPo면 Full Voltage를 4.20으로 (LiHV면 4.35)
- [ ] **Power → ESC Calibration**
- [ ] **Sensors → Level Horizon** (기체를 평평한 바닥에)
- [ ] **Actuators → 모터 1~4 위치와 방향** (1 앞오른쪽 반시계, 2 뒤왼쪽 반시계, 3 앞왼쪽 시계, 4 뒤오른쪽 시계)
- [ ] 조종기로 시동 (스로틀 아래 + 요 오른쪽 2초) → 4개가 천천히 돈다 → **SD(킬 스위치)로 즉시 멈춘다** → SD 원위치 → 시동 끄기 (스로틀 아래 + 요 왼쪽)

### A5. 챙길 것
- [ ] 배터리 2개 만충 (3300 mAh 하나로 전부는 부족)
- [ ] 프롭 4개 + 여분, 프롭 공구
- [ ] 노트북 충전, 텔레메트리 USB 모듈
- [ ] iPhone 핫스팟: 설정 → 개인용 핫스팟 → **호환성 최대화 켜기**
- [ ] 조종기 충전, ELRS 바인딩 상태 (수신기 LED 계속 켜짐)

---

## B. 현장 도착 (프롭 없음)

### B1. 네트워크
1. iPhone 핫스팟을 켜고 핫스팟 화면을 열어 둔다.
2. 노트북을 핫스팟에 연결하고, 기체 배터리를 꽂는다. Pi 부팅에 1~2분 걸린다.
3. 주소를 확인한다.
```bash
# [노트북]
ip -br addr show wlp0s20f3        # 기대: 172.20.10.2
ping -c 2 drone-pi.local          # 기대: 172.20.10.3
```
**주소가 다르면** DDS 설정 두 개를 고친다. 아래 `<노트북IP>`, `<PiIP>`에 실제 값을 넣는다.
```bash
# [노트북]
sed -i 's|<Peer address="172.20.10.3"/>|<Peer address="<PiIP>"/>|' ~/drone_ws/src/drone_control/config/cyclonedds_groundpc.xml
cd ~/drone_ws && colcon build --packages-select drone_control
# [Pi]
sed -i 's|<Peer address="172.20.10.2"/>|<Peer address="<노트북IP>"/>|' ~/drone_ws/src/drone_control/config/cyclonedds_onboard.xml
cd ~/drone_ws && colcon build --packages-select drone_control
```
영상 전송 주소(`video_udp`)도 `<노트북IP>`로 바꿔서 친다.

### B2. QGC (텔레메트리 연결)
- [ ] **Sensors → Compass**: 차와 철 구조물에서 10 m 이상 떨어져서
- [ ] 상단 GPS **3D fix, 위성 10개 이상**
- [ ] 배터리 전압이 15.2 V 이상
- [ ] 프리플라이트 경고 없음
- [ ] 모드 스위치 **Position**, 킬 스위치 **꺼짐**

### B3. Pi ↔ Pixhawk
```bash
# [Pi 창 1] 비행이 끝날 때까지 계속 켜 둔다
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```
```bash
# [Pi 창 2]
ros2 topic echo /fmu/out/vehicle_status_v4 --once | grep -E "arming_state|nav_state"
rpicam-hello --list-cameras       # arducam-pivariety 1920x1200 이 보여야 함
```

### B4. 프롭 장착
- [ ] 모터 방향에 맞는 CW/CCW 프롭 (프롭 표시 확인)
- [ ] 단단히 조임

---

## C. 비행 1: 수동 조종 (조종 경험자)

비행 구역: 사람 없음, 바람 5 m/s 이하.

1. **Stabilized**: 1~2 m 이륙, 10초 호버, 착륙
2. **Position**: 2 m 이륙, 30초 호버, 전후좌우 조금씩 이동, 착륙

**계속 진행해도 되는 조건**
- [ ] 떨림이 없고 제자리를 유지한다 (±0.5 m)
- [ ] 제자리에서 빙글빙글 돌지 않는다. 돌면 컴퍼스 문제이니 중단
- [ ] 착륙 후 모터가 손으로 만질 수 있을 정도로만 따뜻하다

배터리 40 %가 되면 착륙하고 교체한다.

---

## D. 비행 2: 자동, 제자리 3 m 호버 후 착륙

기체를 이륙할 자리에 두고, 사람은 5 m 이상 떨어진다.
```bash
# [Pi 창 2]
ARGS=$(~/drone_ws/src/drone_control/tools/forward_target.sh 0)
ros2 launch drone_control drone_onboard.launch.py mission:=land target_altitude:=3.0 \
    reach_hold_time:=10 camera:=false $ARGS
```
- [ ] 로그 `target (N=… E=… alt=3.00m …)`의 N, E가 `forward_target.sh`가 출력한 현재 위치와 같다
- [ ] `ARM gate closed … waiting` 이 10초마다 나오고 아무 일도 일어나지 않는다

준비가 되면 승인한다.
```bash
# [Pi 창 3]
ros2 param set /waypoint_navigator arm_approved true
```
**예상 동작**: 시동 → 3 m 상승 → 10초 호버 → 착륙 → 자동 시동 꺼짐 (`STATE LANDING -> LANDED`)

**이상하면** SB를 **Position**으로 옮긴다. 그 순간 자동 비행이 끝나고 조종기로 넘어온다. 더 급하면 SD(킬)를 쓴다. 킬은 모터가 즉시 멈춰 떨어지니 마지막 수단이다.

끝나면 Pi 창 2에서 Ctrl+C.

---

## E. 비행 3: 자동, 제자리 3 m 9장 촬영 + 영상 저장

**배터리: 이륙 전 60 % 이상.** 비행 구역: 이륙 지점을 중심으로, 기체가 바라보는 방향으로 **앞뒤 5.5 m**, 그 옆으로 **좌우 9 m**보다 넓게 비어 있어야 한다.

### E1. 노트북: 영상 저장 시작
```bash
# [노트북 창 1]
mkdir -p ~/flight_videos
ffmpeg -fflags nobuffer -i udp://@:5000 -c copy ~/flight_videos/flight3_$(date +%H%M%S).ts
```

### E2. 노트북: 사진 수신과 스티칭
```bash
# [노트북 창 2]
source /opt/ros/humble/setup.bash && source ~/drone_ws/install/setup.bash
export ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/drone_ws/install/drone_control/share/drone_control/config/cyclonedds_groundpc.xml
ros2 run drone_control ground_station
```
- [ ] `listening on /survey/frames` 가 나온다

### E3. Pi: 미션 실행
카메라를 쓰는 다른 프로그램(`rpicam-vid`, `rpicam-hello`)이 꺼져 있어야 한다.
```bash
# [Pi 창 2]
ARGS=$(~/drone_ws/src/drone_control/tools/forward_target.sh 0)
ros2 launch drone_control drone_onboard.launch.py mission:=mosaic target_altitude:=3.0 \
    mosaic_extent_x:=9 mosaic_extent_y:=5.5 after_mosaic:=land \
    video_dir:=~/flight_videos video_udp:=udp://172.20.10.2:5000 $ARGS
```
승인 전에 확인한다.
- [ ] `mosaic plan: altitude 3.00 m … overlap 56 % / 58 %, mosaic 9.0 x 5.5 m`
- [ ] `video: 960x600 @ 15 fps H.264 … -> /home/pi/flight_videos/flight_….ts, udp://172.20.10.2:5000`
- [ ] 노트북 창 1의 ffmpeg `frame=` 숫자가 올라간다

```bash
# [Pi 창 3]
ros2 param set /waypoint_navigator arm_approved true
```
**예상 동작**: 시동 → 3 m → 9칸을 돌며 칸마다 3초 정지 후 촬영 → 착륙. 비행 약 1.5~2분.
- [ ] 노트북 창 2: `frame 1/9 … 9/9` → `stitch OK … map.png`
- [ ] 한 칸에서 멈춰 계속 떠 있으면(`HOLD`) 카메라 문제다. SB → Position → 수동 착륙

### E4. 착륙 후
1. 시동이 꺼진 걸 확인하고 Pi 창 2에서 Ctrl+C (녹화 종료), 노트북 창 1에서 `q`.
2. Pi에 저장된 원본 영상(끊김 없음)을 노트북으로 가져온다.
```bash
# [노트북]
scp 'pi@drone-pi.local:~/flight_videos/*.ts' ~/flight_videos/
ls -l ~/flight_videos/
```
3. mp4가 필요하면 변환한다: `ffmpeg -i 파일.ts -c copy 파일.mp4`

**결과물 위치 (노트북)**
| 무엇 | 경로 |
|---|---|
| 비행 영상 (Wi-Fi 실시간) | `~/flight_videos/flight3_*.ts` |
| 비행 영상 (Pi 원본) | `~/flight_videos/flight_*.ts` |
| 스티칭 지도 | `~/drone_survey_sessions/session_*/stitch_01/map.png` |
| 사진 9장 | `~/drone_survey_sessions/session_*/images/` (Pi에도 `~/drone_survey_photos`) |

**스티칭이 실패해도 사진 9장은 남습니다.** 사진은 촬영 순간 Pi(`~/drone_survey_photos`)에 저장되고, 노트북에도 한 장씩 도착하는 즉시 `images/`에 저장됩니다. 스티칭은 9장이 다 모인 뒤 따로 돌기 때문에, 실패해도 사진은 지워지지 않습니다. 실패 원인은 `stitch_01/diagnostics.json`에 남습니다. 나중에 노트북에서 다시 스티칭하려면:
```bash
# [노트북] 가장 최근 세션을 다시 스티칭
S=$(ls -d ~/drone_survey_sessions/session_* | tail -1)
cd ~/drone_ws/src/drone-stitching && env -u PYTHONPATH .venv/bin/python -I -m drone_stitch stitch \
    $S/manifest.json --output $S/stitch_manual \
    --config ~/drone_ws/install/drone_control/share/drone_control/config/stitch_config.json
```
사진이 노트북에 다 오지 못했으면(Wi-Fi 끊김) Pi에서 가져옵니다: `scp 'pi@drone-pi.local:~/drone_survey_photos/*' ~/flight_photos/`

**3 m 9장 결과 크기 (계산값)**
| 항목 | 값 |
|---|---|
| 사진 1장이 덮는 땅 | 4.8 × 3.0 m |
| 격자 간격 | 좌우 2.1 m, 앞뒤 1.25 m |
| 겹침 | 56 % / 58 % |
| 스티칭 지도 | 약 9 × 5.5 m, 약 1350 × 825 px (150 px/m) |
| 사진 원본 해상도 | 픽셀당 2.5 mm (4 m보다 선명) |

**3 m에서 알아둘 점**
- 격자 간격이 1.25 m로 작아서, 바람에 위치가 0.5 m쯤 흔들리면 겹침이 눈에 띄게 줄어듭니다. 바람이 3 m/s를 넘으면 4 m 버전이 더 안전합니다.
- 프롭 바람이 바닥의 먼지와 풀을 날려 사진이 흐려질 수 있습니다. 모래나 먼지가 많은 곳은 피하세요.
- 9장 촬영 범위가 4 m 버전보다 좁아 필요한 공터도 작습니다.

---

## 비상 절차
| 상황 | 할 일 |
|---|---|
| 자동 비행이 이상함 | SB → **Position** (조종기로 넘어옴) |
| 조종 불능, 사람 쪽으로 감 | SD **킬 스위치** (모터 즉시 정지, 추락) |
| Pi 프로그램이 멈춤 | 1초 뒤 PX4가 자동으로 제자리 정지(Hold). SB → Position으로 받아서 착륙 |
| Wi-Fi 끊김 | 비행에는 영향 없음 (Pi↔Pixhawk는 선 연결). 사진 전송과 실시간 영상만 멈추고, Pi 원본 영상은 계속 녹화됨 |
| 조종기 신호 끊김 | PX4가 자동 귀환(RTL) |
| 배터리 30 % | 즉시 착륙 |
