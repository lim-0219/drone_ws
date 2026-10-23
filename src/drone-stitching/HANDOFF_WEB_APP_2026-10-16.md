# Drone Stitching 로컬 웹앱 인수인계

- 작성일: 2026-10-16
- 기준 프로젝트: `drone-stitching`
- 전제: 기존 USB 번들과 기존 설계 MD의 내용은 이미 전달되어 있다. 이 문서는 그 이후 추가된 **로컬 웹앱 작업과 검증 결과**만 기록한다.

## 1. 이번 작업의 목표와 범위

기존 9장 스티칭 엔진을 수정하지 않고, 사용자가 브라우저에서 이미지 9장을 선택하면 별도의 `지도 생성` 버튼 없이 자동으로 2D 지도를 만드는 테스트용 로컬 웹앱을 추가했다.

이번 버전에 포함된 범위는 다음과 같다.

- 이미지 9장 선택 또는 드래그 앤 드롭
- 파일명 자연 정렬 후 촬영 순서에 대응
- 9장이 모이면 자동 업로드 및 자동 스티칭
- 처리 중 2D 지도 영역에 `Map Scanning` 표시
- 완료된 `map.png`를 화면에 자동 표시
- 실패 원인 표시 및 다시 선택 기능
- 작업 상태 확인용 HTTP API
- 데스크톱과 모바일 폭에 대응하는 반응형 화면

이번 버전에 포함되지 않은 것은 다음과 같다.

- 드론/Raspberry Pi에서 Wi-Fi로 이미지를 한 장씩 받는 기능
- 실시간 카메라 영상
- 실시간 GPS 및 비행 상태
- 위험물 탐지와 라벨링
- 여러 드론 또는 여러 촬영 세션의 동시 처리

화면에 보이는 연결 상태, GPS, 카메라, 위험물 관련 요소는 현재 UI 자리 표시용이며 실제 장치 데이터와 연결되어 있지 않다.

## 2. 기존 USB 번들 이후 변경된 파일

### 새로 추가된 소스와 문서

| 파일 | 역할 |
|---|---|
| `src/drone_stitch/webapp.py` | FastAPI 서버, 업로드 검증, 작업 큐, 스티칭 실행, 상태/지도 API |
| `web/static/index.html` | 관제 화면 구조 |
| `web/static/styles.css` | 화면 디자인과 반응형 레이아웃 |
| `web/static/app.js` | 9장 선택, 자연 정렬, 자동 시작, 상태 조회, 지도 표시 |
| `tests/test_webapp.py` | 업로드/API/실패 처리/입력 검증 테스트 |
| `WEB_APP.md` | 웹앱 간단 사용 설명 |
| `HANDOFF_WEB_APP_2026-10-16.md` | 이 인수인계 문서 |
| `scripts/start_webapp.py` | 격리된 가상환경으로 서버를 시작하고 브라우저를 여는 실행 도우미 |
| `scripts/build_portable_bundle.py` | 배포 파일만 선별하고 무결성 목록과 ZIP을 만드는 도우미 |

### 수정된 기존 파일

| 파일 | 변경 내용 |
|---|---|
| `pyproject.toml` | FastAPI, Uvicorn, multipart 의존성과 `drone-stitch-web` 실행 명령 추가 |
| `requirements.txt` | `fastapi==0.115.12`, `uvicorn==0.34.2`, `python-multipart==0.0.20` 추가 |
| `requirements-dev.txt` | API 테스트용 `httpx==0.28.1` 추가 |
| `scripts/setup_env.py` | ROS/Python 환경변수 격리, 설치 실패 시 불완전한 `.venv` 정리 |
| `scripts/run_demo.py` | 격리 모드(`-I`)로 CLI 데모 실행 |
| `START_HERE.md` | 새 컴퓨터용 웹앱 설치·실행 절차 추가 |

기존 스티칭 핵심 모듈은 그대로 사용한다. `.venv`, `__pycache__`, `.pytest_cache`, `src/*.egg-info`는 설치나 실행 중 다시 생기는 생성물이므로 소스 변경으로 보지 않는다.

## 3. 촬영 순서와 3×3 위치 대응

서버가 기대하는 정확한 업로드 순서는 다음과 같다.

```text
5 → 4 → 1 → 2 → 3 → 6 → 9 → 8 → 7
```

즉, 업로드 배열의 위치와 내부 프레임 ID는 다음처럼 대응한다.

| 업로드 순번 | 내부 프레임 ID |
|---:|---:|
| 1 | 5 |
| 2 | 4 |
| 3 | 1 |
| 4 | 2 |
| 5 | 3 |
| 6 | 6 |
| 7 | 9 |
| 8 | 8 |
| 9 | 7 |

브라우저 화면은 사용자가 고른 파일을 **파일명 자연 정렬**한 뒤 이 순서에 넣는다. 예를 들어 `capture_2.jpg`가 `capture_10.jpg`보다 먼저 온다. 실제 촬영 파일을 시험할 때는 촬영 순서를 알 수 있도록 `capture_01`부터 `capture_09`까지 이름을 붙이는 것이 안전하다. 파일 선택 창에서 클릭한 순서 자체는 촬영 순서로 사용하지 않는다.

## 4. UI 상태 흐름

```text
대기(0/9)
  → 수신 중(1~8/9)
  → 9장 준비 완료 / 자동 시작
  → 서버 전송 중
  → queued(처리 대기)
  → scanning(Map Scanning)
  → completed(2D 지도 표시)
```

처리에 실패하면 `failed` 상태와 오류 내용을 표시하고, 사용자가 9장을 다시 선택할 수 있다. 9장 준비 후 약 450 ms 뒤 자동 요청하므로 별도의 지도 생성 버튼은 없다. 화면은 약 1.1초 간격으로 서버 상태를 확인하며 최대 10분 동안 기다린다.

서버는 OpenCV의 프로세스 전역 설정 충돌을 피하기 위해 스티칭 작업자를 1개만 사용한다. 여러 요청이 들어오면 한 번에 하나씩 처리한다.

## 5. 새 컴퓨터에서 설치

### 권장 환경

- Ubuntu 22.04
- Python 3.10 이상
- 최초 Python 패키지 설치 시 인터넷 연결 필요
- Chromium, Chrome 또는 Firefox 계열 브라우저

압축 파일을 원하는 위치에 **완전히** 푼 뒤, 압축 해제된 `drone-stitching` 폴더에서 실행한다. ZIP 내부에서 직접 실행하지 않는다.

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip

cd /압축을/푼/경로/drone-stitching
python3 scripts/verify_bundle.py
python3 scripts/setup_env.py
```

설치 상태 확인:

```bash
.venv/bin/python -I -m pip --isolated check
```

정상이면 다음 문구가 나온다.

```text
No broken requirements found.
```

## 6. 실행

프로젝트 루트에서 다음 명령을 실행한다.

```bash
cd /압축을/푼/경로/drone-stitching
python3 scripts/start_webapp.py
```

브라우저에서 다음 주소를 연다.

```text
http://127.0.0.1:8000
```

서버 종료는 서버를 실행한 터미널에서 `Ctrl+C`를 누른다.

개발 중 소스 변경을 자동 반영하려면 다음과 같이 실행할 수 있다.

```bash
.venv/bin/python -I -m drone_stitch.webapp --reload
```

다른 장치의 브라우저에서 같은 PC의 화면에 접속해야 할 때만, 신뢰할 수 있는 로컬 네트워크에서 다음처럼 연다.

```bash
python3 scripts/start_webapp.py --host 0.0.0.0
```

이 경우 접속 주소는 `http://서버_PC의_IP:8000`이다. 현재 인증과 HTTPS가 없으므로 인터넷에 직접 공개하면 안 된다.

## 7. 테스트용 이미지 9장 선택 방법

기존 번들에 포함된 다음 폴더를 사용한다.

```text
runs/verified_input/images/
```

아래 9개 파일을 **모두 한 번에 선택**하거나 화면의 선택 영역에 함께 드롭한다.

```text
capture_01_id5.png
capture_02_id4.png
capture_03_id1.png
capture_04_id2.png
capture_05_id3.png
capture_06_id6.png
capture_07_id9.png
capture_08_id8.png
capture_09_id7.png
```

파일명이 자연 정렬되면서 정확한 촬영 순서 `5 → 4 → 1 → 2 → 3 → 6 → 9 → 8 → 7`이 된다. 9번째 이미지가 준비되면 자동으로 `Map Scanning`으로 넘어가고, 완료 후 2D 지도가 표시된다.

작업별 파일은 다음 위치에 남는다.

```text
runs/web/<job-id>/
  input/
    capture_01.<확장자>
    ...
    capture_09.<확장자>
    manifest.json
  output/
    map.png
    diagnostics.json
    coverage.png
    overview.png
    transforms.json
    valid_mask.png
```

서버를 재시작해도 위 파일은 디스크에 남지만, 현재 작업 상태 목록은 메모리에만 있으므로 이전 작업 ID를 API로 다시 조회할 수는 없다.

## 8. 자동 테스트

ROS 환경 변수가 섞이지 않도록 다음 명령으로 실행한다.

```bash
cd /압축을/푼/경로/drone-stitching
.venv/bin/python -I -m pytest -q
```

2026-10-16 기준 결과:

```text
35 passed
```

Starlette TestClient와 AnyIO 사이의 `DeprecationWarning` 1건이 출력될 수 있지만 테스트 실패는 아니다.

## 9. HTTP API

### `GET /api/health`

서버 상태, 상태별 작업 수, 작업자 제한 수를 반환한다.

### `POST /api/jobs`

`multipart/form-data`의 `images` 필드에 이미지 정확히 9장을 순서대로 전달한다. 정상 접수 시 HTTP `202`와 작업 ID를 반환하고 스티칭은 자동으로 시작된다.

허용 확장자:

```text
.jpg .jpeg .png .webp .bmp .tif .tiff
```

제한은 파일당 25 MiB, 디코딩 후 50 메가픽셀, 최소 한 변 64 px이다.

```bash
curl \
  -F images=@runs/verified_input/images/capture_01_id5.png \
  -F images=@runs/verified_input/images/capture_02_id4.png \
  -F images=@runs/verified_input/images/capture_03_id1.png \
  -F images=@runs/verified_input/images/capture_04_id2.png \
  -F images=@runs/verified_input/images/capture_05_id3.png \
  -F images=@runs/verified_input/images/capture_06_id6.png \
  -F images=@runs/verified_input/images/capture_07_id9.png \
  -F images=@runs/verified_input/images/capture_08_id8.png \
  -F images=@runs/verified_input/images/capture_09_id7.png \
  http://127.0.0.1:8000/api/jobs
```

### `GET /api/jobs/{id}`

작업 상태를 반환한다. 상태 값은 다음 네 가지다.

- `queued`: 작업 대기
- `scanning`: 스티칭 실행 중
- `completed`: 지도 생성 완료, 응답에 `map_url` 포함
- `failed`: 실패, 응답에 `error` 포함

### `GET /api/jobs/{id}/map`

완료된 작업의 PNG 지도를 반환한다. 아직 완료되지 않았으면 HTTP `409`를 반환한다.

브라우저 UI와 API는 같은 백엔드를 사용한다. 앞으로 Wi-Fi 수신부를 추가할 때도 최종 9장을 같은 작업 생성 경로에 연결하면 스티칭 엔진과 화면을 재사용할 수 있다.

## 10. 검증 결과

2026-10-16에 확인한 결과는 다음과 같다.

- 전체 자동 테스트: `35 passed`
- 검증용 합성 이미지 9장을 실제 웹 API 경로로 전달해 자동 합성 성공
- API 합성 소요 시간: 약 `4.48초` (`4.476131713초`)
- 생성 지도 크기: `1062 × 2009 px`
- 검증 당시 성공 작업 ID: `d04fa1a25c9a4f66ad91a5d15978158a`

위 작업 ID와 생성 지도는 제작 PC의 검증 기록이다. 중복 용량을 피하기 위해 새 휴대용 ZIP에서는 `runs/web/`를 제외했다. 새 컴퓨터에서는 동봉된 `runs/verified_input/images/` 9장을 선택하면 새 작업 ID와 결과가 생성된다.

기본 웹 매니페스트는 현재 테스트 가정값인 촬영 범위 `5 × 10 m`, 기준 지상고 `5 m`, 격자 간격 `2.5 × 5 m`를 사용한다. 실제 카메라와 비행 경로에 맞춘 값은 아니다.

## 11. 알려진 주의점

### 파일명 자연 정렬

웹 화면은 선택 순서나 파일 수정 시간이 아니라 파일명 자연 정렬을 우선 사용한다. 실제 데이터 파일명이 촬영 순서를 보장하지 않으면 잘못된 프레임 ID에 연결될 수 있다. Wi-Fi 수신 구현에서는 파일명 추측 대신 명시적인 `capture_index` 또는 프레임 ID를 보내야 한다.

### ROS의 `PYTHONPATH`

ROS 2 환경을 `source`한 터미널에서는 시스템 ROS 패키지가 가상환경 안으로 보일 수 있다. 그 상태의 `pip check`는 이 프로젝트와 무관하게 다음과 비슷한 잡음을 낼 수 있고, pytest가 ROS용 플러그인을 잘못 불러올 수도 있다.

```text
launch-ros ... requires pyyaml, which is not installed
```

그래서 설치 및 실행 도우미는 `PYTHONPATH`와 `PYTHONHOME`을 제거하고 가상환경 Python을 격리 모드로 실행한다. 수동 검증 명령에서도 `.venv/bin/python -I`를 사용한다. 새 터미널에서 ROS 환경을 불러오지 않고 실행해도 된다.

### Sheffield 공개 실영상의 경계 실패

공개 Sheffield 9장 데이터를 새 환경에서 SIFT로 재실행한 한 검증은 최적화 후 `(3, 6)` 경계의 중앙 오차가 `4.059333 px`로 측정되어 허용 기준 `4.0 px`를 약 `0.059 px` 초과했다. 따라서 파이프라인이 의도대로 품질 실패 처리했다.

```text
ValueError: Inconsistent overlap constraints after optimization: [(3, 6)]
```

이는 웹 API 고장이나 9장 자동 실행 실패가 아니라 엄격한 품질 임계값의 경계 사례다. 실제 카메라 영상을 확보한 뒤 렌즈 보정과 촬영 중첩률을 먼저 확인하고, 충분한 근거가 있을 때만 품질 임계값을 조정해야 한다.

## 12. 다음 작업

### 1) Wi-Fi 개별 이미지 수신 세션/API

> **구현됨 (2026-10-23).** `POST /api/sessions/{session_id}/frames` 와 `GET /api/sessions/{id}`,
> `GET /api/sessions/latest`, 화면의 드론 세션 자동 표시. 사용법은 `WEB_APP.md`의
> "드론 세션 API" 절, 클라이언트 예는 `drone_control` ROS 패키지의 `ground_station` 노드.
> 아래 권장 구조는 설계 당시 기록으로 남겨 둔다.

현재 API는 9장을 한 요청에 함께 받는다. 실제 드론 연동에서는 촬영 1장마다 전송할 수 있도록 별도 수신 계층을 추가한다.

권장 구조:

1. 비행 또는 스캔 시작 시 고유한 `session_id` 생성
2. 각 이미지에 `session_id`, `capture_index` 또는 프레임 ID, 촬영 시각, 파일 체크섬을 함께 전송
3. 같은 세션의 중복 전송은 체크섬과 순번으로 안전하게 무시하거나 교체
4. 서버가 세션별 `n/9` 상태를 UI에 제공
5. 정확한 9장이 모이면 기존 스티칭 작업을 자동 시작
6. 누락, 순서 오류, 세션 시간 초과, 재전송 상태를 명시적으로 기록

서로 다른 비행의 사진이 섞이지 않도록 세션 구분을 먼저 설계해야 한다. 파일명만으로 순서를 판단하면 안 된다.

### 2) 실제 카메라 보정과 비행 조건 반영

- 실제 렌즈 왜곡 계수로 입력 영상을 보정
- 실측 고도, 화각, 이미지 방향으로 `footprint_m`와 yaw 오프셋 결정
- 3×3 촬영 간격과 전후/좌우 중첩률 확인
- 실제 영상 여러 세트로 특징점 수, 매칭, RANSAC, 품질 임계값 재검증
- 필요한 경우 GPS/자세 메타데이터를 Wi-Fi 전송 규격에 포함

### 3) 운영용 상태 저장

현재 작업 목록은 서버 메모리에 있다. 운영 버전에서는 SQLite 같은 영속 저장소를 사용해 서버 재시작 후에도 세션과 작업 이력을 복구하도록 한다.

## 13. 설치형 데스크톱 앱으로 전환 가능 여부

가능하다. 현재 구조는 스티칭 엔진, FastAPI 백엔드, HTML/CSS/JavaScript 화면이 분리되어 있으므로 UI와 엔진을 다시 만들지 않고 설치형 앱으로 포장할 수 있다.

- 가장 단순한 방향: `pywebview` 창 안에 현재 로컬 웹 화면을 띄우고 Python 서버를 앱 시작/종료와 함께 관리
- 더 정교한 방향: Tauri 화면에서 현재 웹 정적 파일을 사용하고 Python/FastAPI를 sidecar로 패키징

설치형으로 만들 때는 Python/OpenCV 런타임 포함, 출력 데이터 저장 위치, 로컬 포트 충돌, 앱 종료 시 서버 정리, 로그 수집을 추가로 처리해야 한다. 먼저 현재 로컬 웹앱으로 실제 9장과 Wi-Fi 수신 규격을 확정한 뒤 포장하는 편이 변경 비용이 적다.
