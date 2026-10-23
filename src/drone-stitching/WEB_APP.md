# 로컬 테스트 웹앱

이 웹앱은 이미지 9장을 받은 순서대로 `5 → 4 → 1 → 2 → 3 → 6 → 9 → 8 → 7` 위치에 대응시킵니다. 업로드가 검증되면 별도의 버튼 없이 자동으로 스티칭을 시작하며, 한 번에 한 작업만 처리합니다. 실시간 카메라·GPS·Wi-Fi 수신은 이 테스트 버전에 포함하지 않았습니다.

## 설치와 실행

압축을 완전히 푼 프로젝트 루트에서 설치 도우미를 한 번 실행한 뒤 웹앱을 시작합니다. 설치할 때만 인터넷이 필요합니다.

```bash
python3 scripts/setup_env.py
python3 scripts/start_webapp.py
```

준비가 끝나면 브라우저가 <http://127.0.0.1:8000>을 자동으로 엽니다. 자동 실행을 원하지 않으면 `--no-browser`를 사용합니다. 다른 장치에서 접속하게 할 때만 신뢰할 수 있는 로컬 네트워크에서 `python3 scripts/start_webapp.py --host 0.0.0.0`을 실행하세요. 개발 중 소스 변경을 자동 반영하려면 `.venv/bin/python -I -m drone_stitch.webapp --reload`를 사용합니다. 프로덕션 작업자 수는 반드시 기본값 1로 유지해야 합니다.

Windows에서는 위 두 명령의 `python3` 대신 `py -3.12`를 사용합니다. 실행 도우미는 ROS나 다른 개발 환경의 `PYTHONPATH`가 가상 환경에 섞이지 않게 처리하므로 활성화 명령은 필요하지 않습니다.

작업 파일은 다음처럼 생성됩니다.

```text
runs/web/<job-id>/
  input/
    capture_01.<ext>
    ...
    capture_09.<ext>
    manifest.json
  output/
    map.png
    diagnostics.json
    ...
```

각 파일은 최대 25 MiB, 디코딩 후 최대 50 메가픽셀이며 JPEG, PNG, WebP, BMP, TIFF를 허용합니다. 원래 파일명은 저장 경로에 사용하지 않습니다.

## API

- `POST /api/jobs`: multipart의 `images` 필드로 정확히 9개를 촬영 순서대로 전송합니다. `202`와 작업 ID를 반환합니다.
- `GET /api/jobs/{id}`: `queued`, `scanning`, `completed`, `failed` 중 현재 상태를 반환합니다.
- `GET /api/jobs/{id}/map`: 완료된 작업의 PNG 지도를 반환합니다.
- `GET /api/health`: 서버와 단일 작업자 상태를 확인합니다.

간단한 API 호출 예시는 다음과 같습니다.

```bash
curl -F images=@capture_01.jpg \
     -F images=@capture_02.jpg \
     -F images=@capture_03.jpg \
     -F images=@capture_04.jpg \
     -F images=@capture_05.jpg \
     -F images=@capture_06.jpg \
     -F images=@capture_07.jpg \
     -F images=@capture_08.jpg \
     -F images=@capture_09.jpg \
     http://127.0.0.1:8000/api/jobs
```

기본 촬영 범위는 `[5, 10] m`, 기준 지상고는 `5 m`, 격자 간격은 `[2.5, 5] m`입니다. 이는 테스트 가정값이므로 실제 카메라와 비행 조건을 측정한 뒤 조정해야 합니다.

## 드론 세션 API: 한 장씩 받기 (2026-10-23 추가)

드론(또는 지상국 중계 노드)이 촬영할 때마다 사진 1장을 보내는 경로입니다. 브라우저 업로드와 같은 스티칭 작업 경로를 재사용하며, 9장이 모이면 자동으로 시작합니다. `drone_control` ROS 패키지의 `ground_station` 노드가 이 API의 클라이언트입니다.

- `POST /api/sessions/{session_id}/frames` (multipart): `image` 파일 1개 + `metadata` JSON 문자열.
  `session_id`는 비행 1회를 뜻하며 영문·숫자·`.`·`_`·`-`만 허용합니다.
- `GET /api/sessions/{session_id}`: `received`/`total`/`missing`/`frames`와 상태(`collecting` → `queued` → `scanning` → `completed`|`failed`), 완료 시 `job_id`, `map_url`, `output_dir`.
- `GET /api/sessions/latest`: 가장 최근에 갱신된 세션. 브라우저 화면이 유휴 상태일 때 이 값을 1.1초마다 읽어 드론 세션의 `n/9`, Map Scanning, 완성 지도를 자동으로 표시합니다. 사용자가 파일을 직접 고르면 수동 업로드 흐름이 우선합니다.

`metadata` 예시:

```json
{
  "sequence": 3, "grid_id": 1, "total": 9,
  "timestamp_s": 1760600000.25,
  "ned_m": [2.37, -3.43, -5.01], "yaw_rad": 0.0, "agl_m": 5.0,
  "camera": {"footprint_m": [6.82, 5.11], "reference_agl_m": 4.98, "image_top_yaw_offset_rad": 0.0,
             "calibration": {"image_size": [1280, 960], "K": [[935.6, 0, 639.5], [0, 935.6, 479.5], [0, 0, 1]],
                             "distortion": [0, 0, 0, 0, 0]}},
  "grid": {"spacing_m": [3.42, 2.57]},
  "ned_frame_id": "session_20261016T091500Z"
}
```

규칙:

- `sequence`(1~9)와 `grid_id`는 촬영 순서 `5 → 4 → 1 → 2 → 3 → 6 → 9 → 8 → 7`과 일치해야 하며, 어긋나면 `422`로 거절합니다. 파일명은 순서 판단에 쓰지 않습니다.
- `camera`/`grid`/`ned_frame_id`는 어느 프레임에 실어도 되고 마지막 값이 적용됩니다. 없으면 기존 테스트 가정값(`[5, 10] m`, `2.5 × 5 m`)을 씁니다. `timestamp_s`/`ned_m`/`yaw_rad`/`agl_m`은 이미지별로 manifest에 들어갑니다.
- 같은 `sequence`를 다시 보내면 내용(SHA-256)이 같을 때는 무시하고, 다를 때는 교체합니다. 스티칭이 시작된 세션에는 더 보낼 수 없고(`409`) 새 `session_id`를 써야 합니다.
- 저장 위치: `runs/web/sessions/<session_id>/input/capture_NN_idK.<ext>` + `manifest.json`, 결과는 `output/`.

테스트: `tests/test_webapp_sessions.py` (3개). 전체 `38 passed`.
