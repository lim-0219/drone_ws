# 다른 컴퓨터에서 시작하기

이 폴더 하나를 다른 컴퓨터에 복사하면 코드와 샘플 사진을 함께 사용할 수 있습니다. 압축을 먼저 완전히 풀고 아래 명령을 **drone-stitching 폴더 안에서** 실행하세요. 설치할 때 인터넷이 필요하며 설치 후 동봉된 데이터 실험은 오프라인으로 가능합니다.

## 1. Ubuntu 22.04에서 설치

Python 3.10~3.12를 사용합니다. Ubuntu 22.04 기본 Python 3.10을 사용할 수 있도록 라이브러리 버전을 고정했습니다. CPU OpenCV를 사용하므로 RTX 5070, CUDA, GPU용 OpenCV 설정이 필요하지 않습니다.

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
python3 scripts/verify_bundle.py
python3 scripts/setup_env.py
```

설치 도우미는 이 폴더 안에 `.venv`를 만들고 의존성과 프로젝트를 설치합니다. 시스템 Python 패키지를 변경하지 않습니다. 이미 `.venv`가 있으면 중단합니다. 실행할 때 아래처럼 환경의 Python을 직접 사용하면 별도로 환경을 활성화하지 않아도 됩니다.

## 2. 공개 실영상 9장으로 실행

```bash
.venv/bin/python scripts/run_demo.py sheffield
```

완료되면 `runs/sheffield-날짜-시각/sift/`에 다음 파일이 생깁니다. 매번 새 폴더를 만들어 기존 결과를 보존합니다.

| 파일 | 확인할 내용 |
|---|---|
| map.png | 스티칭된 지도 |
| overview.png | 지도 위 사진 번호·촬영 범위·원점 |
| transforms.json | 이미지별 행렬, 지도 픽셀↔로컬 지면 좌표, 원점과 메타데이터 |
| diagnostics.json | 매칭·전체 정합 품질과 실행 환경 |
| valid_mask.png, coverage.png | 유효 촬영 영역과 겹친 사진 수 |

선정 입력은 `datasets/sheffield_park_3/selected9/`에 있습니다. 5번 기준 사진은 `DJI_0075.JPG`이며 GPS·짐벌 yaw를 초기 배치에 사용합니다. 이 데이터의 위치는 GPS로 유도한 좌표이며 Pixhawk 로그가 아닙니다. 상세 배치와 메타데이터 해석은 해당 폴더의 `README.md`를 참고하세요.

기존 성공 결과 `runs/sheffield_selected9_sift_west/`와 비교할 수 있습니다. 공개 실영상 9장의 SIFT 기능 검증은 완료했지만, 나무·건물의 잔상과 미검증 거리 축척이 남아 있습니다. 이 영상은 우리 카메라의 고도 5m 실험 데이터가 아닙니다.

## 3. 합성 데이터와 자동 테스트

```bash
.venv/bin/python scripts/run_demo.py synthetic
.venv/bin/python -m pytest -q
```

합성 데모는 SIFT+메타데이터와 ORB+격자 초기화를 각각 실행하고 생성 정답과의 오차를 `evaluation.json`에 저장합니다. 공개 실영상에는 이런 실측 정답이 없습니다. 기존 자동 테스트는 27개입니다.

## 4. 내 사진으로 교체

1. `examples/manifest.json`을 새 촬영 폴더에 복사합니다. 사진 9장의 경로를 `5→4→1→2→3→6→9→8→7` 순서로 지정합니다.
2. 실제 카메라 방향·촬영 범위·격자 간격에 맞춰 값을 수정합니다. 예제 숫자는 가정값입니다.
3. 선택적으로 사진별 NED/yaw를 넣거나 `examples/manifest_with_telemetry.json`과 `telemetry.csv` 형식을 사용합니다.
4. 아래 명령에서 입력과 새 출력 폴더 경로를 지정합니다.

```bash
.venv/bin/python -m drone_stitch stitch my-flight/manifest.json --output runs/my-flight-001 --config examples/config.json
```

Pixhawk 좌표축, 촬영 시각 동기화, 렌즈 보정, 드론 위치 표시 API와 모듈별 설명은 루트 `README.md`에 있습니다. Python 소스는 `src/drone_stitch/`에서 수정합니다. 편집 가능 설치이므로 소스 수정 내용은 다음 실행에 반영됩니다.

## 5. 압축 파일의 구성

```text
drone-stitching/
  START_HERE.md              # 이 안내
  README.md                 # 입력·좌표계·알고리즘·API 상세
  VALIDATION.md              # 합성/실영상 검증 기록과 한계
  PORTABILITY_CHECK.md       # 옮긴 폴더에서의 설치·재현 확인
  THIRD_PARTY_NOTICES.md     # 사진 출처와 외부 라이브러리
  BUNDLE_MANIFEST.json       # 배포 파일별 SHA-256
  requirements*.txt          # 고정한 라이브러리 버전
  pyproject.toml
  scripts/                  # 설치, 데모, 배포 파일 검증 도우미
  src/drone_stitch/          # 모듈식 Python 구현
  tests/                    # 자동 테스트
  examples/                 # 내 사진·Pixhawk 입력 예제
  datasets/sheffield_park_3/selected9/
                            # 원본 사진 9장·메타데이터·설정·라이선스
  runs/verified_input/      # 기존 합성 입력과 정답
  runs/verified_sift/       # 기존 합성 SIFT 결과
  runs/verified_orb/        # 기존 합성 ORB 결과
  runs/sheffield_selected9_sift_west/
                            # 기존 공개 실영상 성공 결과
  runs/sheffield_selected9_sift/diagnostics.json
                            # 처음 시도한 다른 9장 조합의 실패 기록
```

가상 환경, 캐시, 전체 공개 데이터셋 32장, GPU 드라이버는 포함하지 않습니다. `.venv`는 컴퓨터마다 다시 만듭니다. 폴더를 이동한 뒤에도 기존 가상 환경을 재사용하지 말고 새로 설치하세요. 동봉 결과의 이미지 경로는 결과 JSON이 있는 폴더 기준 상대 경로로 바꿨으며 변환 행렬과 수치는 그대로 보존했습니다.

## 6. Windows/macOS 또는 설치 문제

다른 OS에서도 Python 3.10~3.12로 같은 설치 도우미를 사용할 수 있습니다. Windows에서는 `py -3.12 scripts/setup_env.py`로 설치하고, `.venv/bin/python` 대신 `.venv\Scripts\python.exe`를 사용하세요. macOS에서는 `python3 scripts/setup_env.py`와 `.venv/bin/python`을 사용합니다. Windows와 Ubuntu에서의 직접 실행은 아직 검증하지 않았으며 확인한 환경은 `PORTABILITY_CHECK.md`에 구분해 기록합니다.

- `No module named venv` / `ensurepip`: Ubuntu의 `python3-venv` 설치 여부를 확인하세요.
- 설치 중 네트워크 오류: 연결을 복구한 뒤 아래 두 명령으로 이어서 설치하세요.

```bash
.venv/bin/python -m pip install -r requirements-dev.txt
.venv/bin/python -m pip install --no-deps -e .
.venv/bin/python -m pip check
```

- `No module named drone_stitch`: 시스템 Python 대신 `.venv`의 Python을 사용하세요.
- 출력 폴더가 비어 있지 않다는 오류: 새 출력 경로를 지정하세요. 데모 도우미는 자동으로 새 경로를 만듭니다.
- `quality` / `disconnected` 등의 정합 실패: 출력 폴더의 `diagnostics.json`을 확인하세요. 사진의 겹침·흔들림·지형 높이 차를 먼저 점검합니다.
- 패키지 무결성 검증 실패: 압축을 다시 풀어 확인하세요. 코드나 설정을 수정한 뒤 해시가 달라지는 것은 정상입니다.
