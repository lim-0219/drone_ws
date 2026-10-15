# 창의설계 드론 지상 PC 스티칭

**다른 컴퓨터로 옮겨 처음 실행한다면 [START_HERE.md](START_HERE.md)의 설치·실행 순서부터 따라 하세요.**

촬영 순서 **5 → 4 → 1 → 2 → 3 → 6 → 9 → 8 → 7**의 수직 촬영 이미지 9장으로 평면 지도를 만드는 Python 프로젝트입니다. CPU OpenCV로 실행하며 RTX 5070이나 CUDA 설정은 필요하지 않습니다. 기본 특징점은 SIFT이고 ORB로 전환할 수 있습니다.

현재 검증 범위는 **합성 데이터, 자동 테스트 27개, Sheffield Park 3 공개 실영상 중 선정한 9장**입니다. 프로젝트의 실제 카메라 영상, Ubuntu 22.04 PC에서의 실행 성능, 실제 지면 거리 정확도는 아직 검증하지 않았습니다. 재현 결과는 `VALIDATION.md`, 공개 실영상의 선정 파일과 실행법은 [Sheffield 9장 안내](datasets/sheffield_park_3/selected9/README.md)를 참고하세요.

## 1. Ubuntu 22.04 설치

아래 명령은 압축을 푼 `drone-stitching` 폴더에서 실행합니다. Ubuntu 22.04의 기본 Python 3.10을 사용할 수 있습니다.

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-dev.txt
python -m pip install --no-deps -e .
```

`requirements.txt`는 실행 라이브러리, `requirements-dev.txt`는 테스트 도구를 포함합니다. GUI가 없는 `opencv-python-headless`를 사용하므로 화면 표시 대신 PNG를 저장합니다. 다른 OpenCV 배포 패키지와 섞지 않은 가상 환경을 사용하세요.

## 2. 합성 데이터로 바로 실행

```bash
drone-stitch synthetic --output runs/demo-input
drone-stitch stitch runs/demo-input/manifest.json --output runs/demo-sift
drone-stitch evaluate runs/demo-sift/transforms.json runs/demo-input/ground_truth.json --output runs/demo-sift/evaluation.json
drone-stitch stitch runs/demo-input/manifest_no_metadata.json --output runs/demo-orb --detector orb
python -m pytest -q
```

`python -m drone_stitch`도 `drone-stitch`와 같은 명령입니다. 출력 폴더는 새 폴더 또는 빈 폴더여야 합니다. 재실행할 때 다른 이름을 주면 이전 결과가 보존됩니다.

합성 데이터는 무작위 텍스처의 평면, 위치 변화, yaw 변화, 고도 변화, 약한 원근 변화, 밝기 차이를 포함합니다. 400×800 픽셀 이미지 9장과 보간용 CSV, 정답 변환 행렬이 생성됩니다. 정답은 검증에만 사용하고 스티칭에는 입력하지 않습니다. 특징점이 풍부하고 평면이라는 유리한 조건이므로 작은 합성 오차를 실비행 정확도로 해석하면 안 됩니다.

## 3. 실제 입력 준비

`examples/manifest.json`을 촬영 폴더에 복사하고 파일 경로와 촬영 조건을 수정합니다. 예제의 이미지 파일은 자리표시자이며, 예제 CSV 값도 가상의 좌표입니다.

```text
my-flight/
  manifest.json
  telemetry.csv          # 선택 사항
  images/
    capture_01.png       # 위치 5
    capture_02.png       # 위치 4
    ...
    capture_09.png       # 위치 7
```

```bash
drone-stitch stitch my-flight/manifest.json --output runs/flight-001 --config examples/config.json
```

이미지는 JSON 배열에 촬영 순서대로 넣습니다. `id`를 생략하면 순서대로 5,4,1,2,3,6,9,8,7을 부여합니다. 지정한 `id`가 이 순서와 다르면 오류로 처리합니다. 경로는 manifest 위치 기준입니다. EXIF 자동 회전은 사용하지 않고 파일에 저장된 픽셀 방향을 읽습니다.

최소 입력 예시는 다음 구조입니다. 실제로는 `images` 항목 9개가 필요합니다.

```json
{
  "version": 1,
  "dataset_kind": "real",
  "camera": {
    "footprint_m": [5.0, 10.0],
    "reference_agl_m": 5.0,
    "image_top_yaw_offset_rad": 0.0
  },
  "grid": {"spacing_m": [2.5, 5.0]},
  "images": [
    {"id": 5, "file": "images/capture_01.png"},
    {"id": 4, "file": "images/capture_02.png"}
  ]
}
```

**`footprint_m=[5,10]`은 카메라의 검증된 사양이 아닌 초기 가정값입니다.** 첫 값은 이미지의 u축(오른쪽), 둘째 값은 v축(아래쪽) 전체 폭의 지면 길이입니다. 1920×1200 원본 이미지에 5×10m를 그대로 적용하면 축/종횡비가 잘못될 수 있습니다. 카메라 장착 방향, 저장된 이미지 방향, 실측 범위에 맞춰 설정하세요. 격자 이동 간격과 촬영 범위는 서로 다른 설정입니다.

`reference_agl_m`는 촬영 범위를 측정한 지상고입니다. 이미지별 `agl_m`가 있으면 범위를 `agl_m/reference_agl_m`만큼 확대·축소합니다. `LOCAL_POSITION_NED.z`는 EKF 원점 기준 아래 방향 좌표이므로 `-z`를 자동으로 지상고로 쓰지 않습니다.

## 4. 선택적인 Pixhawk 메타데이터

두 입력 방식을 지원합니다. 둘 다 있으면 해당 이미지에 직접 입력한 필드가 CSV보다 우선합니다.

**촬영별 직접 입력:** `images`의 각 항목에 다음 필드를 추가합니다. 위치, yaw, 지상고는 각각 생략할 수 있습니다.

```json
{
  "id": 5,
  "file": "images/capture_01.png",
  "timestamp_s": 100.0,
  "ned_m": [10.0, 20.0, -5.0],
  "yaw_rad": 0.0,
  "agl_m": 5.0
}
```

**시각별 CSV:** `examples/manifest_with_telemetry.json`과 `examples/telemetry.csv`를 참고합니다. manifest 최상위에 `telemetry_csv`, `ned_frame_id`를 넣고 이미지마다 `timestamp_s`를 지정합니다.

```csv
timestamp_s,north_m,east_m,down_m,yaw_rad,agl_m
99.9,10.0,20.0,-5.0,0.0,5.0
100.1,10.0,20.0,-5.0,0.0,5.0
```

- `LOCAL_POSITION_NED`의 x,y,z를 각각 north_m,east_m,down_m에 넣습니다. yaw는 별도의 `ATTITUDE.yaw` 등의 값이며 단위는 rad입니다. `LOCAL_POSITION_NED` 자체에는 yaw가 없습니다.
- 이미지와 CSV 시각은 같은 시계의 초 단위여야 합니다. `time_boot_ms/1000`을 쓴다면 노출 시각도 그 시계로 변환해야 합니다. PC 수신 시각을 노출 시각으로 간주하지 않습니다. 원시 MAVLink/ULog 파싱과 시계 동기화는 이 프로젝트의 입력 수집 계층 밖입니다.
- 위치는 선형 보간, yaw는 최단 각도 방향으로 보간합니다. 기본 최대 보간 간격은 0.5초, 최근접 허용 차이는 0.05초입니다. 큰 공백은 보간하지 않습니다. 필드별 비동기 샘플을 빈 셀로 표현할 수 있지만 위치의 N/E/D 세 값은 함께 제공해야 합니다.
- CSV 시각은 엄격히 증가해야 합니다. 동일 시각의 서로 다른 메시지는 한 행으로 병합하세요. EKF 원점 변경이나 시계 리셋을 걸쳐 한 지도를 만들지 마세요.
- 사용할 수 없는 메타데이터는 격자 가정으로 보완하고 실제 적용한 값과 출처를 저장합니다. 메타데이터가 없는 경우 NED 변환을 제공하지 않습니다.

## 5. 좌표계와 원점

5번의 지면 수직 투영점을 `(0,0)m`으로 고정합니다. 기본 격자는 다음과 같습니다.

```text
 Y+
 7(-2.5,+5)   8(0,+5)   9(+2.5,+5)
 4(-2.5, 0)   5(0, 0)   6(+2.5, 0)  → X+
 1(-2.5,-5)   2(0,-5)   3(+2.5,-5)
```

`grid.heading_rad=θ`는 격자 +Y 방향의 북쪽 기준 시계방향 각도입니다. 생략하고 5번 yaw가 있으면 `yaw_5 + image_top_yaw_offset_rad`를 사용합니다. 두 값 모두 없으면 격자는 방위가 없는 로컬 좌표계이며 NED 위치는 적용하지 않습니다. 방위를 정할 근거 없이 NED 축을 격자 축으로 간주하지 않기 위한 처리입니다.

```text
ΔN = N - N5, ΔE = E - E5
X = -sin(θ)·ΔN + cos(θ)·ΔE
Y =  cos(θ)·ΔN + sin(θ)·ΔE
이미지 위쪽의 격자 기준 각도 = yaw + 장착 offset - θ
```

기체 전방과 이미지 위쪽이 같으면 장착 offset은 0입니다. 이미지 위쪽이 기체 오른쪽을 향하면 +π/2입니다. 짐벌이 기체와 독립적으로 회전한다면 기체 yaw 대신 실제 카메라 yaw를 공급해야 합니다. yaw가 없는 이미지는 격자 +Y와 이미지 위쪽이 같다고 초기화합니다.

행렬은 행 우선 JSON 3×3 배열이며 **열벡터**에 곱합니다. `q=H@[u,v,1]`, 결과는 `q[:2]/q[2]`입니다. 픽셀 중심은 정수 좌표, 영상 경계는 `-0.5`부터 `width-0.5`입니다. 지도 u는 오른쪽, v는 아래쪽이므로 로컬 Y+는 지도 v 감소 방향입니다.

지도 해상도 기본값은 100px/m입니다. 지도 좌표 변환 행렬은 이 출력 좌표계에서 정확히 역변환되지만, **실제 미터값의 정확도는 촬영 범위·지상고·장착각·원점 추정의 정확도에 의존**합니다. 이미지 정합만으로 절대 위치 오차나 지상 거리 오차가 해결되는 것은 아닙니다. 모든 결과에 `metric_reference.survey_validated=false`를 기록합니다.

## 6. 렌즈 왜곡 보정

내부 보정값이 있다면 `camera.calibration`에 아래 형식으로 넣습니다. 숫자는 설명용입니다.

```json
{
  "image_size": [400, 800],
  "K": [[400, 0, 199.5], [0, 400, 399.5], [0, 0, 1]],
  "distortion": [-0.10, 0.01, 0.0, 0.0, 0.0]
}
```

OpenCV 일반 핀홀 왜곡 모델이며 fisheye 모델은 지원하지 않습니다. K는 저장된 이미지 해상도와 방향에 맞아야 하고 skew는 0이어야 합니다. 보정 후 같은 K와 해상도를 유지하며 잘라내지 않습니다. 보정 이미지의 주점 `(cx,cy)`를 카메라의 수직 투영점으로 사용하고, 보정값이 없으면 이미지 중심을 사용합니다.

`footprint_m`는 **보정 후 픽셀 축 범위**에 맞춰 설정합니다. 평지·수직 핀홀 모델에서는 기준 지상고 h에 대해 `[image_width*h/fx, image_height*h/fy]`로 계산할 수 있습니다. 이 구현은 사용자가 입력한 촬영 범위를 스케일 기준으로 사용하며 K로 자동 덮어쓰지 않습니다.

저장되는 이미지별 homography는 **왜곡 보정된 픽셀**에서 지도로 가는 행렬입니다. 원본의 왜곡된 픽셀을 변환할 때는 `image_to_pixel(..., raw_pixels=True)`를 사용하세요. 렌즈 왜곡은 단일 homography로 표현할 수 없습니다. 보정된 이미지를 입력했다면 다시 왜곡 보정하지 않도록 manifest를 구성하세요.

## 7. 결과 파일과 드론 위치 표시

| 파일 | 내용 |
|---|---|
| `map.png` | 블렌딩된 지도. 미촬영 영역은 검정 |
| `valid_mask.png` | 촬영된 영역 255, 미촬영 영역 0. 실제 검은 물체와 구분 |
| `coverage.png` | 각 픽셀을 덮는 이미지 수 0~9. 표시용 명암 정규화 없음 |
| `overview.png` | 지도 위에 영상 외곽, 영상 번호, 원점 표시 |
| `transforms.json` | 이미지별 초기/최종/역변환, 지도↔로컬 변환, 기준 NED·yaw·시각, 입력 설정 및 메타데이터 |
| `diagnostics.json` | 특징점 수, 각 쌍의 RANSAC 통계/거절 이유, 정합 잔차, 최적화 상태, 실행 환경/시간 |

실패 시 진단 JSON을 저장하고 명령은 종료 코드 2를 반환합니다. 연결이 끊긴 영상 집합을 임의 배치한 성공 지도로 출력하지 않습니다.

```bash
# 원점이 지도에서 어느 픽셀인지
drone-stitch point runs/demo-sift/transforms.json --local 0 0
# 합성 데이터의 NED 원점 확인
drone-stitch point runs/demo-sift/transforms.json --ned 100 200 --frame-id synthetic-session-1
# 로컬 위치를 표시한 이미지 저장
python examples/show_drone.py runs/demo-sift 1.0 2.0
```

```python
from drone_stitch.coordinates import MapCoordinates

coords = MapCoordinates("runs/flight-001/transforms.json")
uv = coords.local_to_pixel([[1.0, 2.0]])
xy = coords.pixel_to_local(uv)
uv_drone = coords.ned_to_pixel([[north_m, east_m]], frame_id="flight-001-ekf-origin-0")
coverage = coords.coverage_at(uv_drone)
# 검출한 원본 이미지 픽셀도 지도에 표시 가능
uv_object = coords.image_to_pixel(5, [[u, v]], raw_pixels=True)
```

`ned_frame_id`가 저장되어 있으면 API 호출 때 같은 값을 명시해야 합니다. 이는 세션 혼동 방지용 식별자이며 EKF 원점 변경을 자동 감지하지는 않습니다. 원점이 변경되면 기준점과 변환을 다시 정해야 합니다. 좌표가 지도 바깥이거나 빈 영역에 있는지도 확인할 수 있습니다. 라이브 MAVLink 수신과 GUI는 포함하지 않았으며 좌표 API에 연결하면 됩니다.

## 8. 정합 처리와 모듈

1. 격자 위치와 사용 가능한 상대 NED·yaw·지상고로 초기 homography를 만듭니다.
2. ORB/SIFT 특징점을 추출합니다. 특징점 계산만 축소하며 변환 행렬은 전체 해상도 픽셀 기준입니다.
3. 가로·세로 이웃 12쌍과 기본 대각선 8쌍을 비교합니다. 상호 일치하는 ratio-test 매칭에 RANSAC을 적용하고 인라이어 수·분포·양방향 오차·변환 형태를 확인합니다.
4. 가장 신뢰도 높은 연결들로 전체 최적화의 시작값을 만듭니다. 격자/메타데이터 시작값의 목적함수가 더 작으면 그 값을 선택합니다.
5. 5번 변환을 고정하고 나머지 8개의 homography(각 8개 변수)를 **모든 유효 연결에 대해 동시 최적화**합니다. 인라이어 대응점의 공통 좌표 오차, 초기 위치·yaw·크기에 대한 약한 제약, SciPy `soft_l1` 손실을 사용합니다.
6. 최적화 수렴, 연결별 잔차, 뒤집힘/비정상 배율, 영상 내부 무한대 투영을 검사합니다.
7. 지정 해상도로 워핑하고 유효 마스크의 가장자리 거리로 feather blending합니다. 검은 픽셀을 배경으로 취급하지 않습니다.

| 모듈 | 역할 |
|---|---|
| `inputs.py`, `metadata.py` | manifest, 영상 보정, 시각 보간, 초기 배치 |
| `geometry.py` | 영상/격자/NED 좌표 및 homography |
| `features.py` | 특징점, 인접 그래프, RANSAC, 매칭 품질 |
| `optimize.py` | 연결성 검사, 전체 정합 및 품질 검사 |
| `render.py` | 캔버스 제한, 영역별 워핑, 마스크 블렌딩 |
| `pipeline.py` | 전체 실행, 결과/진단 저장 |
| `coordinates.py` | 후속 위치 표시용 변환 API |
| `synthetic.py`, `tests/` | 재현 데이터 생성과 자동 검증 |

이 정합은 평면 장면의 2D projective 최적화입니다. 카메라 3D 자세/지형 복원이나 측량용 정사영상 생성은 구현하지 않습니다. 밝기 차이는 단순 feather로 완화하며 노출 보정, seam 최적화, multiband blending은 포함하지 않습니다.

## 9. 주요 설정과 실영상 적용

`examples/config.json`에 필요한 항목만 덮어쓸 수 있고 전체 기본값은 `config.py`에 있습니다.

| 설정 | 기본값 | 의미 |
|---|---:|---|
| `detector` | sift | orb로 교체 가능 |
| `feature_max_dimension` | 1600 | 특징점 계산 영상의 최대 변 길이 |
| `ransac_threshold_px` | 3 | 원본 해상도에서의 픽셀 임계값 |
| `min_inliers` | 18 | 연결당 최소 인라이어 |
| `diagonals` | true | 대각 이웃 8쌍 추가 |
| `match_all` | false | true면 총 36쌍 검사 |
| `grid_prior_sigma_m` | 5 | 격자 초기 사각형의 약한 제약. 클수록 약함 |
| `metadata_prior_sigma_m` | 1.5 | 메타데이터가 적용된 초기 사각형의 제약 |
| `max_edge_median_error_px` | 4 | 최적화 후 5번 영상 좌표 기준 연결별 중앙 오차 상한 |
| `pixels_per_meter` | 100 | 지도 출력 해상도 |
| `max_canvas_megapixels` | 24 | 큰 오정합/메모리 과다 사용 방지 |

최대 캔버스 24MP에서는 영상·누적 가중치·미리보기와 입력 영상 때문에 수백 MB 이상, 입력 해상도에 따라 1GB 안팎 이상 메모리를 사용할 수 있습니다. 지도 해상도를 낮추면 사용량이 줄어듭니다. 입력 영상 9장은 메모리에 함께 읽습니다.

실영상 적용 시 먼저 지상 정지 촬영 9장으로 보정값/축 방향을 확인한 뒤 비행 영상에 적용하세요. 다음 결과를 별도로 기록해야 합니다.

- 캘리브레이션과 실제 촬영 범위, 카메라 장착 offset, 지상고 및 노출 시각 동기화.
- 겹치는 지면의 잔차와 `overview.png`의 이중 경계. 반복 무늬·약한 텍스처에서는 RANSAC도 잘못된 정합을 만들 수 있습니다.
- 지상에서 독립 측정한 점과 거리 오차. 특징점 잔차가 작아도 실제 거리·NED 위치 정확도를 보장하지 않습니다.
- 기울기, 지형 높이 차, 높은 물체의 시차, 움직이는 물체, 그림자 변화. 이 조건들은 평면 모델의 한계입니다.
- 동일 EKF 세션에서 드론을 알려진 지점으로 이동했을 때 지도상의 위치 표시 오차.

문제가 있으면 `diagnostics.json`의 거절 이유부터 확인합니다. 특징점 부족은 촬영 범위/질감/흔들림을 확인하고, 이웃 관계가 실제 배치와 다르면 `match_all`을 검토합니다. 무조건 임계값을 완화해 성공 지도를 만드는 방식은 피하세요.

## 구현에 참고한 공식 문서

- [MAVLink LOCAL_POSITION_NED](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED), [ATTITUDE](https://mavlink.io/en/messages/common.html#ATTITUDE): NED 축·단위와 yaw 입력 규약.
- [OpenCV 특징점 매칭과 Homography](https://docs.opencv.org/4.x/d1/de0/tutorial_py_feature_homography.html): 특징점 대응과 RANSAC homography API.
- [SciPy least_squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html): 비선형 최소제곱, 강건 손실, 희소 Jacobian.
