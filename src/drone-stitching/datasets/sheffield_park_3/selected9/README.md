# Sheffield Park 3: 선정한 실영상 9장

[공개 원본](https://github.com/pierotofy/drone_dataset_sheffield_park_3)의 32장을 내려받아 GPS, 짐벌 방향, 실제 영상 및 정합 결과를 확인하고 서쪽 구간의 9장을 골랐습니다. 원본 JPG는 회전·축소·재압축하지 않았습니다. 원본 라이선스는 동봉한 `LICENSE`를 확인하세요.

## 공간 배치와 파일

아래는 북쪽이 위인 대략적인 공간 배치입니다. 정확한 정사각 격자는 아니며 GPS 상대 좌표를 초기 배치에 사용합니다.

| 서쪽 | 가운데 | 동쪽 |
|---|---|---|
| 7: DJI_0079.JPG | 8: DJI_0082.JPG | 9: DJI_0084.JPG |
| 4: DJI_0076.JPG | **5: DJI_0075.JPG** | 6: DJI_0074.JPG |
| 1: DJI_0062.JPG | 2: DJI_0063.JPG | 3: DJI_0065.JPG |

코드의 입력 순서 `5→4→1→2→3→6→9→8→7`에 대응하는 파일은 다음과 같습니다.

```text
DJI_0075.JPG → DJI_0076.JPG → DJI_0062.JPG → DJI_0063.JPG → DJI_0065.JPG
→ DJI_0074.JPG → DJI_0084.JPG → DJI_0082.JPG → DJI_0079.JPG
```

번호는 테스트용 공간 배치에 맞춰 부여했습니다. **이 배열은 실제 비행의 촬영 시간 순서가 아닙니다.** 원본 촬영 시각은 `manifest.json`의 `source_metadata.capture_time_original`과 `selected_metadata.csv`에 보존했습니다. 시간 보간을 사용하는 `timestamp_s`는 지정하지 않았습니다.

`selected9_overview.jpg`는 북쪽이 위가 되도록 돌린 미리보기이며 실제 입력은 `images/`의 원본입니다. `capture_positions.png`는 전체 32장과 선택한 9장의 GPS 촬영 위치를 표시합니다.

## 실행

프로젝트 루트의 `README.md`에 따라 환경을 설치한 뒤, `drone-stitching` 폴더에서 실행합니다.

```bash
python -m drone_stitch stitch datasets/sheffield_park_3/selected9/manifest.json --config datasets/sheffield_park_3/selected9/config.json --output runs/my-sheffield-test
```

출력 폴더는 없거나 비어 있어야 합니다. 원본은 4000×3000 픽셀이며, 특징점 계산 때만 긴 변을 2000 픽셀로 줄입니다. 출력 해상도는 명목상 15 px/m로 지정했습니다.

이미 실행한 결과는 `runs/sheffield_selected9_sift_west/`에 있습니다. 다른 PC용 통합 압축본에는 코드, 선정 원본 9장, 입력 설정, 실영상·합성 실행 결과, 초기 실패 진단을 포함했습니다. 전체 원본 32장은 포함하지 않습니다. 합성 입력은 제공된 생성기로도 재현할 수 있습니다.

## 메타데이터 해석

- 기준 원점은 5번 `DJI_0075.JPG`의 촬영 위치입니다. 로컬 X는 동쪽, Y는 북쪽입니다.
- EXIF GPS 위도·경도를 WGS84 기준의 근거리 상대 N/E로 근사 변환했습니다. `ned_m`의 D는 기준 사진 대비 GPS 고도 차의 부호를 바꾼 값입니다. **Pixhawk LOCAL_POSITION_NED 로그가 아닙니다.** 별도의 `ned_frame_id`로 구분합니다.
- 초기 방향은 DJI XMP의 `GimbalYawDegree`를 사용했습니다. 선택 사진의 `GimbalPitchDegree`는 약 -90도로 지면 수직에 가깝습니다.
- EXIF의 35mm 환산 초점거리 20mm, 영상 비율 4:3, 기준 사진의 상대 고도 61.3m를 이용해 촬영 범위를 약 106.1×79.6m로 추정했습니다. 상대 고도는 실제 지상고와 같다고 확인되지 않았으므로 이는 명목 축척입니다.
- 사진별 실제 지상고를 알 수 없어 개별 `agl_m`은 넣지 않았습니다. GPS 고도를 지상고로 사용하지 않습니다. 내부 카메라 보정값과 지상 기준점도 제공하지 않았습니다.

원본 메타데이터는 `manifest.json`의 `source_metadata`에 남겨 두었습니다. `selected_metadata.csv`로 주요 좌표와 방향을 확인하고 `selected_images_sha256.json`으로 원본 보존 여부를 검증할 수 있습니다.

## 실제 실행 결과와 한계

2026-10-13, macOS ARM64 / Python 3.12 / OpenCV 4.10 / CPU SIFT 기준입니다. 사용자의 Ubuntu 22.04 PC에서 실행한 결과는 아닙니다.

- 출력 지도: **2315×2617 px**, 처리 시간 약 **11.4초**.
- 이웃 연결 **20/20** 채택, 연결마다 RANSAC 인라이어 **187~1141개**.
- 연결별 중앙 잔차의 중앙값 **1.99 px**, 최댓값 **3.51 px**. 기존 통과 기준 4 px를 유지했습니다.
- 평면 촬영 범위의 추정 면적 중첩률: 상하·좌우 **52~80%**, 대각선 **34~65%**. 교집합 면적을 두 범위 중 큰 면적으로 나눈 값입니다.
- 지도↔로컬 좌표 왕복 변환과 원점 위치를 확인했습니다. 실측 정답을 이용한 거리·위치 정확도 검증은 아닙니다.

나무와 건물에는 평면 homography로 설명되지 않는 시차와 블렌딩 잔상이 남습니다. 공개 실영상으로 전체 파이프라인의 실행을 검증한 첫 사례이며, 실제 카메라의 비행 성능이나 측량 정확도를 입증하지 않습니다. 이번 9장에서는 SIFT만 실행했습니다.

처음 시도한 더 넓은 조합은 전체 정합 잔차가 기준을 넘어 실패했으며, 해당 진단을 보존했습니다. 따라서 임의의 9장이 모두 잘 붙는다는 의미는 아닙니다. 상세 수치는 성공 결과의 `validation_summary.json`, 실패 진단은 `runs/sheffield_selected9_sift/diagnostics.json`을 참고하세요.
