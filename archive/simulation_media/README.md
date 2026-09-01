# SCONE 시뮬레이션 캡처

이 폴더는 논문용 benchmark controller를 사람이 빠르게 확인할 수 있도록 만든
저용량 영상과 대표 사진을 보관한다. 생성 명령은 다음과 같다.

```bash
mjpython -m benchmark capture --suite all
```

기본 인코딩은 640×360, 15 fps, H.264 CRF 34, 무음이며 각 영상과 같은 이름의
progressive JPEG가 `images/`에 생성된다. 원본 무압축 frame은 저장하지 않는다.

## 장면

- `flat_*`: 평지 `vx=0.18 m/s`에서 관절 보행, 말단 단독 회전, 전체 SCONE 주행
- `stairs_200mm_*`: 200 mm riser / 350 mm tread에서 세 계단 전략
- `transition_*`: Walk→Roll과 Roll→Walk 전환 전체
- `robustness_*`: 고정 perturbation과 `terrain_seed=2027`의 불규칙 지형 예시

정확한 파라미터, 성공 여부, simulation/encoded duration과 파일 크기는
[`manifest.json`](manifest.json)에 저장된다.

## 해석 제한

영상은 현재 MuJoCo 모델에서 화면상 동작을 확인하는 자료다. 실물 성능 증거가
아니며 카메라 perspective 때문에 속도·거리·계단 높이를 영상만으로 측정하면 안
된다. 정량 결과는 `benchmark/results/*.jsonl`과 대응 문서를 사용한다.
