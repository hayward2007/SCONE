# SCONE RL

input은 [x, y, yaw] 값을 넣어서 명령 조건부 정책 (command-conditioned policy)로 진행함 <- z 대신 yaw가 있는 것은 로봇이 공중에서 날지 않기 때문

각 스텝마다 input을 새로 만드는데 랜덤 값을 주기는 하나 값이 이전 스텝에서 튀기 보다 미분가능한 곡선에서 각 이산적인 값들이 나타내질 수 있게 인풋 데이터를 만드는 알고리즘이 필요함

policy에서 출력하는 action을 설정해야함. 첫 구현에서는 18개 actuator의 기준
목표각에 더할 작은 보정값(position residual)을 사용함

target = reference_gait + action_scale * action, 타겟 움직임은 기존 Walk 보행을
부드러운 주기 운동으로 만든 기준값 + policy 보정값

관측값 : 폴리씨가 판단에 사용할 로봇 상태 정의, 몸체 기울기, 속도 관절각도 속도 이전 action 등등

이동 명령 <- 이게 x, y, yaw 넣는거

좋아요. SCONE의 시뮬레이션 생성부터 강화학습, 조이스틱 운용까지 한 번에 정리하면 다음 순서입니다.

## 준비 단계

1. **목표 정의** - SCONE이 `[전후 속도, 좌우 속도, 회전 속도]` 명령을 따라 움직이도록 목표를 정합니다.

2. **MJCF 모델 제작** - `SCONEv2.xml`에 몸체, 다리, 관절, 질량, 관성, 충돌 형상을 정의합니다.

3. **관절 검증** - 각 joint의 위치, 회전축, 움직이는 방향, 제한 각도를 확인합니다. 현재 SCONE에서는 우선 이 부분을 수정해야 합니다.

4. **Actuator 연결** - 18개 관절에 각각 actuator를 연결합니다. Policy는 여전히 목표 각도(18개)만 출력하고, 그 목표 각도를 실제로 실현하는 물리 모델은 `<dcmotor>` + `pid.py`로 구성됩니다 (아래 "DYNAMIXEL과 MuJoCo dcmotor 액추에이터" 참고).

5. **물리 설정** - 중력, 지면 마찰력, MuJoCo timestep 등을 설정합니다. 예를 들어 timestep이 `0.002초`면 물리 계산은 `500Hz`입니다.

## 강화학습 환경 설계

1. **Action 정의** - Policy가 출력할 18개 값을 정의합니다. 각 값은 18개 actuator의 목표 관절 각도에 대응합니다.

2. **Action 범위 설정** - Policy 출력은 보통 `-1~1`로 제한하고 실제 관절 각도로 변환합니다.

```python
target = default_pose + action_scale * action
```

1. **관측값 정의** - Policy가 판단에 사용할 로봇 상태를 정합니다. 예를 들면 몸체 기울기, 몸체 속도, 관절 각도, 관절 속도, 이전 Action 등이 있습니다.

2. **이동 명령 정의** - 관측값에 조이스틱 역할을 하는 명령을 포함합니다.

```python
command = [vx, vy, yaw_rate]
```

1. **보상함수 정의** - 명령한 속도를 잘 따라가면 보상하고, 넘어짐·미끄러짐·심한 흔들림·과도한 토크에는 감점을 줍니다.

2. **종료 조건 정의** - 몸체가 지면에 닿거나 너무 많이 기울어지면 현재 에피소드를 종료하도록 설정합니다.

3. **초기화 조건 정의** - 새로운 에피소드가 시작될 때 SCONE을 기본 자세로 배치하고 관절 속도와 이전 Action 등을 초기화합니다.

## 한 번의 Policy 제어 단계

1. **명령 생성** - 학습 프로그램이 이번에 따라갈 속도 명령을 생성합니다.

```python
command = [0.3, 0.0, 0.0]  # 0.3m/s 전진
```

1. **상태 관측** - MuJoCo에서 현재 몸체와 관절 상태를 읽어옵니다.

```python
observation = get_observation(data, command)
```

1. **Policy 실행** - Policy가 관측값을 입력받고 18개의 Action을 출력합니다.

```python
action = policy(observation)
```

1. **목표 각도 변환** - `-1~1`의 Action을 실제 관절 목표 각도로 변환하고 관절 한계 안으로 제한합니다.

```python
target = default_pose + action_scale * action
target = np.clip(target, lower_limit, upper_limit)
```

1. **Actuator에 전달** - 계산된 18개의 목표 각도를 시뮬레이션 컨트롤러에
   전달합니다. 현재 `model.xml`은 전압 입력 방식의 `dcmotor`이므로 목표각을
   `data.ctrl`에 직접 넣지 않습니다.

```python
for motor_id, target_degrees in enumerate(targets, start=1):
    controller.set_position(motor_id, target_degrees)
```

1. **물리 시뮬레이션 실행** - Policy가 50Hz이고 MuJoCo가 500Hz라면 같은 Action을 유지하면서 물리 계산을 10번 실행합니다.

```python
for _ in range(10):
    controller.update(model.opt.timestep)  # 목표각 -> 제한된 모터 전압
    mujoco.mj_step(model, data)
```

1. **결과 관측** - Action을 적용한 뒤 SCONE의 새로운 위치, 속도, 자세와 관절 상태를 읽습니다.

2. **보상 계산** - 실제 속도가 명령 속도와 얼마나 비슷한지, 몸체가 안정적인지 등을 이용해 이번 단계의 점수를 계산합니다.

3. **경험 저장** - 다음 정보를 학습용 메모리에 저장합니다.

```text
이전 관측값
Action
보상
새로운 관측값
종료 여부
```

1. **다음 제어 단계 진행** - 종료 조건에 걸리지 않았다면 새로운 관측값으로 다시 Policy를 실행합니다.

## 에피소드 진행

1. **제어 단계 반복** - 13~22단계를 계속 반복하며 SCONE이 몇 초 동안 걷게 합니다.

2. **에피소드 종료** - SCONE이 넘어지거나 제한 시간이 끝나면 에피소드를 끝냅니다.

3. **환경 초기화** - SCONE을 다시 기본 자세에 배치하고 새로운 이동 명령으로 다음 에피소드를 시작합니다.

## Policy 학습

1. **학습 데이터 수집** - 여러 에피소드에서 수백~수천 개의 경험을 모읍니다.

2. **누적 보상 계산** - 각각의 Action이 장기적으로 얼마나 좋은 결과를 만들었는지 계산합니다.

3. **Policy 업데이트** - PPO 같은 알고리즘이 좋은 결과를 만든 Action은 더 자주 출력하고, 나쁜 결과를 만든 Action은 덜 출력하도록 신경망을 수정합니다.

4. **수집과 업데이트 반복** - 아래 과정을 수백만 번의 시뮬레이션 스텝 동안 반복합니다.

```text
경험 수집 → Policy 업데이트 → 경험 수집 → Policy 업데이트
```

1. **점진적으로 난이도 증가** - 처음에는 느린 전진만 학습시키고, 이후 후진·횡이동·회전과 더 빠른 속도를 추가합니다.

2. **성능 평가** - 학습에 사용하지 않은 명령에서도 속도 추종, 안정성, 에너지 사용량과 미끄러짐을 검사합니다.

3. **Policy 저장** - 충분히 학습된 신경망 파라미터를 파일로 저장합니다.

## 조이스틱 운용

1. **학습된 Policy 불러오기** - 저장된 Policy를 시뮬레이션 또는 실제 SCONE 제어 프로그램에서 불러옵니다.

2. **조이스틱 입력 변환** - 조이스틱 입력을 속도 명령으로 변환합니다.

```text
왼쪽 스틱 위아래 → 전진·후진 속도
왼쪽 스틱 좌우   → 좌우 이동 속도
오른쪽 스틱 좌우 → 회전 속도
```

1. **실시간 제어** - 조이스틱 명령과 현재 로봇 상태를 Policy에 넣고, 출력된 18개 목표 각도를 모터에 전달합니다.

2. **보상함수 제거** - 학습 완료 후 실제 운용할 때는 보상 계산과 Policy 업데이트가 필요 없습니다. 보상함수는 Policy를 훈련할 때만 사용합니다.

전체를 한 줄로 압축하면 다음과 같습니다.

```text
명령과 상태 관측
→ Policy가 18개 목표 각도 생성
→ Actuator에 전달
→ MuJoCo 물리 계산
→ 결과에 보상 부여
→ 경험을 이용해 Policy 업데이트
→ 학습 완료 후 조이스틱으로 명령
```

## walk_learn.py: 첫 보행 강화학습 환경

`walk_learn.py`에 Gymnasium 환경과 PPO 학습 진입점을 만들었습니다. 이것은
최종 튜닝값이 아니라, 보상 항목을 하나씩 측정하고 고칠 수 있는 **첫 기준선**입니다.

### 입력, 관측값, Action

이동 명령은 로봇 몸체 좌표계 기준의 다음 세 값입니다.

```text
command = [vx, vy, yaw_rate]
vx       : 전진(+)/후진(-) 속도, m/s
vy       : 좌(+)/우(-) 속도, m/s
yaw_rate : 반시계(+)/시계(-) 회전 속도, rad/s
```

Policy가 받는 관측값은 총 68개입니다.

| 관측값 | 개수 | 의미 |
| --- | ---: | --- |
| 몸체 선속도 | 3 | 몸체 좌표계의 x, y, z 속도 |
| 몸체 각속도 | 3 | roll, pitch, yaw 각속도 |
| 몸체에 투영한 중력 방향 | 3 | roll/pitch를 각도 불연속 없이 표현 |
| 18개 관절 위치 | 18 | 기본 자세로부터의 각도 차이 |
| 18개 관절 속도 | 18 | 각 모터가 움직이는 속도 |
| 이전 Action | 18 | 급격한 명령 변화를 판단하는 데 사용 |
| 이동 명령 | 3 | `[vx, vy, yaw_rate]` |
| 보행 위상 | 2 | 주기의 `sin`, `cos` |

Action은 `[-1, 1]` 범위의 값 18개입니다. 지금은 Action 자체를 전류로 쓰지 않고,
기존 Walk 동작으로 만든 기준 목표각에 작은 각도 보정값을 더합니다.

```text
q_target = q_reference(command, phase) + residual_scale * action
```

- 바디 관절 1~6: 최대 ±10도 보정
- 1단 관절 7~12: 최대 ±12도 보정
- 2단 관절 13~18: 최대 ±15도 보정

이 방식을 **residual RL**이라고 부릅니다. 기존 `src/provider/walk.py`의 대각
삼각보 그룹 `{2,3,6}` / `{1,4,5}`와 20도 보폭·들기 동작을 부드러운 주기
함수로 바꾸어 `q_reference`로 사용했습니다. 따라서 처음부터 무작위 전류로
넘어지는 동작을 탐색하는 대신 기존 보행 주변에서 개선을 시작합니다.

직접 전류 Action을 첫 버전에서 사용하지 않은 이유도 있습니다. 다리의 XM430
(ID 7~18)은 전류 제어를 지원하지만, 몸통의 MX-28AT (ID 1~6)는 전류 센서와
전류 제어 모드가 없습니다. 18개 모터에 동일한 Action 의미를 유지하고 실기로
옮길 수 있게 하려면 목표각 보정 방식이 안전한 출발점입니다. `dcmotor`와
`pid.py`는 이 목표각을 실제 모터의 속도-토크 특성이 반영된 전압과 토크로
바꿉니다.

### 보상함수

한 Policy 스텝의 총 보상은 아래 항목의 합입니다. 모든 항은 제어 주기
`dt=0.02초`를 곱했기 때문에 나중에 50Hz를 바꾸어도 초당 보상 크기가 크게
달라지지 않습니다.

```text
R = + 2.0 * 전후/좌우 속도 추종
    + 1.0 * yaw 속도 추종
    + 0.5 * 수평 자세 유지
    - 0.2 * 몸체 높이 변화
    - 0.1 * z/roll/pitch 진동
    - 0.02 * Action 급변
    - 0.02 * 전류 사용량
    - 0.1 * 접지점 미끄러짐
    - 0.2 * 관절 한계 접근
    - 1.0 * 타이어 외 부품의 바닥 충돌
```

속도 보상은 “빠를수록 무조건 높은 점수”가 아니라 **명령 속도와 실제 속도가
가까울수록** 높습니다. 그렇지 않으면 `[0, 0, 0]` 정지 명령에도 전속력으로
달리는 편법을 배울 수 있습니다. 각 속도 추종값은 `exp(-오차²/σ²)` 형태라
0~1 사이이며, 정답 근처에서 부드럽게 변합니다.

수평 자세 항은 roll/pitch 각도를 Euler angle로 직접 계산하지 않고 몸체에
투영된 중력 벡터를 씁니다. 높이 변화와 수직 속도, roll/pitch 각속도는 별도
감점으로 두어 “평균 자세는 맞지만 계속 덜덜 떠는” 동작도 구분합니다. 진동을
물리적으로 과도하게 댐핑해서 숨기기보다는 실제 모터 감쇠를 유지하고 이 항으로
학습시키는 편이 sim-to-real에 더 적합합니다.

### 호형 말단의 접지 미끄러짐

Walk 모드에서 호형 말단은 바퀴처럼 계속 구르는 것이 아니라, 부채꼴 끝부분을
발처럼 바닥에 고정해 몸체를 밉니다. 그래서 다음처럼 계산합니다.

1. `TIRE_1_geom`~`TIRE_6_geom`과 바닥의 **실제 접촉점**을 찾습니다.
2. 접촉 수직력이 1N 이상인 지지 다리만 선택합니다.
3. 접촉점의 속도에서 바닥 법선 방향을 빼고 접선 속도만 계산합니다.
4. 접선 속도가 0.02m/s를 넘는 부분에만 미끄러짐 감점을 줍니다.

공중에 든 스윙 다리는 접촉이 없으므로 감점하지 않습니다. 또한 말단 body의
중심 속도를 쓰지 않기 때문에, 나중에 Drive 모드에서 정상적으로 구르는 운동과도
혼동하지 않습니다. 이 보상은 Walk 전용입니다.

### 종료 조건과 난이도

다음 중 하나가 발생하면 넘어짐으로 보고 에피소드를 즉시 종료하며 `-5`점을
추가합니다.

- 몸체 기울기가 60도를 넘음
- 초기 서 있는 높이보다 0.12m 이상 내려감
- 기본 자세로부터 관절이 90도 이상 벗어남
- 타이어가 아닌 충돌 geom이 바닥에 닿음
- NaN/무한대와 같은 물리 계산 오류가 발생함

명령 난이도는 세 단계입니다.

| 단계 | 명령 범위 `[|vx|, |vy|, |yaw_rate|]` | 목적 |
| --- | --- | --- |
| `easy` | `[0.30, 0.00, 0.00]` | 느린 전진·후진부터 학습 |
| `medium` | `[0.40, 0.00, 0.60]` | 전후진 + 회전 |
| `full` | `[0.50, 0.25, 0.80]` | 전후·좌우·회전 전체 |

처음에는 `easy`로 보상 그래프와 실제 움직임을 확인해야 합니다. 검증 후
`--resume`으로 이전 Policy를 불러와 `medium`, `full` 순서로 명령 범위만 넓혀
이어 학습합니다.

### 실행 방법

환경과 접촉·보상 계산부터 검사합니다.

```bash
python3 walk_learn.py check --steps 500
python3 walk_learn.py check --steps 500 --random-actions
```

첫 전진/후진 PPO 학습을 실행합니다. MuJoCo 물리는 500Hz, Policy는 50Hz이며
Policy Action 하나를 유지하는 동안 물리 계산을 10번 수행합니다.

```bash
python3 walk_learn.py train \
  --curriculum easy \
  --timesteps 1000000 \
  --num-envs 4 \
  --output runs/scone_walk_easy
```

저장 결과는 `runs/scone_walk_easy/final_model.zip`, 중간 checkpoint 및
`monitor.csv`입니다. 학습된 Policy를 고정 명령으로 화면에서 확인합니다.

```bash
mjpython walk_learn.py enjoy runs/scone_walk_easy/final_model.zip \
  --command 0.25 0.0 0.0
```

`easy` 결과를 이어 받아 회전을 추가하는 예시는 다음과 같습니다.

```bash
python3 walk_learn.py train \
  --curriculum medium \
  --resume runs/scone_walk_easy/final_model.zip \
  --timesteps 1000000 \
  --output runs/scone_walk_medium
```

### 학습 전에 반드시 다시 측정할 값

- `model.xml`에 실제 기계적 관절 `range`가 아직 없으므로 현재는 기본 자세 ±60도
  clip과 ±90도 종료 조건을 임시로 사용합니다. 실제 hard stop을 재서 교체해야 합니다.
- 일부 시각 mesh는 충돌이 꺼져 있습니다. 현재 프레임 충돌 감점은 MuJoCo에서
  실제 collision geom으로 설정된 부품만 감지합니다.
- 시뮬레이션 관측값의 몸체 선속도는 실기에서 그대로 얻을 수 없습니다. 실기
  배포 전 IMU/관절 상태 기반 속도 추정기 또는 별도 estimator가 필요합니다.
- 시뮬레이션과 실기 모터 ID·회전 방향 매핑을 한 모터씩 최종 확인해야 합니다.
- 보상 가중치는 출발값입니다. 각 항은 TensorBoard에서 `reward/*`, 실제 상태는
  `state/*`로 기록되므로 한 항이 전체 보상을 압도하는지 확인하며 조정해야 합니다.

## SSH 원격 학습을 로컬에서 실시간 확인

`remote_watch.py`는 원격 머신의 학습 화면을 전송하지 않습니다. 원격 학습은
뷰어 없이 최대 속도로 계속 돌리고, 로컬 컴퓨터가 최신 PPO checkpoint를 SSH로
가져와 로컬 `model.xml`에서 재생합니다.

```text
원격 walk_learn.py
  → 주기적으로 scone_walk_<step>_steps.zip 저장
  → 로컬 remote_watch.py가 SSH로 최신 번호 검색
  → .part 파일로 다운로드
  → ZIP 무결성 및 관측/Action 크기 검사
  → 검사에 통과한 Policy만 로컬 MuJoCo에 적용
```

이 방식은 원격 학습 프로세스에 렌더링 부담을 주지 않습니다. 다만 “방금 원격에서
실행된 로봇 상태”를 영상처럼 보는 것이 아니라, **가장 최근에 저장된 Policy를
로컬에서 다시 실행**하는 방식입니다. 화면 갱신 지연은 대략
`checkpoint 저장 간격 + SSH polling 간격`입니다.

### 1. 로컬과 원격 코드 맞추기

Policy의 관측값 의미와 MuJoCo 물리가 같아야 하므로 최소한 다음 파일은 원격과
로컬에서 같은 버전을 사용해야 합니다.

```text
walk_learn.py
model.xml
src/
meshes/
```

Python 패키지도 호환되는 버전을 맞추는 것이 안전합니다. 현재 로컬 검증 환경은
MuJoCo 3.10.0, Gymnasium 1.3.0, Stable-Baselines3 2.9.0, PyTorch 2.13.0입니다.
특히 Stable-Baselines3/PyTorch 버전 차이가 크면 원격에서 저장한 ZIP을 로컬에서
불러오지 못할 수 있습니다.

예를 들어 로컬에서 원격 프로젝트로 복사할 때는 다음처럼 실행할 수 있습니다.
`<SSH_HOST>`는 `~/.ssh/config`에 등록한 별칭 또는 `user@hostname`입니다.

```bash
rsync -av \
  walk_learn.py model.xml src meshes \
  <SSH_HOST>:~/Developer/SCONE/
```

이미 실행 중인 원격 Python 프로세스는 파일을 메모리에 읽은 상태이므로, 파일을
복사한 뒤에는 학습 프로세스를 다시 시작해야 변경 내용이 반영됩니다.

### 2. 원격 머신에서 학습

SSH로 원격 머신에 접속한 뒤 학습합니다. 실시간 확인용으로 checkpoint 간격을
20,000스텝으로 설정한 예시입니다.

```bash
cd ~/Developer/SCONE
python3 walk_learn.py train \
  --curriculum easy \
  --timesteps 1000000 \
  --num-envs 4 \
  --checkpoint-every 20000 \
  --keep-checkpoints 10 \
  --output runs/scone_walk_easy
```

`--keep-checkpoints 10`은 최신 10개만 유지해 장시간 학습 중 디스크가 계속
증가하는 것을 막습니다. 최종 Policy는 별도로 `final_model.zip`에 저장됩니다.

### 3. SSH와 checkpoint 다운로드만 먼저 검사

로컬 컴퓨터에서 다음 명령을 먼저 실행합니다.

```bash
python3 remote_watch.py \
  --host <SSH_HOST> \
  --checkpoint-dir '~/Developer/SCONE/runs/scone_walk_easy/checkpoints' \
  --download-only
```

`downloaded step ...`가 나오면 SSH 인증, 원격 경로, checkpoint 이름과 ZIP 검사가
전부 통과한 것입니다. 기본 prefix는 `scone_walk`이며 학습 코드가 생성하는
`scone_walk_<step>_steps.zip`과 일치합니다.

### 4. 로컬 MuJoCo 실시간 재생

macOS에서는 MuJoCo 창을 메인 스레드에서 열기 위해 `mjpython`으로 실행합니다.

```bash
mjpython remote_watch.py \
  --host <SSH_HOST> \
  --checkpoint-dir '~/Developer/SCONE/runs/scone_walk_easy/checkpoints' \
  --poll-interval 5 \
  --curriculum easy \
  --command 0.25 0.0 0.0
```

처음 checkpoint를 찾기 전에는 기존 기준 보행과 0 residual Action이 보입니다.
새 checkpoint를 찾으면 터미널에 다음 로그가 나오고 Policy가 자동 교체됩니다.

```text
[remote-watch] downloaded step 20,000: ...
[remote-watch] now replaying step 20,000
```

SSH config에 Port와 IdentityFile이 있으면 그대로 사용합니다. 직접 덮어쓸 때만
`--port 2222`, `--identity-file ~/.ssh/id_ed25519`를 추가합니다. 프로그램은
`BatchMode=yes`를 사용하므로 비밀번호를 코드에 저장하거나 반복 입력하지 않으며,
OpenSSH의 기존 host-key 검사를 끄지 않습니다.

네트워크가 잠시 끊기거나 저장 중인 ZIP을 불완전하게 읽으면 현재 Policy 재생을
계속하면서 다음 polling 때 다시 시도합니다. 로컬로 받은 파일은
`runs/remote_watch/`에 캐시되며 Git에는 포함되지 않습니다.

같은 컴퓨터에서 학습과 뷰어를 시험할 때는 SSH 대신 다음 명령을 사용할 수
있습니다.

```bash
mjpython remote_watch.py \
  --local-dir runs/scone_walk_easy/checkpoints \
  --curriculum easy \
  --command 0.25 0.0 0.0
```

## DYNAMIXEL 게인과 MuJoCo 게인의 차이 (이력)

`model.xml`은 처음에 18개 액추에이터를 전부 MuJoCo `<position>` (PD 서보)으로
만들었고, DYNAMIXEL Control Table의 P/D 레지스터를 공식 변환식으로 옮긴 값을
`kp`/`kv`로 썼습니다.

| 모터 | 기본 P/D 레지스터 | 펌웨어 내부 변환 | 내부 P/D 게인 |
| --- | ---: | --- | ---: |
| MX-28AT, Protocol 1 | P=32, D=0 | P/8, D\*4/1000 | P=4, D=0 |
| XM430-W350-T | P=800, D=0 | P/128, D/16 | P=6.25, D=0 |
| XM430-W210-T | P=800, D=0 | P/128, D/16 | P=6.25, D=0 |

다만 이 `4`, `6.25`는 DYNAMIXEL 펌웨어 내부 게인일 뿐 MuJoCo의 토크 게인이
아니라서(단위가 다름), 최종적으로는 "보수적 연속사용 토크 한계가 5도 오차에서
포화되도록 `kp = 토크 한계 / 5도`, `kv`는 임계감쇠"로 다시 계산해 사용했습니다.
이때 나온 `kp`/`kv` 값은 지금도 유효합니다 -- 아래 dcmotor 방식에서 토크공간
PID 게인으로 그대로 재사용됩니다. `<position>` 액추에이터 자체는 더 이상 쓰지
않습니다 (아래 참고).

## DYNAMIXEL과 MuJoCo dcmotor 액추에이터

`<position>` 액추에이터의 근본적인 한계는 "각도 오차 -> 토크"만 계산하고, 그
값이 `forcerange`를 넘으면 속도와 무관하게 그냥 잘라버린다(hard clip)는
점입니다. 실제 DC 모터는 속도가 빨라질수록 역기전력(back-EMF) 때문에 낼 수
있는 토크가 연속적으로 줄어듭니다. RL 관점에서는 이 차이가 중요한데, `<position>`
+ hard clip 모델로 학습하면 policy가 "순간적으로 최대 토크를 계속 뽑아 쓰는"
sim-to-real에서 통하지 않는 편법을 배울 수 있습니다.

그래서 18개 액추에이터를 전부 MuJoCo `<dcmotor>`로 바꿨습니다
([MuJoCo dcmotor XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-dcmotor),
MuJoCo 3.7.0에서 추가된 요소). `nominal="전압 정지토크 무부하속도"` 하나만 주면
컴파일러가 모터 상수 `K = 전압/무부하속도`와 저항 `R = K*전압/정지토크`를 자동
계산해서, 토크-속도 직선(정지토크에서 시작해 무부하속도에서 0이 되는 선형 관계)을
그대로 재현합니다.

| 모터 | 정지토크 (12V) | 무부하속도 (12V) | K (N·m/A) | R (Ω) |
| --- | ---: | ---: | ---: | ---: |
| MX-28AT | 2.5 N·m | 55 rpm | 2.083 | 10.00 |
| XM430-W350-T | 4.1 N·m | 46 rpm | 2.491 | 7.291 |
| XM430-W210-T | 3.0 N·m | 77 rpm | 1.488 | 5.953 |

`saturation`은 처음엔 이전 `<position>`이 쓰던 보수적 연속사용 토크로
잡았었는데(MX-28AT ±0.50, XM430-W350-T ±0.82, XM430-W210-T ±0.60 N·m),
**실제로 문제를 일으켜서 각 모터의 정지토크로 다시 올렸습니다** (아래
"1단 모터가 목표각에 못 미치고 멈추는 문제" 참고). 이유: dcmotor의 K/R 자체가
이미 "고토크는 저속에서만" 관계를 갖고 있어서, `saturation`을 굳이 더 낮게
잡을 이유가 없었고 — 오히려 하중이 실린 상태에서 자세를 복구하는 순간에
연속사용 한계보다 더 큰 토크가 필요한 상황이 실제로 있었습니다. 정지토크까지
순간적으로 쓰는 건 실제 DYNAMIXEL도 정상적으로 하는 동작이고(정지토크가 원래
그런 뜻), "policy가 편법을 배운다"는 우려는 **지속적으로** 정지토크를 유지하는
경우에 해당하는 것이지 순간적인 사용까지 막을 필요는 없었습니다.
inductance(전기 시정수)/thermal(발열)/cogging(코깅 토크)/lugre(정밀 마찰)는
지금은 비활성 기본값(0)으로 남겨뒀고, 나중에 실제 모터 스텝 응답을 측정하면
그 데이터로 채울 수 있습니다.

## pid.py -- 목표각을 전압으로 바꾸는 외부 루프

`<dcmotor>`는 `input="voltage"`(기본값)를 쓰므로 `data.ctrl`은 목표 각도가
아니라 **전압**입니다. MuJoCo의 dcmotor는 `input="pos vel ff"`를 쓰면 이
변환을 내부적으로 대신 해주는 PID 컨트롤러도 갖고 있지만, SCONE에서는 그
로직을 `src/simulation/pid.py`에 파이썬으로 직접 구현해서 게인이 XML 안에
숨지 않고 보이게/튜닝 가능하게 했습니다. 이 파일의 `DCMotorPID.step()`이
쓰는 식은 MuJoCo dcmotor의 `controller` 속성이 문서화한 것과 동일합니다.

```text
torque  = kp*(목표각 - 현재각) + kd*(목표각속도 - 현재각속도) + ki*적분 + ff
voltage = (R / K) * torque + K * 현재각속도   # 뒷항이 역기전력 보상
```

`kp`/`kd`는 이전 `<position>` 시절에 구했던 값을 그대로 재사용합니다
(토크공간 게인이라는 의미가 그대로 유지되기 때문):

| 액추에이터 | kp (N·m/rad) | kd (N·m·s/rad) | 연속토크 한계 |
| --- | ---: | ---: | ---: |
| A01-A06, MX-28AT | 5.73 | 0.752 | ±0.50 N·m |
| A07-A12, XM430-W350-T | 9.40 | 0.792 | ±0.82 N·m |
| A13-A18, XM430-W210-T | 6.88 | 0.264 | ±0.60 N·m |

`ki`는 기본 0입니다 (보행 중에는 목표각과 접촉 상태가 계속 바뀌기 때문에,
적분 와인드업 위험이 있는 `ki`는 지속적인 정상상태 오차가 실제로 관측될 때만
넣는 것을 권장). `src/simulation/controller.py`의 `update()`는 기존과
동일하게 DYNAMIXEL Profile Velocity/Acceleration 램핑으로 매 스텝의
목표각/목표각속도를 만들고, 그 값을 `DCMotorPID.step()`에 넘겨 전압을 받아
`data.ctrl`에 씁니다 -- 즉 policy와 `Walk`/`Drive`/`Climb` 쪽 코드는
전혀 바뀌지 않았습니다.

## 실제 로봇에 배포할 때: 모터 종류에 따라 옵션이 다름

Policy의 행동 공간(목표 각도 18개)은 실기에서도 그대로 씁니다 -- "전압/전류를
직접 학습"하는 방식으로 바꾸지 않았기 때문에, 상위 제어 코드는 시뮬레이션과
실기 모두에서 동일합니다. 다만 다리 모터와 몸통 모터는 실기에서 지원하는
옵션이 다릅니다 (ROBOTIS e-Manual 기준).

- **XM430-W350-T / XM430-W210-T (다리, ID 7-18)**: **Current-based Position
  Control Mode** (`operating_mode = 5`)를 지원합니다. `Goal Position`(주소
  116)과 `Goal Current`(주소 102)를 동시에 독립적으로 지정할 수 있어서, "이
  각도로 가되 토크는 이 이하로 제한"을 모터 자체가 처리합니다. 지금
  `src/core/actuator.py`의 `XM.operating_mode`에는 이 값(5)이 없고
  `goal_current` 주소도 없으므로, 실기에 이 정도까지 반영하려면 그 두 가지만
  추가하면 됩니다 (상위 제어 코드는 그대로).
- **MX-28AT (몸통, ID 1-6)**: 전류 센서가 아예 없어서 전류 기반 모드가
  존재하지 않습니다. Joint Mode(위치 제어) + `Torque Limit`(주소 34)로 토크
  상한만 걸 수 있고, dcmotor 시뮬레이션처럼 전압/전류를 실시간으로 명령할 수는
  없습니다. 즉 시뮬레이션이 더 사실적으로 바뀌어도, 몸통 6개 모터는 실기에서
  계속 지금과 같은 위치+토크상한 방식일 수밖에 없습니다.

참고 자료:

- [ROBOTIS MX-28AT, Protocol 1 e-Manual](https://emanual.robotis.com/docs/en/dxl/mx/mx-28/)
- [ROBOTIS XM430-W350-T e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)
- [ROBOTIS XM430-W210-T e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/)
- [MuJoCo dcmotor actuator XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-dcmotor)
- [MuJoCo Menagerie ROBOTIS OP3 model](https://github.com/google-deepmind/mujoco_menagerie/blob/main/robotis_op3/op3.xml)

## symetry.py 삭제

`src/simulation/symetry.py`(좌우 ID 1↔2, 3↔4 ... 스왑)와 `--symmetry` 플래그를
완전히 제거했습니다. 이유는 실측으로 확인했습니다: `model.xml`에서 같은 부호의
관절 각도를 좌우 한 쌍(L1/L2, L3/L4, L5/L6)에 동시에 넣고 발끝이 몸체 기준
어느 방향으로 움직이는지 비교했더니,

| 관절 단 | 같은 부호를 좌우에 넣었을 때 |
|---|---|
| 바디(M01-M06) | 몸체 기준 **같은 방향** |
| 1단(M07-M12) | 몸체 기준 **같은 방향** |
| 2단/휠(M13-M18) | 몸체 기준 **반대 방향** |

바디·1단 관절은 애초에 좌우 차이가 없으므로 ID를 맞바꾸면 그냥 엉뚱한 다리로
명령이 가는 것이고, 2단(휠) 관절은 실제로 좌우가 반대로 반응하지만
`symetry.py`는 부호를 안 건드리고 ID만 바꾸므로 이 경우도 보정이 안 됩니다.
즉 이 방식으로는 애초에 고칠 수 있는 문제가 아니었습니다. 기본값이 꺼져
있어서(`mirror_commands=False`) 지금까지 동작에는 영향이 없었지만, 코드
자체를 지웠습니다.

`Drive.left()`/`Drive.right()`가 `Actuator.lower_index`(13-18) 전부에 동일
부호 속도를 주는 것은 위 표와 맞습니다 — 2단 관절이 좌우 반대로 반응하니,
전부 같은 부호를 주면 좌우가 자연히 반대로 굴러 제자리 회전이 나옵니다. 지금
단계에서는 Drive는 그대로 두고 Walk 쪽 개발/검증에 집중하기로 했습니다.

## 부품 번호를 다리/모터 번호에 맞춤

`model.xml`의 body/geom 이름 중 `BODY_ACTUATOR_N`, `FR07_N`, `FR12-v1_N`,
`LINK_N`, `ARC_SHAPED_WHEEL_N`, `TIRE_N`은 원래 Fusion 익스포트 순서를
따라서 다리 번호와 어긋나 있었습니다 (예: 옛 `TIRE_2`는 L2가 아니라 L3의
타이어). 전부 실제 다리 번호에 맞게 다시 번호를 붙였고, `LEG_ACTUATOR_N`도
그 부품을 구동하는 모터 ID(07-18)에 맞췄습니다. 조인트 이름(`M0X_..._LY`)은
원래부터 다리 번호를 올바르게 쓰고 있었으므로 바뀌지 않았습니다.

```text
옛 번호(Fusion 순서) -> 다리 번호: 1=L1, 2=L3, 3=L5, 4=L2, 5=L4, 6=L6
```

지금은 `TIRE_3`이면 무조건 L3, `LEG_ACTUATOR_09`면 무조건 M09를 구동하는
부품입니다. 예전 노트나 스크린샷에 옛 번호가 남아 있다면 위 대응표로 환산하면
됩니다.

## 모든 모터의 회전 방향 규칙과 initial position 버그 (해결됨)

18개 액추에이터는 전부 raw `0~4096`이 한 바퀴(2π rad)에 대응하는 동일한
규칙을 씁니다 (`raw 2048` = 중앙). 이 규칙 자체는 모터 종류나 좌우에 상관없이
완전히 동일하고, 실기 컨트롤러(`core/controller.py`)도 시뮬레이션도 이 점은
처음부터 올바르게 구현돼 있었습니다.

문제는 다른 곳에 있었습니다: **L2/L4/L6 다리 전체는 L1/L3/L5를 Z축 기준
180도 "회전 복사"해서 만들어졌지, 거울처럼 "반사"해서 만든 게 아닙니다.**
회전 복사와 반사는 수학적으로 다릅니다 — 그래서 관절축이 이 180도 회전축
(Z축)과 나란하면 좌우가 "똑같이" 반응하고, 수직이면 "반대로" 반응합니다.
실측해보니:

| 단 | 관절축과 180도 회전축(Z)의 관계 | 같은 값을 좌우에 주면 |
|---|---|---|
| 바디(M01-M06) | 나란함 | **똑같이** 반응 (거울 대칭엔 반대로 반응해야 정상이라 버그) |
| 1단(M07-M12) | 수직 | 반대로 반응 (거울 대칭에 정확히 맞음, 정상) |
| 2단(M13-M18) | LINK 파트에서 추가로 한 번 더 180도 꼬여서 다시 정상으로 돌아옴 | 반대로 반응 (정상) |

즉 **바디 관절(M02/M04/M06)만 유일하게 문제였습니다.** Fusion에서 내보낸 관절
축이 180도 회전 복사를 따라가지 못하고 오른쪽과 같은 방향을 그대로 유지하고
있었던 것입니다. `SCONE.Standard`의 home 자세(`upper_initial_position =
[135,135,180,180,225,225]`, 좌우 동일값)를 그대로 렌더링해보면 오른쪽 다리는
정상으로 뻗지만 왼쪽 다리는 몸통 쪽으로 접혀 들어가는 게 실제로 보였습니다.

**중요:** 이건 제어 코드의 문제가 아니라 **모델(`model.xml`)만의 문제**였고,
그래서 실제 로봇은 항상 정상 작동했던 것입니다 (실기 모터는 XML을 모르고,
실제로 조립된 다리는 애초에 올바른 방향으로 붙어 있었기 때문). 고친 방법은
`M02_body_L2`/`M04_body_L4`/`M06_body_L6` 세 관절의 `axis`만 부호를
반전한 것 — 그 외 어떤 것도(제어 코드, 다른 관절, 메쉬, 질량) 건드리지
않았습니다. `Walk`/`Drive`/`Climb`/`SCONE.py`가 지금처럼 좌우에 똑같은
각도값을 보내는 방식 그대로 정상적으로 대칭 자세가 나옵니다 — 실기 컨트롤러
코드는 전혀 수정하지 않았습니다.

검증: `SCONE.Standard`의 home 자세를 실기와 동일한 좌우-동일값 그대로 렌더링해서
수정 전/후를 비교 — 수정 전엔 좌우 비대칭(오른쪽만 정상), 수정 후엔 6족이
완전 대칭 방사형으로 펼쳐짐을 확인했습니다.

## core/actuator.py 가정 (참고용, 실기 배선과 대조는 여전히 유효)

`src/core/actuator.py`의 ID 그룹은 아래를 가정합니다. 회전 방향 버그는
해결됐지만, "배선 ID 자체가 이 표와 맞는지"는 별개 확인 사항으로 남아있으니
실기로 한 ID씩 개별 구동해보면서 확인하는 걸 권장합니다.

| ID | 단 | 모터 | 프로토콜 | 가정된 물리적 위치 |
|---:|---|---|---|---|
| 1,3,5 | 바디(upper) | MX-28AT | 1.0 | 오른쪽(전/중/후) |
| 2,4,6 | 바디(upper) | MX-28AT | 1.0 | 왼쪽(전/중/후) |
| 7,9,11 | 1단(middle) | XM430-W350-T | 2.0 | 오른쪽(전/중/후) |
| 8,10,12 | 1단(middle) | XM430-W350-T | 2.0 | 왼쪽(전/중/후) |
| 13,15,17 | 2단(lower) | XM430-W210-T | 2.0 | 오른쪽(전/중/후) |
| 14,16,18 | 2단(lower) | XM430-W210-T | 2.0 | 왼쪽(전/중/후) |

대각(tripod) 그룹은 표준 6족 보행의 삼각보 패턴을 가정합니다: 매 단마다
`{전-우, 중-좌, 후-우}` = `upper_diagonal_right_index=[1,4,5]`와
`{전-좌, 중-우, 후-좌}` = `upper_diagonal_left_index=[2,3,6]` (1단·2단도
+6, +12한 동일 패턴). 배선 ID가 실제로 이 표와 같은지, 대각 그룹으로 걸을 때
실제로 삼각보(한 번에 세 다리만 지면에서 떨어짐)가 나오는지는 실기 확인이
필요합니다. 다른 점이 발견되면 `Actuator`의 인덱스 리스트만 고치면 되고,
`Walk`/`Drive`/`Climb` 로직은 인덱스 이름만 참조하므로 그대로 재사용됩니다.

## Floating base로 바꾼 뒤 로봇이 주저앉던 문제 (해결)

`model.xml`에 루트 `<freejoint>`와 바닥 plane을 영구적으로 추가한 뒤,
`home()` 이후 로봇이 원래 높이보다 10~20cm씩 주저앉는 문제가 있었습니다.
원인이 세 개 겹쳐 있었고 전부 시뮬레이션 쪽(`model.xml`, `src/simulation/*`)
문제였습니다 — 실기 컨트롤러(`src/core/*`)는 전혀 안 건드렸습니다.

1. **몸체 부품 대부분이 충돌 비활성(`contype="0"`)** — 다리가 조금만 처져도
   `BODY_ACTUATOR`/`FR07`/`LEG_ACTUATOR`/`LINK`/`UPPER_BODY`/`LOWER_BODY`
   같은 시각용 부품이 바닥을 그대로 통과했습니다 (오직 타이어만 충돌 有).
   실측: 처짐이 생기면 이 부품들이 바닥보다 최대 2.7cm 아래로 내려감.
   **고침**: `UPPER_BODY_1_geom`/`LOWER_BODY_1_geom`에 충돌을 켜서(`contype="1"
   conaffinity="1"`), 다리가 처져도 몸체 플레이트가 바닥에 물리적으로
   걸리게 만듦 — 이후 최악의 경우도 0.9cm 이내로 줄어듦.
2. **`MuJoCoController`가 CAD 원본 자세(모든 관절 raw 2048)에서 시작** —
   이 자세는 서 있는 자세가 아니라서 첫 물리 스텝부터 불안정했습니다.
   **고침**: `_seed_stable_pose()`가 생성자에서 대략적인 "서 있는" 각도로
   qpos를 미리 채우고, freejoint가 있으면 가장 낮은 충돌 지점이 바닥에
   오도록 몸체 높이도 맞춰줌 (시뮬레이션 전용, 실기엔 이런 개념 자체가
   없음). 이것만으로는 근본 원인을 못 고쳤지만(아래 3번 참고), 첫 스폰
   상태를 더 안전하게 만들어 놓는 건 그대로 유지.
3. **(진짜 원인) 1단 모터가 하중이 실린 상태에서 목표각까지 못 감** —
   `runner.home()`은 1단(middle)을 안전 대피각 135도로 먼저 보낸 뒤, 상체+하체를
   최종 자세로 옮기고(이때 로봇 전체 하중이 다리에 실림), **마지막에** 1단을
   135→240도로 복귀시킵니다. 실측해보니 이 마지막 단계에서 1단이 240도까지
   못 가고 170도 근처에서 멈췄습니다(68도 부족!) — 그리고 2단(휠)은
   부채꼴이라 반지름이 일정하지 않아서, 1단이 어긋나면 접지 위치가 달라져
   전체 높이까지 바뀝니다. 사용자가 XM430-W350-T e-manual의
   토크/속도/전류/효율 성능그래프를 제공해줘서 대조해보니, 그래프가 끝나는
   지점(~2.9 N·m)이 정지토크(4.1 N·m)보다 낮은 이유는 "그래프 측정 범위가
   거기까지"일 뿐 모터 한계가 아니라는 게 ROBOTIS 문서에 명시돼 있었습니다.
   즉 `saturation`을 정지토크보다 낮은 "보수적 연속사용" 값으로 잡아둔 게
   문제였습니다. **고침**: `saturation`을 세 모터 전부 정지토크로 올림
   (MX-28AT 0.50→2.5, XM430-W350-T 0.82→4.1, XM430-W210-T 0.60→3.0 N·m).
   결과: 1단이 240도 목표에 238도까지 도달(오차 2도), root 높이도 정상
   범위(+0.155)로 완전히 회복. `kp`/`kd`는 그대로 둬도 됨 — dcmotor의
   K/R이 이미 "고토크는 저속에서만" 관계를 갖고 있어서 `saturation`만
   올리면 큰 오차에서 더 큰 토크를 자연스럽게 쓸 수 있게 됨.

**정리: 순간적으로 정지토크까지 쓰는 건 실제 DYNAMIXEL도 정상적으로 하는
동작**(정지토크가 원래 그런 뜻)이고, 문제가 될 수 있는 건 policy가 이걸
"지속적으로" 유지하는 경우뿐입니다. 1번(몸체 충돌)과 2번(시작 자세)은 그
자체로도 유효한 개선이라 남겨뒀지만, 실제로 문제를 일으킨 근본 원인은
3번(`saturation`이 너무 보수적이었던 것)이었습니다.
