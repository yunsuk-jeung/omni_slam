# IMU 노이즈 파라미터: 연속시간 밀도 vs 이산시간 std

> IMU 노이즈 파라미터(noise density / random walk)가 왜 "밀도"로 정의되는지,
> 이산화(discretization)를 반영하지 않으면 무엇이 틀어지는지를 이론적으로 정리한다.
> 마지막에 basalt / omni_slam / VINS-Mono 구현이 각각 어디서 이산화를 수행하는지 대응시킨다.

---

## 1. 연속시간 측정 모델

자이로 기준 (가속도계도 동일 구조):

```text
ω̃(t) = ω(t) + b(t) + n(t)          측정 = 참값 + bias + 백색잡음
ḃ(t)  = w(t)                        bias는 랜덤워크 (백색잡음의 적분)
```

백색잡음의 정의는 자기상관이 델타 함수라는 것:

```text
E[n(t)·n(t')ᵀ] = σ_c² · δ(t − t')
E[w(t)·w(t')ᵀ] = σ_b² · δ(t − t')
```

핵심: **δ(t−t')의 단위가 1/s** 이다. 따라서 σ_c²의 단위는 `(rad/s)²·s`,
σ_c의 단위는 `rad/s·√s = rad/s/√Hz`.
이것이 "density(밀도)"라는 이름의 근원이다 — **백색잡음은 한 시점의 분산이
무한대**(델타 자기상관)라서 "순간의 std"라는 개념 자체가 성립하지 않고,
시간(또는 대역폭)으로 정규화한 밀도로만 정의된다.

---

## 2. 샘플링하면 왜 √Δt가 나오나

실제 센서는 샘플 구간 Δt 동안의 평균을 출력한다:

```text
n_d[k] = (1/Δt) ∫ n(t) dt          (구간 [kΔt, (k+1)Δt])

Var(n_d) = (1/Δt²) · σ_c² · Δt = σ_c² / Δt

∴ σ_d = σ_c / √Δt = σ_c · √f       ← 이산시간(샘플당) std
```

**같은 센서라도 빨리 샘플링할수록 샘플 하나는 더 시끄럽다**
(평균을 낼 시간이 짧아지므로). 200 Hz면 σ_d = σ_c·√200 ≈ 14 σ_c.
"샘플 하나의 std"는 레이트에 따라 변하는 양이므로 센서 고유 특성이 될 수 없고,
레이트 불변량인 σ_c로 센서를 특징짓는다.

---

## 3. 검산: 누적 오차는 샘플레이트와 무관해야 한다

자이로 잡음을 시간 T 동안 적분했을 때의 각도 오차:

```text
연속:  Var(θ_err) = σ_c² · T

이산:  N = T/Δt 개 샘플의 합
       Var(θ_err) = Σ Var(Δt · n_d[k])
                  = N · Δt² · (σ_c²/Δt)
                  = σ_c² · T        ✓  Δt 소거 — 레이트 무관
```

basalt식 이름(`gyro_noise_std`)의 근거가 여기 있다:

```text
std(θ_err, T=1s) = σ_c·√1 = σ_c
```

즉 σ_c는 **"1초 적분 후 각도 드리프트의 std"** 라는 진짜 표준편차로 읽을 수 있다
(데이터시트의 ARW, Angle Random Walk와 동일량).
Kalibr는 같은 숫자를 주파수 영역(`per √Hz`)으로, basalt는 시간 영역(`per √s`)으로
읽은 것뿐이며 `1/√Hz ≡ √s`이므로 값은 완전히 동일하다.

**이산화를 생략하면** (σ_c를 샘플 std로 그대로 쓰면):

```text
Var(θ_err) = N · Δt² · σ_c² = σ_c² · T · Δt     ← Δt가 남는다!
```

누적 분산이 정답 대비 **Δt배 과소평가**되고 (200 Hz면 1/200),
샘플레이트를 바꾸면 추정기의 가중 밸런스가 통째로 달라진다.
이것이 "discrete 특성을 반영하지 않으면 생기는 차이"의 정확한 정체다.

---

## 4. bias random walk는 방향이 반대

bias는 백색잡음이 아니라 백색잡음의 **적분**(상태)이다:

```text
b(t+T) − b(t) = ∫ w dt
Var = σ_b² · T,    std = σ_b · √T
```

이산 스텝으로 쓰면:

```text
b[k+1] = b[k] + w_d[k],    Var(w_d) = σ_b² · Δt     ← 곱하기!
```

두 잡음의 이산화 방향이 반대인 비대칭이 생긴다:

| | 이산 분산 | 이유 |
|---|---|---|
| 측정 백색잡음 | `σ_c² / Δt` (나누기) | 측정치는 잡음의 **평균** → 구간이 짧을수록 시끄럽다 |
| bias RW 증분 | `σ_b² · Δt` (곱하기) | bias는 잡음의 **적분** → 구간이 길수록 많이 표류한다 |

`gyro_bias_std`(basalt) = `gyroscope_random_walk`(Kalibr) = σ_b 역시
**"1초 후 bias 표류량의 std"** 로 읽을 수 있는 동일한 값이다.

---

## 5. 추정기에서 어디에 어떻게 들어가나

**preintegration 공분산 전파** (omni_slam `ImuPreintegration::PropagateError`):

```text
Q_백색 = σ_c² / Δt              (이산 측정 분산; 샘플별 실제 Δt 사용)
V ∝ Δt                          (측정 노이즈 → 상태 야코비)
스텝당 주입 = V·Q·Vᵀ ∝ Δt²·(σ_c²/Δt) = σ_c²·Δt    ✓ §3와 일치
```

주의: 사다리꼴(midpoint) 적분처럼 인접 두 샘플을 평균하는 구조는 스텝당
주입 분산이 0.5배가 되므로, 백색잡음 Q 블록에 2를 곱해 보정한다
(`imu_preintegration.cpp`의 백색잡음 블록 참조).

**bias random-walk factor** (basalt `imu_block`; omni는 15D 공분산에 통합):

```text
잔차 = b_i − b_j        (프레임 간격 Δt_f)
분산 = σ_b² · Δt_f
whitening 가중 = 1/(σ_b·√Δt_f)     ✓ §4와 일치
```

---

## 6. 구현별 이산화 위치 비교

| | 변환 위치 | 사용하는 Δt | 비고 |
|---|---|---|---|
| **basalt** | `dicrete_time_*_noise_std()` = σ_c·√rate | 고정 nominal `imu_update_rate` | 지터/샘플드랍 시 스케일 오차 |
| **omni_slam** | `Q = σ_c²/Δt` (PropagateError 내부) | **샘플별 실제 Δt** | 지터에도 정확 |
| **VINS-Mono** | 변환 없음 | — | config 값이 차원적으로 애매, 튜닝으로 흡수 |

세 구현 모두 config 필드에 넣는 값 자체는 같은 물리량(연속시간 밀도)을 기대하는
것이 바람직하며, omni_slam은 basalt calib의 키 이름
(`accel_noise_std`, `gyro_noise_std`, `accel_bias_std`, `gyro_bias_std`)과
Kalibr 키 이름을 모두 alias로 지원한다 (`SVIOConfig::ParseConfig`).

---

## 7. 파라미터 값은 어디서 오나 — Allan variance

정지 상태 IMU 로그(수 시간)의 Allan deviation 곡선에서:

- **기울기 −1/2 구간**을 τ=1s에서 읽으면 → σ_c (noise density / ARW)
- **기울기 +1/2 구간**에서 → σ_b (bias random walk)

도구: `allan_variance_ros`, `imu_utils`, 또는 데이터시트 스펙.

실무 주의: Allan/데이터시트 값은 진동·scale factor 오차·시간동기 오차 같은
모델링 안 된 오차를 포함하지 않으므로, VIO에서는 보통 **2~10배 부풀려** 사용한다.
basalt가 배포하는 EuRoC 값(accel 0.016 = 데이터시트의 8배 등)도 이런 튜닝값이다.
