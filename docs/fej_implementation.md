# FEJ (First-Estimate Jacobians) 구현 정리

Basalt 방식(상대포즈 분해 + chain rule)을 따른 FEJ 구현 문서다. 관련 코드는
다음과 같다: 선형화 포인트는 `src/database/frame.hpp`와
`src/odometry/imu_preintegration.hpp`, 팩터는 `src/optimizer/relative_pose.hpp`와
`src/optimizer/cost_function.hpp`, 배선은 `src/optimizer/vio_estimator.cpp`,
스위치는 config의 `enable_fej`.

표기는 [marginalization_prior_math.md](marginalization_prior_math.md)와 같다.
회전행렬 $R$, log 좌표 $r = \mathrm{Log}(R)$, residual $e$, tangent 증분
$\delta = (\delta t, \delta\theta)$.

---

## 1. 왜 FEJ인가

야코비안은 선형화 지점의 함수다. marginalization prior는 과거 지점 $x^0$에서
선형화해 고정해 둔 정보인데, 이후에 들어오는 새 팩터들을 현재 추정치에서
재선형화하면 두 정보의 선형화 지점이 서로 어긋난다. 하필 이 어긋남이
**관측 불가능한 방향(전역 병진·yaw, gauge null-space)** 에 가짜
정보(spurious information)를 주입한다. 그 결과 시스템은 실제보다 자신만만해지고
(inconsistency), yaw가 서서히 드리프트한다.

FEJ의 처방은 간단하다. **prior에 연결된 상태의 야코비안은, 그 상태가 처음
prior에 편입될 때의 추정치(first estimate)에서 계속 평가한다.** 다만 이걸
순진하게 적용하면 모든 야코비안이 낡은 지점에 묶여 정확도가 떨어진다는 문제가
남는다.

## 2. Basalt의 트릭: 상대포즈로 갈라치기

핵심 아이디어는 reprojection residual을 절대포즈가 아니라 **상대포즈**를 거치게
쓰는 것이다:

$$e = \pi\big( T_{t\,h} \cdot p_h \big) - z,\qquad
T_{t\,h} = (T_{w b_t}\,T_{b c_t})^{-1}\,(T_{w b_h}\,T_{b c_h})$$

($h$ = host, $t$ = target/obs, $p_h$ = host 카메라 기준 랜드마크.)

여기서 결정적인 관찰: 모든 포즈에 전역 변환 $G$를 곱해도($T_{wb} \to G\,T_{wb}$)
$T_{t\,h}$는 변하지 않는다 — **상대포즈 공간은 gauge에 불변**이다. 그래서
야코비안을 chain rule로 두 조각으로 쪼개면 성질이 서로 다른 두 항이 나온다:

$$\frac{\partial e}{\partial \delta_{\text{pose}}} =
\underbrace{\frac{\partial e}{\partial \delta_{rel}}}_{J_{rel}\;(2\times6)}
\cdot
\underbrace{\frac{\partial \delta_{rel}}{\partial \delta_{\text{pose}}}}_{d_{rel}\;(6\times6)}$$

| 조각 | 평가 지점 | 이유 |
|---|---|---|
| residual $e$ | 항상 **현재** | 실제 오차여야 하므로 |
| $J_{rel}$ (잔차→상대포즈) | 항상 **현재** | gauge 불변 공간이라 재선형화해도 가짜 정보가 없다 → 정확도 유지 |
| $d_{rel}$ (상대→절대) | **FEJ** (동결 상태면) | gauge 방향이 야코비안에 들어오는 유일한 통로 |

즉 얼려야 하는 건 $d_{rel}$ 하나뿐이다. 덕분에 정확도 손해를 최소로 줄이면서
consistency를 얻는다. Basalt는 이 규칙을 단일 코드 경로로 처리한다. $d_{rel}$은
언제나 `getPoseLin()`에서 계산하고, 그 값을 동결할지 여부는 상태 객체가
결정한다(`linearization_abs_qr.cpp:195`).

## 3. 우리 구현

### 3.1 상태의 선형화 포인트

```cpp
// Frame
const Sophus::SE3d& twb_lin() const;   // 동결값 or 현재값
void set_lin_true();                   // 최초 1회만 동결
bool is_linearized() const;

// InertialState — 같은 패턴 (v, ba, bg 스냅샷)
```

둘 다 `std::optional` 스냅샷이다. 값이 들어 있으면 동결된 상태를 뜻한다. 동결
전에는 `*_lin()`이 현재값을 그대로 돌려주므로 **코드 경로가 하나로 유지**된다
(Basalt와 같은 설계).

### 3.2 동결 트리거

`VIOEstimator::marginalize()`의 마지막에서, 새 prior의 `frame_ids_`에 속한
프레임과 그 inertial state에 `set_lin_true()`를 호출한다. 이 지점이 **FEJ의 유일한
게이트**다. 그래서 `SVIOConfig::enable_fej = false`면 아무것도 동결되지 않고 전체가
기존의 순수 재선형화로 동작한다(A/B 비교용).

### 3.3 상대포즈 헬퍼 — `compute_rel_pose()`

Basalt의 `computeRelPose`에 해당한다. 다만 공식은 **우리 boxplus 컨벤션**
($t \mathrel{+}= \delta t$는 월드, $R \leftarrow R\,\mathrm{Exp}(\delta\theta)$는
바디)에 맞춰 새로 유도했다. Basalt는 rotation이 left-increment라 그대로 이식할 수
없기 때문이다:

$$d_{rel,h} = \begin{bmatrix} R_{c_t w} & -R_{c_t w} R_{w b_h} [\,t_{b c_h}\,]_\times \\ 0 & R_{c_h b_h} \end{bmatrix},\qquad
d_{rel,t} = \begin{bmatrix} -R_{c_t w} & R_{c_t b_t} [\,u\,]_\times \\ 0 & -R_{t\,h}^\top R_{c_t b_t} \end{bmatrix}$$

$$u = R_{w b_t}^\top (t_{w c_h} - t_{w b_t})$$

12개 열 전부가 수치미분과 일치함을 단위 테스트로 검증했다
(`test_fej.cpp: RelPoseJacobiansMatchNumeric`).

### 3.4 BearingCost의 합성

```
Evaluate(params):
  residual, J_rel  ← 현재 params의 T_t_h                  (항상 현재)
  d_rel_d_h/t      ← compute_rel_pose(lin-or-current 포즈)  (FEJ 지점)
  J_local          =  J_rel · d_rel_d_{h,t}
  jacobians[pose]  =  J_local · blkdiag(I, Jr(r_current))   (chart 변환)
```

마지막 chart 항 $J_r(r_{\text{current}})$는 **반드시 현재값**이어야 한다. Ceres가
PlusJacobian $J_r^{-1}(r_{\text{current}})$을 곱해 이 항을 상쇄시키기 때문이다
(배경은 marginalization_prior_math.md §1 참고). 정리하면, FEJ는 tangent 공간의
$J_{local}$에만 적용되고 ambient↔tangent 번역은 언제나 현재 지점에서 한다.

생성자는 `std::optional<Sophus::SE3d>` lin 포즈 2개를 받는다. 둘 다 nullopt면
기존과 비트 단위로 동일하게 동작한다(autodiff 회귀 테스트로 보증).

### 3.5 IMU 팩터

`ImuPreintegrationCost`는 상대포즈 구조가 아니라서 분해 대신 **상태 의존 야코비안
항만** lin에서 평가한다(`LinState` = 포즈 $i,j$ + 속도 $i,j$ + bias $i$):

- lin으로 평가: $R_{b_i w}$, $a_w$, $b_w$ (중력·속도 항), $e_{bg}$와 $r_{rot}$에서
  오는 $J_l^{-1}$ 항들
- 현재로 평가: residual 전체, chart 항 $J_r(r_i), J_r(r_j)$

### 3.6 세 경로의 선형화 일관성

FEJ가 효과를 내려면 같은 상태를 참조하는 모든 곳이 **같은 지점에서** 선형화해야
한다:

| 경로 | 방법 |
|---|---|
| `optimize_window`의 bearing/IMU 팩터 | lin 주입 (`make_imu_lin_state`, bearing 헬퍼) |
| `marginalize`의 Hessian 구축 | 같은 헬퍼 공유 (IMU는 autodiff → **analytic+lin으로 전환**) |
| prior 자체 | $x^0$·$J_p$를 tangent에서 저장하고 boxminus로 평가 (별도 문서 참고) |

여기서 marginalize 경로를 autodiff(현재점 선형화)에서 analytic+lin으로 바꾼 것이
중요하다. 그러지 않으면 marginalization을 할 때마다 FEJ와 어긋난 prior가 계속
주입된다.

## 4. 적용 범위와 한계

- **동결 대상**: prior에 편입된 프레임 포즈와 inertial state. 활성 프레임끼리의
  팩터는 계속 자유롭게 재선형화된다.
- **비-getter 유지**: `get_next_*` 류의 소비성 연산과 달리, FEJ 접근자는 순수
  조회다. 동결은 오직 marginalize에서만 일어난다.
- **랜드마크는 미동결**: host 프레임 기준 bearing/inv_dist는 lin을 갖지 않는다.
  상대 파라미터화라 gauge 누출 통로가 아니어서 우선순위를 낮췄다.
- **장수명 키프레임**: prior에 오래 남는 프레임은 lin 포즈가 현재 추정치에서 점점
  멀어질 수 있다. FEJ의 본질적 트레이드오프로(consistency를 위해 정확도 일부
  희생), 문제가 관측되면 anchor 재설정(relinearization epoch)을 고려한다.

## 5. 검증

- `test_fej.cpp`
  - `NoOpWhenLinEqualsCurrent`: lin == 현재면 기존 코스트와 완전히 동일
  - `JacobianPinnedResidualCurrent`: 포즈가 움직여도 residual은 따라가고 야코비안의
    chain 항만 고정
  - `RelPoseJacobiansMatchNumeric`: $d_{rel}$ 공식 vs 수치미분
- 기존 `test_cost_function_autodiff` / `test_imu_preintegration_cost`: lin 미주입
  시 회귀 없음
- EuRoC V1_01: `enable_fej` on/off 모두 정상 주행. (참고로, 도입 초기에 관측된
  윈도우 강체 점프는 FEJ가 아니라 prior의 chart 버그였다 —
  marginalization_prior_math.md §4.)
