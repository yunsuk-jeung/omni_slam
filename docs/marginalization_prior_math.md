# Marginalization Prior와 Manifold 좌표계 정리

`MarginalizationCost`가 tangent 공간의 정보를 ambient 좌표로 오용하던 버그(윈도우
강체 점프의 원인)의 수학적 배경 정리. 관련 코드:
`src/optimizer/marginalizer.hpp`, `src/optimizer/parameterization.hpp`,
`src/optimizer/vio_estimator.cpp`의 `marginalize()`.

표기: pose 파라미터 $x = (t, \varphi) \in \mathbb{R}^6$ (ambient),
$R = \mathrm{Exp}(\varphi)$, tangent 증분 $\delta = (\delta t, \delta\theta)$.

---

## 1. Manifold와 PlusJacobian

### Ambient와 Tangent란?

회전은 벡터가 아니다. 회전 두 개를 더한다고 회전이 되지 않고
($R_1 + R_2 \notin SO(3)$), "평균"도 그냥 성분 평균으로는 안 된다. 이런 집합을
**manifold**(곡면)라고 부른다 — 지구 표면처럼, 전체적으로는 휘어 있지만
어느 지점에서든 국소적으로는 평평한 공간처럼 다룰 수 있는 집합.

그런데 컴퓨터에 상태를 저장하려면 결국 실수 배열이 필요하다. 그래서 공간이
둘로 갈라진다:

**Ambient 공간** — manifold 위의 점을 표현하기 위해 **저장하는 숫자들**의 공간.
표현 방식은 선택의 문제다:

| 표현 | ambient 차원 | 특징 |
|---|---|---|
| 회전행렬 $R$ | 9 | 제약 6개 (직교성) |
| 쿼터니언 $q$ | 4 | 제약 1개 (단위노름) |
| log 좌표 $\varphi = \mathrm{Log}(R)$ | 3 | 제약 없음, 대신 눈금이 비선형 |

우리 코드는 pose를 $x = (t, \varphi) \in \mathbb{R}^6$으로 저장한다 — 세 번째 방식.
ambient는 말 그대로 "둘러싸는" 공간이다: manifold가 이 좌표 공간 안에 잠겨 있다.

**Tangent 공간** — manifold 위의 한 점 $x$에서 **실제로 움직일 수 있는 방향들**의
공간 (그 점에 붙인 접평면). 회전이라면 "지금 자세에서 바디 축으로 얼마나 더
도는가" $\delta\theta \in \mathbb{R}^3$. 지구 비유로: ambient가 위도·경도라면,
tangent는 "지금 서 있는 곳에서 북쪽으로 1 km, 동쪽으로 2 km"라는 실제 걸음이다.
최적화의 증분, 공분산, Hessian이 모두 사는 곳이 여기다 — **물리적 의미를 갖는
"변화량"은 전부 tangent 소속**이다.

둘이 왜 어긋나는가: ambient 좌표의 눈금이 균일하지 않기 때문이다. 극지방에서
경도 1도의 실제 거리가 적도와 다르듯, $\varphi$가 큰 곳에서는 "$\varphi$ 숫자
1단위 변화"와 "실제 회전 1 rad"이 서로 다르다 (아래 §5 수치 예시).
둘 사이의 국소 환산이 바로 PlusJacobian이다.

**Boxplus** (`SE3BoxplusManifold`):

$$x \boxplus \delta = \big(\,t + \delta t,\;\; \mathrm{Log}(\mathrm{Exp}(\varphi)\,\mathrm{Exp}(\delta\theta))\,\big)$$

BCH 1차 전개:

$$\mathrm{Log}(\mathrm{Exp}(\varphi)\,\mathrm{Exp}(\delta\theta)) = \varphi + J_r^{-1}(\varphi)\,\delta\theta + O(\|\delta\theta\|^2)$$

따라서 **PlusJacobian**:

$$P(x) \;=\; \frac{\partial (x \boxplus \delta)}{\partial \delta}\Big|_{\delta=0} \;=\; \begin{bmatrix} I_3 & 0 \\ 0 & J_r^{-1}(\varphi) \end{bmatrix}$$

### PlusJacobian이 ambient와 tangent를 잇는 방식

두 공간과 그 사이의 사상을 그림으로 놓으면:

```
   tangent δ ∈ ℝ⁶  ──── x ⊞ δ ────▶  ambient x' ∈ ℝ⁶
   (진짜 증분)        1차 근사:         (저장된 숫자)
                    x' ≈ x + P(x)·δ
```

즉 $P(x)$는 **"tangent 방향 단위 증분이 ambient 좌표를 어느 방향으로 얼마나
움직이는가"** 를 열(column)마다 담은 행렬이다. $P$의 $j$번째 열
$= \lim_{h\to0} \tfrac{1}{h}\big((x \boxplus h e_j) - x\big)$ — tangent 기저
$e_j$로 한 발 디뎠을 때의 ambient 변위.

이제 residual $r(x)$를 생각하자. 코스트 함수가 반환하는 것은 ambient 기울기:

$$J_a = \frac{\partial r}{\partial x}$$

하지만 최적화 변수는 $\delta$다 — 상태는 반드시 $x(\delta) = x^{\text{cur}} \boxplus \delta$
형태로만 움직이기 때문. 따라서 실제로 필요한 미분은 합성함수
$r\big(x(\delta)\big)$의 것이고, 체인룰로:

$$J_t = \frac{\partial r}{\partial \delta}\Big|_{\delta=0} = \frac{\partial r}{\partial x}\cdot\frac{\partial x(\delta)}{\partial \delta}\Big|_{\delta=0} = J_a\,P(x)$$

**$P$는 "ambient 언어로 말한 기울기 $J_a$"를 "tangent 언어의 기울기 $J_t$"로
번역하는 접착제**다. 방향에 주의:

| 변환 | 행렬 | 비고 |
|---|---|---|
| tangent 증분 → ambient 변위 | $P(x) = \mathrm{blkdiag}(I, J_r^{-1})$ | PlusJacobian의 정의 |
| ambient 야코비안 → tangent 야코비안 | $J_t = J_a\,P$ | 오른쪽 곱 (열이 변수 방향) |
| ambient 변위 → tangent 증분 | $P^{-1} = \mathrm{blkdiag}(I, J_r)$ | boxminus의 1차 근사 |
| tangent 야코비안 → ambient 야코비안 | $J_a = J_t\,P^{-1}$ | 이번 수정에서 쓴 방향 |

마지막 행이 이번 버그 수정의 핵심이다: 우리가 가진 prior의 $J_p$는 tangent
야코비안이므로, Ceres에 돌려주기 전에 $P^{-1}$ (회전 열에 $J_r(\varphi)$)을 곱해
ambient로 되돌려 놓아야 Ceres가 다시 $P$를 곱했을 때 원래의 $J_p$가 복원된다.

한 가지 흔한 오해: "ambient와 tangent는 차원이 다르다"는 인상 (쿼터니언 4→3 같은
경우). 차원이 다르면 실수를 컴파일/차원 검사에서 잡을 수 있지만, 우리처럼
**둘 다 6차원이면 $P$가 정칙 정방행렬이라 어떤 조합도 계산은 돌아가고, 값만
조용히 틀린다.** 이번 버그가 오래 숨어있던 이유다.

### Ceres의 계약

- 코스트 함수의 `Evaluate`는 **ambient 야코비안** $J_a = \partial r/\partial x$를
  반환한다.
- Ceres는 내부에서 $J_t = J_a\,P(x)$로 **tangent 야코비안**을 만들고, tangent에서
  $\min_\delta \tfrac12\|r + J_t\,\delta\|^2$ (+ damping)을 푼 뒤
  $x \leftarrow x \boxplus \delta$로 갱신한다.
- **`Problem::Evaluate`가 반환하는 야코비안도 $J_t$다** (ceres `problem.h` Note 2:
  manifold 블록은 TangentSize 열). 우리 pose는 ambient = tangent = 6이라
  **차원만 봐서는 구분이 안 된다.**

### Boxminus와 그 미분

$$x \boxminus x^0 = \big(\,t - t^0,\;\; \mathrm{Log}(\mathrm{Exp}(\varphi^0)^{-1}\mathrm{Exp}(\varphi))\,\big)$$

$f(\varphi) = \mathrm{Log}(\mathrm{Exp}(\varphi^0)^{-1}\mathrm{Exp}(\varphi))$에
$\mathrm{Exp}(\varphi+\varepsilon) = \mathrm{Exp}(\varphi)\,\mathrm{Exp}(J_r(\varphi)\varepsilon) + O(\varepsilon^2)$를
대입하면:

$$\frac{\partial f}{\partial \varphi} = J_r^{-1}\!\big(f(\varphi)\big)\, J_r(\varphi) \;\;\xrightarrow{\;x \to x^0\;}\;\; J_r(\varphi)$$

특히 $\varphi = \varphi^0 + \varepsilon$이면 $f = J_r(\varphi^0)\,\varepsilon + O(\varepsilon^2)$:

$$\boxed{\;\delta\theta_{\text{true}} = J_r(\varphi^0)\,(\varphi - \varphi^0)\;}$$

**raw 좌표 차이와 진짜 tangent 증분은 $J_r$ 한 배 차이다.**

---

## 2. Marginalization 유도

marginalize 시점 상태를 $x^0 = (x^0_m, x^0_k)$ (m = marginalize, k = keep)라 하자.
모든 팩터의 residual을 쌓아 $\bar r$, tangent 야코비안을 $\bar J_t$라 하면
($x^0$에서 선형화), Gauss–Newton 2차 모델은 tangent 변수
$\delta = (\delta_m, \delta_k)$에 대해:

$$\tfrac12\|\bar r + \bar J_t \delta\|^2 = \text{const} + b^\top\delta + \tfrac12 \delta^\top H \delta,\qquad H = \bar J_t^\top \bar J_t,\;\; b = \bar J_t^\top \bar r$$

(`create_hessian_from_crs_matrix`가 만드는 $H$, $b$가 정확히 이것 — **tangent 기준**.)

$\delta_m$에 대해 최소화하면 $\delta_m^\ast = -H_{mm}^{-1}(b_m + H_{mk}\,\delta_k)$,
대입하면 $\delta_k$만의 함수:

$$\tfrac12\,\delta_k^\top A\,\delta_k + b^{\ast\top}\delta_k + \text{const}$$

$$A = H_{kk} - H_{km}H_{mm}^{-1}H_{mk},\qquad b^\ast = b_k - H_{km}H_{mm}^{-1}b_m \quad\text{(Schur complement)}$$

Ceres 팩터로 쓰기 위한 제곱꼴 분해: $A = V\Lambda V^\top$에서

$$J_p = \Lambda^{1/2}V^\top,\qquad r_0 = \Lambda^{-1/2}V^\top b^\ast$$

로 두면 $J_p^\top J_p = A$, $J_p^\top r_0 = b^\ast$이므로 아래 팩터가 같은
gradient·Hessian을 재현한다:

$$\boxed{\;\text{prior cost} = \tfrac12\big\| \,r_0 + J_p\,\delta_k\, \big\|^2,\qquad \delta_k = x_k \boxminus x_k^0\;}$$

**핵심: $J_p$가 tangent에서 만들어졌으므로 $\delta_k$도 tangent 증분(boxminus)이어야
한다.** (고유값이 $\epsilon$ 이하인 방향 — gauge null-space — 은 0으로 클리핑되어
prior가 그 방향으로는 어떤 gradient도 만들지 않는다.)

---

## 3. 이후 solve에서의 올바른 Evaluate

Ceres에는 ambient 야코비안을 반환해야 하므로 체인룰:

$$\frac{\partial r_p}{\partial x} = J_p \cdot D(x),\qquad D(x) = \frac{\partial (x \boxminus x^0)}{\partial x} = \begin{bmatrix} I & 0\\ 0 & J_r^{-1}(f)\,J_r(\varphi)\end{bmatrix} \approx \begin{bmatrix} I & 0\\ 0 & J_r(\varphi)\end{bmatrix}$$

($f \approx 0$ 근사는 prior 자체가 1차 모델인 것과 같은 급 — `SE3BoxplusManifold::MinusJacobian`과 동일 관례.)

Ceres가 내부에서 만드는 유효 tangent 야코비안:

$$J_t^{\text{eff}} = \big(J_p\, D(x)\big)\, P(x) = J_p \begin{bmatrix} I & 0\\ 0 & J_r(\varphi)\,J_r^{-1}(\varphi)\end{bmatrix} = J_p \;\checkmark$$

저장된 tangent 정보가 정확히 복원된다. 이것이 수정된 `MarginalizationCost::Evaluate`:

- pose 블록의 $dx$: raw 차이 대신 **boxminus** $[\,t-t_0,\; \mathrm{Log}(R_0^{-1}R)\,]$
- 반환 야코비안: 회전 열에 $J_r(\varphi_{\text{current}})$를 곱해 ambient로 변환

---

## 4. 기존 코드의 오류를 같은 수식으로

**Residual** — $\delta_k$ 자리에 raw 차이 사용:

$$r_p^{\text{old}} = r_0 + J_p\,(x - x^0) = r_0 + J_p \begin{bmatrix} I & 0\\ 0 & J_r^{-1}(\varphi^0)\end{bmatrix}\delta_{\text{true}} + O(\delta^2)$$

스프링이 $J_r^{-1}(\varphi^0)$만큼 왜곡된 자로 잰 변위에 벌점을 매긴다.

**Jacobian** — $J_p$를 그대로 반환하여 Ceres가 $P$를 곱함:

$$J_t^{\text{eff,old}} = J_p\,P(x) = J_p \begin{bmatrix} I & 0\\ 0 & J_r^{-1}(\varphi)\end{bmatrix}$$

**이중 왜곡의 자기모순** — 이 팩터의 gradient:

$$g^{\text{old}} = (J_t^{\text{eff,old}})^\top r_p^{\text{old}} = \begin{bmatrix} I & 0\\ 0 & J_r^{-\top}(\varphi)\end{bmatrix} J_p^\top\Big(r_0 + J_p \begin{bmatrix} I & 0\\ 0 & J_r^{-1}(\varphi^0)\end{bmatrix}\delta\Big)$$

정답 $g = J_p^\top(r_0 + J_p\,\delta)$ 대비, residual 쪽 왜곡은 $J_r^{-1}(\varphi^0)$,
gradient 사영 쪽 왜곡은 $J_r^{-\top}(\varphi)$ — **서로 다른 두 왜곡**이 겹친다.

- $\varphi \to 0$이면 $J_r \to I$라 무해 (초기 구간에서 멀쩡했던 이유).
- $\|\varphi\|$가 클수록 $J_r = I - \tfrac{1-\cos\theta}{\theta^2}[\varphi]_\times + \tfrac{\theta-\sin\theta}{\theta^3}[\varphi]_\times^2$의
  비항등 항이 커져 벌점의 크기·방향이 모두 틀어진다 ($\theta = 90°$에서 ~36%).
- 이 틀린 힘 중 뻣뻣한 방향 성분은 다른 팩터가 상쇄하지만, $A$의
  null-space(전역 병진·yaw) 성분은 저항 없이 상태를 밀어낸다 → **윈도우 강체 점프**
  (EuRoC V1_01의 빠른 회전 구간에서 관측된 현상).

---

## 5. 직관용 수치 예시

$\varphi^0 = [0, 0, \tfrac{\pi}{2}]$ (z축 90°)에서 바디 x축으로
$\delta\theta = [0.1, 0, 0]$ rad 회전하면:

| 량 | 값 |
|---|---|
| $\varphi_{\text{new}} = \mathrm{Log}(\mathrm{Exp}(\varphi^0)\mathrm{Exp}(\delta\theta))$ | $[0.0785,\ 0.0785,\ -0.0014]$ |
| raw 차이 $\varphi_{\text{new}} - \varphi^0$ | $[0.0785,\ 0.0785,\ -0.0014]$ |
| boxminus $\mathrm{Log}(R_0^\top R)$ | $[0.1,\ 0,\ 0]$ ✓ |

x축으로만 걸었는데 raw 차이는 x·y가 섞이고 크기도 21% 작다.
$J_r(\varphi^0)$의 상단 2×2 블록이 $\begin{bmatrix}0.64 & 0.64\\ -0.64 & 0.64\end{bmatrix}$로
항등과 크게 다른 것이 그 이유이며, $J_r(\varphi^0) \cdot (\text{raw 차이}) = [0.1, 0, -0.001]$로
진짜 걸음이 복원된다.

---

## 참고

- Ceres `problem.h` — `Problem::Evaluate` Note 2 (tangent-size jacobian)
- Forster et al., *On-Manifold Preintegration* (right Jacobian 정의)
- 수정 검증: EuRoC V1_01에서 강체 점프 3회(최대 12.9 cm) → 0회, 평균 prior cost 절반 감소
  (2026-07-16, `[wopt]` 계측 로그)
