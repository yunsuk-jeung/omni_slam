# Marginalization과 Manifold 좌표계 (통합 정리)

이 문서는 pose manifold의 ambient/tangent 좌표, Ceres와의 야코비안 계약,
그리고 marginalization prior의 유도·구현·과거 버그를 한곳에 모은 것이다.
(이전의 `marginalization_prior_math.md`, `Ceres_Manifold_Marginalization_Notes.md`,
`marginalization_tangent_ambient_walkthrough.md`를 통합.)

**관련 코드**

- `src/optimizer/parameterization.hpp` — `SE3BoxplusManifold`
- `src/optimizer/marginalizer.hpp` — `MarginalizationCost::Evaluate`
- `src/optimizer/vio_estimator.cpp` — `marginalize()`
- `src/optimizer/cost_function.hpp` — reprojection/bearing cost의 야코비안

**표기**: 회전행렬 $R$, 그 log 좌표 $\omega = \mathrm{Log}(R) \in \mathbb{R}^3$.
pose 파라미터 $x = (t, \omega) \in \mathbb{R}^6$ (ambient), tangent 증분
$\delta = (\delta t, \delta\theta)$, residual은 $e$ (회전 $\omega$와의 혼동 방지).

---

## 1. Parameterization — 무엇을 저장하는가

저장되는 pose는

- translation $t$
- rotation $\omega = \mathrm{Log}(R)$

이며 파라미터는 $x = [t, \omega] \in \mathbb{R}^6$. 하지만 업데이트는 단순
덧셈이 아니라 오른쪽 섭동을 쓴다:

$$R_{\text{new}} = R\,\mathrm{Exp}(\delta\theta), \qquad t_{\text{new}} = t + \delta t$$

이 "저장 좌표"와 "업데이트 증분"의 괴리가 이 문서 전체의 주제다.

---

## 2. Ambient와 Tangent

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
| log 좌표 $\omega = \mathrm{Log}(R)$ | 3 | 제약 없음, 대신 눈금이 비선형 |

우리 코드는 pose를 $x = (t, \omega) \in \mathbb{R}^6$으로 저장한다 — 세 번째 방식.
ambient는 말 그대로 "둘러싸는" 공간이다: manifold가 이 좌표 공간 안에 잠겨 있다.

**Tangent 공간** — manifold 위의 한 점 $x$에서 **실제로 움직일 수 있는 방향들**의
공간 (그 점에 붙인 접평면). 회전이라면 "지금 자세에서 바디 축으로 얼마나 더
도는가" $\delta\theta \in \mathbb{R}^3$. 지구 비유로: ambient가 위도·경도라면,
tangent는 "지금 서 있는 곳에서 북쪽으로 1 km, 동쪽으로 2 km"라는 실제 걸음이다.
최적화의 증분, 공분산, Hessian이 모두 사는 곳이 여기다 — **물리적 의미를 갖는
"변화량"은 전부 tangent 소속**이다.

둘이 왜 어긋나는가: ambient 좌표의 눈금이 균일하지 않기 때문이다. 극지방에서
경도 1도의 실제 거리가 적도와 다르듯, $\omega$가 큰 곳에서는 "$\omega$ 숫자
1단위 변화"와 "실제 회전 1 rad"이 서로 다르다 (아래 §7 수치 예시).
둘 사이의 국소 환산이 바로 PlusJacobian이다.

### 예제 모음: 여러 manifold에서 본 ambient vs tangent

같은 구분이 어디서나 반복된다. 익숙한 것부터:

| manifold | ambient (저장) | tangent (증분) | 함정 |
|---|---|---|---|
| $\mathbb{R}^n$ (병진, bias) | $\mathbb{R}^n$ | $\mathbb{R}^n$, $\boxplus = +$ | 없음 — $P = I$ |
| $SO(2)$ 각도 $\theta$ | $\mathbb{R}^1$ (wrap) | $\mathbb{R}^1$ | 없음 — 가환이라 $J_r = 1$ |
| 단위원 $S^1$: $(\cos\theta, \sin\theta)$ | $\mathbb{R}^2$ | $\mathbb{R}^1$ | 덧셈하면 원 이탈; 차원이 달라 실수가 드러남 |
| 지구 표면: 위도·경도 | $\mathbb{R}^2$ (deg) | $\mathbb{R}^2$ (km) | 경도 눈금이 위도에 따라 변함 |
| 쿼터니언 $q$ | $\mathbb{R}^4$ | $\mathbb{R}^3$ | raw 차이 $q - q^0$는 무의미; $P$가 4×3 |
| 단위 중력벡터 $S^2$ | $\mathbb{R}^3$ | $\mathbb{R}^2$ | 3성분을 자유 최적화하면 $\|g\|$가 9.81에서 이탈 |
| $SO(3)$ log 좌표 $\omega$ | $\mathbb{R}^3$ | $\mathbb{R}^3$ | **같은 차원인데 눈금이 비선형** — 이번 버그 |

**예제 1 — $\mathbb{R}^n$: 왜곡이 없는 기준점.** 병진 $t$나 IMU bias가 여기
산다. $x \boxplus \delta = x + \delta$, raw 차이 = tangent 증분, $P = I$.
"ambient vs tangent 구분이 없다"기보다 **둘이 우연히 일치하는 특수한 manifold**다.
우리 pose의 병진 블록에서 $P$의 좌상단이 $I_3$인 이유.

**예제 2 — $SO(2)$: 같은 차원인데도 무해한 경우.** 평면 로봇의 헤딩을 각도
$\theta$ 하나로 저장하면 ambient = tangent = 1차원이고, 회전이 가환이라

$$\mathrm{Log}\big(\mathrm{Exp}(\theta^0)\,\mathrm{Exp}(\delta)\big) = \theta^0 + \delta \quad\text{(정확히!)}$$

raw 차이가 곧 boxminus이고 $J_r \equiv 1$이다. **이번 버그는 2D SLAM에서는
아예 존재할 수 없다** — $J_r \ne I$는 $SO(3)$의 비가환성
($R_1 R_2 \ne R_2 R_1$)에서 오는 순수 3D 현상이다. 2D 직관이 3D에서
배신하는 지점.

**예제 3 — 단위원 $S^1$을 $\mathbb{R}^2$에 담기.** 헤딩을
$x = (\cos\theta, \sin\theta)$로 저장하면 ambient는 2차원, 실제 자유도는
1차원. $\theta = 0$에서 tangent 증분 $\delta = 0.1$ rad을 밟으면:

- 올바른 갱신: $x \boxplus \delta = (\cos 0.1, \sin 0.1) = (0.995, 0.0998)$ — 원 위에 남음
- ambient 덧셈: $x + (0, 0.1) = (1, 0.1)$ — 노름 1.005, **원을 이탈**

PlusJacobian은 $P(x) = (-\sin\theta, \cos\theta)^\top$, 2×1 행렬 — 접선 방향
단위벡터 그 자체다. 차원이 2→1로 다르므로 $J_a$와 $J_t$를 헷갈리면 행렬
크기가 안 맞아 **즉시 들킨다**. 우리 6→6 케이스와의 결정적 차이.

**예제 4 — 위도·경도: 눈금 왜곡의 원형.** ambient 좌표 (위도 $\phi$, 경도
$\lambda$)에서 tangent는 "북쪽으로 몇 km, 동쪽으로 몇 km". 국소 환산은

$$\begin{bmatrix}\text{북 (km)}\\ \text{동 (km)}\end{bmatrix} = \underbrace{\begin{bmatrix}111.2 & 0\\ 0 & 111.2\cos\phi\end{bmatrix}}_{P(\phi)\;\text{(km/deg)}}\begin{bmatrix}\Delta\phi\\ \Delta\lambda\end{bmatrix}$$

적도에서 경도 1° = 111.2 km, 위도 60°에서는 55.6 km — **같은 ambient 변화가
위치에 따라 다른 물리 변위**다. "경도 오차 ±1°에 벌점"이라는 스프링(prior)을
달면 적도의 로봇과 북극 근처의 로봇이 전혀 다른 세기로 당겨진다. raw 좌표
차이에 $J_p$를 곱하던 기존 코드가 정확히 이 짓을 한 것.

**예제 5 — 쿼터니언: 차원 불일치가 지켜주는 경우.** $q \in \mathbb{R}^4$,
자유도 3. 항등 자세에서 x축 0.2 rad 회전하면
$q = (\cos 0.1, \sin 0.1, 0, 0) = (0.995, 0.0998, 0, 0)$이고

$$q - q^0 = (-0.005,\; 0.0998,\; 0,\; 0)$$

4개 숫자 차이는 3자유도 운동을 담기에 과잉이고, $w$ 성분의 $-0.005$는 어떤
물리적 회전 축에도 대응하지 않는다 (단위노름 제약 방향의 흔적). Ceres에서
쿼터니언 manifold를 쓰면 $P$가 4×3이라 tangent/ambient를 섞는 실수가 차원
검사에서 잡힌다. **쿼터니언에서는 "차원이 다르다"가 사실이지만, log 좌표에서는
아니다** — 우리 케이스가 조용히 틀리는 이유.

**예제 6 — 단위 중력벡터 $S^2$: VIO 초기화의 실전 사례.** 중력 방향
$g \in \mathbb{R}^3$, $\|g\| = 9.81$ 제약으로 자유도 2. tangent 증분은 $g$에
수직인 평면의 2차원 벡터 $\delta \in \mathbb{R}^2$이고
$g \boxplus \delta = \mathrm{Exp}\big(B(g)\,\delta\big)\,g$ ($B(g)$는 $g^\perp$의
3×2 기저). 세 성분을 그냥 자유 변수로 최적화하면 스케일 자유도가 중력 크기로
새어 들어가 $\|g\|$가 9.81에서 표류한다 — "제약이 있는 ambient에서 무제약
최적화를 하면 안 된다"는 것의 가장 흔한 VIO 사례.

### Boxplus와 PlusJacobian

**Boxplus** (`SE3BoxplusManifold`):

$$x \boxplus \delta = \big(\,t + \delta t,\;\; \mathrm{Log}(\mathrm{Exp}(\omega)\,\mathrm{Exp}(\delta\theta))\,\big)$$

BCH 1차 전개:

$$\mathrm{Log}(\mathrm{Exp}(\omega)\,\mathrm{Exp}(\delta\theta)) = \omega + J_r^{-1}(\omega)\,\delta\theta + O(\|\delta\theta\|^2)$$

따라서 **PlusJacobian**:

$$P(x) \;=\; \frac{\partial (x \boxplus \delta)}{\partial \delta}\Big|_{\delta=0} \;=\; \begin{bmatrix} I_3 & 0 \\ 0 & J_r^{-1}(\omega) \end{bmatrix}$$

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

이제 residual $e(x)$를 생각하자. 코스트 함수가 반환하는 것은 ambient 기울기:

$$J_a = \frac{\partial e}{\partial x}$$

하지만 최적화 변수는 $\delta$다 — 상태는 반드시 $x(\delta) = x^{\text{cur}} \boxplus \delta$
형태로만 움직이기 때문. 따라서 실제로 필요한 미분은 합성함수
$e\big(x(\delta)\big)$의 것이고, 체인룰로:

$$J_t = \frac{\partial e}{\partial \delta}\Big|_{\delta=0} = \frac{\partial e}{\partial x}\cdot\frac{\partial x(\delta)}{\partial \delta}\Big|_{\delta=0} = J_a\,P(x)$$

**$P$는 "ambient 언어로 말한 기울기 $J_a$"를 "tangent 언어의 기울기 $J_t$"로
번역하는 접착제**다. 방향에 주의:

| 변환 | 행렬 | 비고 |
|---|---|---|
| tangent 증분 → ambient 변위 | $P(x) = \mathrm{blkdiag}(I, J_r^{-1})$ | PlusJacobian의 정의 |
| ambient 야코비안 → tangent 야코비안 | $J_t = J_a\,P$ | 오른쪽 곱 (열이 변수 방향) |
| ambient 변위 → tangent 증분 | $P^{-1} = \mathrm{blkdiag}(I, J_r)$ | boxminus의 1차 근사 |
| tangent 야코비안 → ambient 야코비안 | $J_a = J_t\,P^{-1}$ | prior 수정에서 쓴 방향 |

마지막 행이 prior 버그 수정의 핵심이다: prior의 $J_p$는 tangent 야코비안이므로,
Ceres에 돌려주기 전에 $P^{-1}$ (회전 열에 $J_r(\omega)$)을 곱해 ambient로 되돌려
놓아야 Ceres가 다시 $P$를 곱했을 때 원래의 $J_p$가 복원된다.

**둘 다 6차원이면 $P$가 정칙 정방행렬이라 어떤 조합도 계산은 돌아가고, 값만
조용히 틀린다.** 쿼터니언(4→3)이었다면 차원 검사에서 잡혔을 실수가 log 좌표에서
오래 숨어있던 이유다.

### Ceres의 계약

- 코스트 함수의 `Evaluate`는 **ambient 야코비안** $J_a = \partial e/\partial x$를
  반환한다.
- Ceres는 내부에서 $J_t = J_a\,P(x)$로 **tangent 야코비안**을 만들고, tangent에서
  $\min_\delta \tfrac12\|e + J_t\,\delta\|^2$ (+ damping)을 푼 뒤
  $x \leftarrow x \boxplus \delta$로 갱신한다.
- **`Problem::Evaluate`가 반환하는 야코비안도 $J_t$다** (ceres `problem.h` Note 2:
  manifold 블록은 TangentSize 열). 우리 pose는 ambient = tangent = 6이라
  **차원만 봐서는 구분이 안 된다.**

### Boxminus와 그 미분

$$x \boxminus x^0 = \big(\,t - t^0,\;\; \mathrm{Log}(\mathrm{Exp}(\omega^0)^{-1}\mathrm{Exp}(\omega))\,\big)$$

$f(\omega) = \mathrm{Log}(\mathrm{Exp}(\omega^0)^{-1}\mathrm{Exp}(\omega))$에
$\mathrm{Exp}(\omega+\varepsilon) = \mathrm{Exp}(\omega)\,\mathrm{Exp}(J_r(\omega)\varepsilon) + O(\varepsilon^2)$를
대입하면:

$$\frac{\partial f}{\partial \omega} = J_r^{-1}\!\big(f(\omega)\big)\, J_r(\omega) \;\;\xrightarrow{\;x \to x^0\;}\;\; J_r(\omega)$$

특히 $\omega = \omega^0 + \varepsilon$이면 $f = J_r(\omega^0)\,\varepsilon + O(\varepsilon^2)$:

$$\boxed{\;\delta\theta_{\text{true}} = J_r(\omega^0)\,(\omega - \omega^0)\;}$$

**raw 좌표 차이와 진짜 tangent 증분은 $J_r$ 한 배 차이다.**

---

## 3. Cost Jacobian vs Reprojection Jacobian

CostFunction이 반환해야 하는 것은 ambient 야코비안 $\partial e/\partial x$이고,
Manifold는 $P = \partial x/\partial\delta$를 제공하며, Ceres 내부에서
$J_{\text{solver}} = J_{\text{cost}}\,P$를 쓴다 (§2 Ceres 계약). 우리 manifold에서
$P$의 회전 블록은 $J_r^{-1}(\omega)$다.

구체 예로 reprojection/bearing residual. tangent 섭동
$R' = R\,\mathrm{Exp}(\delta\theta)$ 기준으로 미분하면 ($J_\pi$는 투영 야코비안,
$P$는 3D 점):

$$\frac{\partial e}{\partial\delta\theta} = -\,J_\pi\,R\,[P]_\times$$

같은 residual을 ambient 파라미터 $\omega$로 미분하면 오른쪽에 $J_r$이 붙는다:

$$\frac{\partial e}{\partial\omega} = -\,J_\pi\,R\,[P]_\times\,J_r(\omega)$$

즉 **Ambient Jacobian = Tangent Jacobian × $J_r$**. `cost_function.hpp`의 bearing
cost가 마지막에 `J = ... * J_r`을 곱해서 돌려주는 것이 정확히 이것이다 (그리고
Ceres가 $P = J_r^{-1}$을 곱해 tangent로 되돌린다).

---

## 4. Marginalization: 유도와 tangent/ambient 수명주기

marginalize 시점 상태를 $x^0 = (x^0_m, x^0_k)$ (m = marginalize, k = keep)라 하고,
FEJ 선형화점으로 고정한다. 아래는 한 사이클 전체를 각 객체가 tangent인지 ambient인지
표시하며 따라간 것이다.

### 1단계 — 선형화: ambient residual, tangent 야코비안

각 팩터 residual $e_i(x)$의 코스트 함수는 **ambient 야코비안**
$J_a^{(i)} = \partial e_i/\partial x|_{x^0}$를 반환한다. 그런데 `problem.Evaluate`가
꺼내오는 `ceres_J`는 이미 **tangent 야코비안**이다 (Ceres가 $P$를 곱해 둠):

$$\bar J_t = J_a\,P(x^0) \qquad [\text{tangent}]$$

`create_hessian_from_crs_matrix`가 이걸로 만드는 것:

$$H = \bar J_t^\top \bar J_t, \qquad b = \bar J_t^\top \bar e \qquad [\text{둘 다 tangent}]$$

이것이 tangent 변수 $\delta = (\delta_m, \delta_k)$에 대한 Gauss–Newton 2차 모델:

$$\tfrac12\|\bar e + \bar J_t \delta\|^2 = \text{const} + b^\top\delta + \tfrac12 \delta^\top H \delta$$

그 뒤 대칭화 $H \leftarrow \tfrac12(H + H^\top)$. **여기서 ambient는 끝났다** —
$x^0$는 고정 상수라 미분에서 사라지고, 남은 것은 전부 tangent $\delta$의 세계다.

### 2단계 — 분할과 Schur complement (전부 tangent)

$$
H = \begin{bmatrix} H_{mm} & H_{mk}\\ H_{km} & H_{kk}\end{bmatrix},
\qquad
b = \begin{bmatrix} b_m \\ b_k\end{bmatrix}
$$

$\delta_m$에 대해 최소화하면 $\delta_m^\ast = -H_{mm}^{-1}(b_m + H_{mk}\,\delta_k)$,
대입하면 $\delta_k$만의 함수 $\tfrac12\,\delta_k^\top A\,\delta_k + b^{\ast\top}\delta_k + \text{const}$:

$$A = H_{kk} - H_{km}H_{mm}^{-1}H_{mk},\qquad b^\ast = b_k - H_{km}H_{mm}^{-1}b_m \quad\text{(Schur complement)}$$

$H_{mm}^{-1}$은 역행렬이 아니라 **고윳값 pseudo-inverse** ($\lambda \le 10^{-8}$
클리핑)로 gauge 방향 특이성을 흡수한다. 다시 대칭화. $A$, $b^\ast$ 모두 여전히
**keep 블록의 tangent 좌표**에 산다.

### 3단계 — 제곱근 prior와 선형화점 저장

Ceres 팩터로 쓰기 위한 제곱꼴 분해: $A = V\Lambda V^\top$에서

$$J_p = \Lambda^{1/2}V^\top \ \Rightarrow\ J_p^\top J_p = A,\qquad e_0 = \Lambda^{-1/2}V^\top b^\ast \ \Rightarrow\ J_p^\top e_0 = b^\ast$$

$\lambda \le 10^{-8}$인 방향은 $\sqrt\lambda = 0$, $1/\sqrt\lambda = 0$으로 죽여서
**null-space(전역 병진·yaw)에는 prior가 어떤 gradient도 만들지 않게** 클리핑한다.
동시에 **선형화점** $x^0_k$ (keep 블록의 ambient 저장값 $(t, \omega)$)를 저장한다.
결국 저장물은 $\{\,J_p,\ e_0,\ x^0_k,\ \text{block\_sizes}\,\}$.

이로써 아래 팩터가 원래의 gradient·Hessian을 재현한다:

$$\boxed{\;\text{prior cost} = \tfrac12\big\| \,e_0 + J_p\,\delta_k\, \big\|^2,\qquad \delta_k = x_k \boxminus x_k^0\;}$$

**핵심: $J_p$가 tangent에서 만들어졌으므로 $\delta_k$도 tangent 증분(boxminus)이어야
한다.**

### 4단계 — 이후 solve에서의 재평가 (`MarginalizationCost::Evaluate`)

#### (a) residual — tangent 증분을 boxminus로 계산

$J_p$가 tangent이므로 $\delta_k$도 tangent 증분이어야 한다. raw 차이가 아니라
boxminus:

$$\delta_k = x_k \boxminus x^0_k = \begin{bmatrix} t - t^0 \\ \mathrm{Log}(R_0^{-1}R)\end{bmatrix} \qquad [\text{tangent}], \qquad e_{\text{prior}}(x) = e_0 + J_p\,\delta_k$$

##### 왜 $\mathrm{Log}(R_0^{-1}R)$가 ambient가 아니라 tangent인가

핵심은 "log를 취했느냐"가 아니라 **"무엇을 기준으로 잰 log냐"**다. log가 두 개
등장하는데 사는 곳이 다르다.

- **Ambient 좌표** $\omega = \mathrm{Log}_I(R)$ — **항등원 기준**의 log. 절대 자세
  $R$ 하나에 숫자 3개를 붙이는 이름표(좌표). 기준점이 필요 없고 저장값 그 자체.
- **Tangent 증분** $\mathrm{Log}(R_0^{-1}R)$ — **기준점 $R_0$ 기준**의 log:
  $\delta\theta = \mathrm{Log}(R_0^{-1}R) \Leftrightarrow R = R_0\,\mathrm{Exp}(\delta\theta)$.
  $R_0$에 붙인 접평면 $T_{R_0}SO(3)$의 원소이자 boxplus의 증분.

| | ambient 좌표 $\omega$ | tangent 증분 $\mathrm{Log}(R_0^{-1}R)$ |
|---|---|---|
| 기준점 | 없음 (항등원) | $R_0$에 매여 있음 |
| 의미 | 절대 자세의 이름 | $R_0$로부터의 변화량 |
| 덧셈 | 무의미 | 의미 있음 (벡터공간) |
| $H$·공분산·$J_p$가 사는 곳 | ✗ | ✓ |

"tangent"는 특정 점에 붙은 접공간의 원소이자 물리적 증분이라는 뜻이다.
$\mathrm{Log}(R_0^{-1}R)$를 "ambient 좌표"로 해석하면 $R$의 좌표가 아니라 **상대
회전 $R_0^{-1}R$라는 다른 자세의 좌표**가 되어 버린다 — 원하는 대상이 아니다.

**결정적 이유 — $J_p$가 요구하는 인자.** $J_p$는 $\bar J_t = J_a P$, 즉
$\delta\theta$($R = R_0\mathrm{Exp}(\delta\theta)$의 그 증분)에 대한 미분에서
만들어졌다. 그러니 $J_p$에 곱해 넣는 값도 반드시 같은 $\delta\theta$여야 하고,
$\mathrm{Log}(R_0^{-1}R)$가 정확히 그 $\delta\theta$라 일관된다. 대비로 ambient
끼리의 뺄셈 $\omega - \omega^0$은 좌표 두 개를 그냥 뺀 값이라 어떤 접공간에도
속하지 않는다 (§2 boxminus 미분: 둘은 $J_r(\omega^0)$ 한 배 차이).

#### (b) 야코비안 — Ceres에는 ambient로 돌려줘야 함

체인룰:

$$\frac{\partial e_{\text{prior}}}{\partial x} = J_p \cdot D(x),\qquad D(x) = \frac{\partial (x \boxminus x^0)}{\partial x} = \begin{bmatrix} I & 0\\ 0 & J_r^{-1}(f)\,J_r(\omega)\end{bmatrix} \approx \begin{bmatrix} I & 0\\ 0 & J_r(\omega)\end{bmatrix}$$

($f = \mathrm{Log}(R_0^{-1}R) \approx 0$ 근사는 prior 자체가 1차 모델인 것과 같은
급 — `SE3BoxplusManifold::MinusJacobian`과 동일 관례.) 즉 회전 열에 $J_r(\omega)$만
곱해 반환:

$$J_{\text{return}} = J_p\,\begin{bmatrix} I & 0\\ 0 & J_r(\omega)\end{bmatrix} \qquad [\text{ambient}]$$

#### (c) 왕복이 닫히는지 확인

Ceres가 이걸 받아 $P(x)$를 곱해 만드는 유효 tangent 야코비안:

$$J_t^{\text{eff}} = \big(J_p\, D(x)\big)\, P(x) = J_p \begin{bmatrix} I & 0\\ 0 & J_r(\omega)\,J_r^{-1}(\omega)\end{bmatrix} = J_p \;\checkmark$$

저장된 tangent 정보가 정확히 복원된다. 요약하면 수정된
`MarginalizationCost::Evaluate`는: pose 블록 $dx$를 raw 차이 대신 **boxminus**로,
반환 야코비안은 회전 열에 $J_r(\omega_{\text{current}})$를 곱해 **ambient로 변환**.

---

## 5. 기존 코드의 오류를 같은 수식으로

옛 코드는 (i) $\delta_k$ 자리에 raw 차이를 쓰고, (ii) $J_p$를 변환 없이 반환했다.

**Residual** — raw 차이 사용:

$$e_p^{\text{old}} = e_0 + J_p\,(x - x^0) = e_0 + J_p \begin{bmatrix} I & 0\\ 0 & J_r^{-1}(\omega^0)\end{bmatrix}\delta_{\text{true}} + O(\delta^2)$$

스프링이 $J_r^{-1}(\omega^0)$만큼 왜곡된 자로 잰 변위에 벌점을 매긴다.

**Jacobian** — $J_p$를 그대로 반환하여 Ceres가 $P$를 곱함:

$$J_t^{\text{eff,old}} = J_p\,P(x) = J_p \begin{bmatrix} I & 0\\ 0 & J_r^{-1}(\omega)\end{bmatrix}$$

**이중 왜곡의 자기모순** — 이 팩터의 gradient:

$$g^{\text{old}} = (J_t^{\text{eff,old}})^\top e_p^{\text{old}} = \begin{bmatrix} I & 0\\ 0 & J_r^{-\top}(\omega)\end{bmatrix} J_p^\top\Big(e_0 + J_p \begin{bmatrix} I & 0\\ 0 & J_r^{-1}(\omega^0)\end{bmatrix}\delta\Big)$$

정답 $g = J_p^\top(e_0 + J_p\,\delta)$ 대비, residual 쪽 왜곡은 $J_r^{-1}(\omega^0)$,
gradient 사영 쪽 왜곡은 $J_r^{-\top}(\omega)$ — **서로 다른 두 왜곡**이 겹친다.

- $\omega \to 0$이면 $J_r \to I$라 무해 (초기 구간에서 멀쩡했던 이유).
- $\|\omega\|$가 클수록 $J_r = I - \tfrac{1-\cos\theta}{\theta^2}[\omega]_\times + \tfrac{\theta-\sin\theta}{\theta^3}[\omega]_\times^2$의
  비항등 항이 커져 벌점의 크기·방향이 모두 틀어진다 ($\theta = 90°$에서 ~36%).
- 이 틀린 힘 중 뻣뻣한 방향 성분은 다른 팩터가 상쇄하지만, $A$의
  null-space(전역 병진·yaw) 성분은 저항 없이 상태를 밀어낸다 → **윈도우 강체 점프**
  (EuRoC V1_01의 빠른 회전 구간에서 관측된 현상).

---

## 6. Exact Jacobian과 FEJ 근사의 차이

4-(b)에서 반환 야코비안의 회전 블록에 붙인 $J_r(\omega)$는 tangent → ambient
변환으로서 **정확하다**. 다만 prior residual
$e = e_0 + J_p\,\mathrm{Log}(R_0^{-1}R)$의 exact tangent 야코비안은

$$J_p\,J_r^{-1}(f),\qquad f = \mathrm{Log}(R_0^{-1}R)$$

이고, 현재 구현은 $J_r^{-1}(f) \approx I$로 근사한다. 정리하면:

- tangent → ambient 변환(`× Jr`)은 **맞다.**
- 하지만 residual의 exact derivative는 아니며 **FEJ 스타일의 근사**다 ($f \approx 0$,
  즉 iterate가 선형화점 근처라는 가정). prior가 애초에 1차 모델이라 같은 급의
  근사이며, `SE3BoxplusManifold::MinusJacobian`의 관례와 일치한다.

---

## 7. 직관용 수치 예시

$\omega^0 = [0, 0, \tfrac{\pi}{2}]$ (z축 90°)에서 바디 x축으로
$\delta\theta = [0.1, 0, 0]$ rad 회전하면:

| 량 | 값 |
|---|---|
| $\omega_{\text{new}} = \mathrm{Log}(\mathrm{Exp}(\omega^0)\mathrm{Exp}(\delta\theta))$ | $[0.0785,\ 0.0785,\ -0.0014]$ |
| raw 차이 $\omega_{\text{new}} - \omega^0$ | $[0.0785,\ 0.0785,\ -0.0014]$ |
| boxminus $\mathrm{Log}(R_0^\top R)$ | $[0.1,\ 0,\ 0]$ ✓ |

x축으로만 걸었는데 raw 차이는 x·y가 섞이고 크기도 21% 작다.
$J_r(\omega^0)$의 상단 2×2 블록이 $\begin{bmatrix}0.64 & 0.64\\ -0.64 & 0.64\end{bmatrix}$로
항등과 크게 다른 것이 그 이유이며, $J_r(\omega^0) \cdot (\text{raw 차이}) = [0.1, 0, -0.001]$로
진짜 걸음이 복원된다.

회전각이 커질수록 왜곡이 얼마나 심해지는지, $\omega^0 = [0, 0, 3.0]$ (z축 171.9°)에서
같은 $\delta\theta = [0.1, 0, 0]$을 밟으면:

| 량 | 값 |
|---|---|
| raw 차이 $\omega_{\text{new}} - \omega^0$ | $[0.0106,\ 0.1499,\ -0.0036]$, 크기 $0.150$ |
| boxminus $\mathrm{Log}(R_0^\top R)$ | $[0.1,\ 0,\ 0]$ ✓ |

이번엔 raw 차이가 진짜 걸음보다 **50% 크고 방향은 거의 y축**이다.
$J_r(\omega^0)$의 상단 2×2가 $\begin{bmatrix}0.05 & 0.66\\ -0.66 & 0.05\end{bmatrix}$ —
대각이 거의 죽고 비대각이 지배한다. 90°에서 21%였던 오차가 180°에 다가갈수록
발산 방향으로 커진다 ($\theta \to 2\pi$에서 $J_r$ 특이). raw 차이를 쓰는 prior는
회전이 클수록 엉뚱한 방향으로 점점 세게 당기는 스프링이 된다.

---

## 8. 한 장 요약

| 단계 | 객체 | 공간 | 근거 |
|---|---|---|---|
| 코스트 함수 반환 | $J_a = \partial e/\partial x$ | **ambient** | 각 cost의 `Evaluate` |
| Ceres 내부 | $\bar J_t = J_a P$ | **tangent** | `problem.Evaluate` |
| Hessian/RHS | $H=\bar J_t^\top\bar J_t,\ b=\bar J_t^\top\bar e$ | **tangent** | `create_hessian_from_crs_matrix` |
| Schur | $A,\ b^\ast$ | **tangent** | `marginalize()` |
| 저장 prior | $J_p=\Lambda^{1/2}V^\top,\ e_0$ | **tangent** | `J_`, `r_` |
| 저장 선형화점 | $x^0_k=(t,\omega)$ | **ambient(값)** | `x0` |
| 재평가 $\delta_k$ | boxminus $x_k\boxminus x^0_k$ | **tangent** | `MarginalizationCost::Evaluate` |
| 재평가 반환 J | $J_p\,\mathrm{diag}(I,J_r(\omega))$ | **ambient** | 회전 열 $\times J_r$ |
| Ceres 재적용 후 | $J_p\,\mathrm{diag}(I,J_r)\,\mathrm{diag}(I,J_r^{-1})=J_p$ | **tangent 복원** | 왕복 검증 |

핵심 한 문장: **"저장은 tangent, Ceres 인터페이스만 ambient"** — 그리고 그 둘을
잇는 접착제는 항상 회전 블록의 $J_r(\omega)$ 한 배다. 단, 4-(b)의
$J_r^{-1}(f)\approx I$는 exact derivative가 아니라 FEJ 스타일 근사라는 점(§6)이
유일하게 "정확하지 않은" 지점이다.

---

## 참고

- Ceres `problem.h` — `Problem::Evaluate` Note 2 (tangent-size jacobian)
- Forster et al., *On-Manifold Preintegration* (right Jacobian 정의)
- 수정 검증: EuRoC V1_01에서 강체 점프 3회(최대 12.9 cm) → 0회, 평균 prior cost 절반 감소
  (2026-07-16, `[wopt]` 계측 로그)
