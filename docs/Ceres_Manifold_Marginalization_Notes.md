# Ceres Manifold / Jacobian / Marginalization 정리

## 1. Parameterization

저장되는 pose는

-   translation: `t`
-   rotation: `ω = log(R)`

이며 parameter는

``` text
x = [t, ω]
```

하지만 업데이트는 단순 덧셈이 아니라

``` math
R_{new}=R\exp(\delta\theta)
```

를 사용한다.

------------------------------------------------------------------------

## 2. Ambient parameter와 Tangent parameter

Ambient parameter

``` math
\omega=\log(R)
```

Optimizer가 실제 최적화하는 것은 tangent increment

``` math
\delta\theta
```

이다.

두 좌표계는 1차 근사에서

``` math
\exp(\omega+\Delta\omega)
\approx
\exp(\omega)\exp(J_r(\omega)\Delta\omega)
```

즉

``` math
\delta\theta
=
J_r(\omega)\Delta\omega
```

관계가 있다.

반대로

``` math
\Delta\omega
=
J_r^{-1}(\omega)\delta\theta
```

이므로

``` math
\exp(\omega)\exp(\delta\theta)
\approx
\exp(\omega+J_r^{-1}(\omega)\delta\theta)
```

위 두 식은 서로 같은 내용을 서로 다른 방향에서 표현한 것이다.

------------------------------------------------------------------------

## 3. Cost Jacobian vs Manifold Jacobian

CostFunction이 반환해야 하는 것은

``` math
\frac{\partial r}{\partial x}
```

(ambient Jacobian)

이다.

Manifold는

``` math
\frac{\partial x}{\partial\delta}
=
J_{plus}
```

를 제공한다.

Ceres 내부에서는

``` math
J_{solver}
=
J_{cost}
J_{plus}
```

를 사용한다.

우리 manifold에서는

``` math
J_{plus}
=
J_r^{-1}(\omega)
```

이다.

------------------------------------------------------------------------

## 4. Reprojection Jacobian

Tangent perturbation 기준

``` math
R' = R\exp(\delta\theta)
```

이면

``` math
\frac{\partial r}{\partial\delta\theta}
=
-
J_\pi
R
[P]_\times
```

이다.

Ambient parameter로 미분하면

``` math
\frac{\partial r}{\partial\omega}
=
-
J_\pi
R
[P]_\times
J_r(\omega)
```

가 된다.

즉

``` text
Ambient Jacobian = Tangent Jacobian × Jr
```

------------------------------------------------------------------------

## 5. Marginalization

상태를

``` math
x=
\begin{bmatrix}
x_m\\
x_r
\end{bmatrix}
```

로 나눈다.

선형화

``` math
r(x_0\boxplus\delta)
\approx
r_0
+
J\delta
```

에서

``` math
H=J^TJ,\qquad g=J^Tr
```

를 만든다.

Schur complement

``` math
H_p
=
H_{rr}
-
H_{rm}
H_{mm}^{-1}
H_{mr}
```

``` math
g_p
=
g_r
-
H_{rm}
H_{mm}^{-1}
g_m
```

Square-root prior

``` math
J_p^TJ_p=H_p
```

``` math
J_p^Tr_p=g_p
```

를 저장한다.

------------------------------------------------------------------------

## 6. Prior residual

저장되는 prior는

``` math
r_{prior}
=
r_p
+
J_p
(x\boxminus x_0)
```

Pose에서는

``` math
x\boxminus x_0
=
\begin{bmatrix}
t-t_0\\
\log(R_0^{-1}R)
\end{bmatrix}
```

이다.

------------------------------------------------------------------------

## 7. 현재 코드가 하는 일

`Problem::Evaluate()`에서 저장한 `J_`는 이미 tangent Jacobian이다.

따라서 그대로 반환하면

``` text
Jsolver = J_ · Jr^{-1}
```

가 되어 틀어진다.

그래서 코드에서는

``` text
Jambient = J_ · Jr
```

를 반환한다.

Ceres가

``` text
Jr^{-1}
```

를 곱하면

``` text
Jsolver
=
J_ · Jr · Jr^{-1}
=
J_
```

가 되어 원래 tangent Jacobian을 복원한다.

이 부분은 맞는 구현이다.

------------------------------------------------------------------------

## 8. Exact Jacobian과의 차이

현재 prior residual

``` math
r
=
r_p
+
J_p
\log(R_0^{-1}R)
```

의 정확한 tangent Jacobian은

``` math
J_p
J_r^{-1}
(\phi),
\qquad
\phi
=
\log(R_0^{-1}R)
```

이다.

현재 구현은

``` math
J_r^{-1}(\phi)
\approx
I
```

로 근사한다.

즉

-   tangent → ambient 변환(`× Jr`)은 **맞다.**
-   하지만 residual의 exact derivative는 아니며 FEJ 스타일의 근사이다.

------------------------------------------------------------------------

## 핵심 정리

-   CostFunction → ambient Jacobian
-   Manifold → PlusJacobian
-   Ceres → `Jsolver = Jcost × PlusJacobian`
-   Reprojection ambient Jacobian = tangent Jacobian × `Jr`
-   Marginalization에서 저장되는 `J_`는 tangent Jacobian
-   따라서 CostFunction에서는 `J_ × Jr`를 반환하는 것이 맞다.
-   현재 prior는 `J_r^{-1}(Log(R0^{-1}R)) ≈ I`를 사용하는 근사 구현이다.
