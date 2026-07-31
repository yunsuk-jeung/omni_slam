# Basalt: NFR Mapper 동작 원리 (VIO prior 추출 → 분해 → 맵 최적화)

> 이 문서는 basalt의 `NfrMapper`(**N**on-linear **F**actor **R**ecovery Mapper)가 어떻게 동작하는지
> 수식 중심으로 최대한 자세히 정리한 것이다. 특히 사용자가 궁금해한
> **"VIO 쪽에서 prior(사전정보)를 뽑아서 어떻게 전달하는가"**, **"그 수식이 무엇인가"**,
> **"그걸 어떻게 분해(decompose)해서 NFR mapping에 쓰는가"** 를 코드 라인과 함께 붙였다.
>
> 관련 파일:
> - `include/basalt/vi_estimator/nfr_mapper.h` — NfrMapper 선언
> - `src/vi_estimator/nfr_mapper.cpp` — 핵심 구현
> - `include/basalt/utils/nfr.h` — 4개의 잔차(residual) 함수와 야코비안
> - `include/basalt/utils/imu_types.h:333-365` — `MargData`, `RelPoseFactor`, `RollPitchFactor` 정의
> - `src/vi_estimator/sqrt_keypoint_vio.cpp:752-782` — **VIO가 prior를 뽑아 전달하는 지점**
> - `src/vi_estimator/marg_helper.cpp` — Schur complement marginalization
> - `src/mapper.cpp` — NfrMapper를 실제로 구동하는 오프라인 앱

---

## 0. 큰 그림 — NFR Mapper는 무엇을 하나

VIO(Visual-Inertial Odometry)는 실시간으로 돌기 위해 **슬라이딩 윈도우**만 유지한다. 윈도우 밖으로
나가는 오래된 키프레임은 **marginalize(소거)** 되는데, 그냥 버리는 게 아니라 그 키프레임이 담고 있던
정보를 **prior(사전정보) H, b 행렬**로 압축해서 남긴다. 이게 "VIO에서 뽑아내는 prior"다.

`NfrMapper`는 이렇게 저장된 marginalization prior들을 **여러 개 모아서** 전역 지도(global map)를
다시 최적화한다. 목적은:

1. VIO가 실시간 제약 때문에 놓친 **루프 클로저(loop closure)** 를 다시 찾고 (BoW로 과거 프레임 재매칭),
2. 각 marginalization prior에서 **비선형 factor(RelPose / RollPitch)** 를 복원(recover)하여,
3. 새로 찾은 시각 관측(feature tracks)과 이 factor들을 **하나의 큰 최소제곱 문제**로 함께 풀어
   전역적으로 일관된(globally consistent) 궤적과 지도를 만든다.

> **NFR = Non-linear Factor Recovery**: 원논문 아이디어는 "marginalization으로 얻은 선형화된
> 정보 행렬(H, b)을, 다시 최적화 가능한 **비선형 factor들의 집합**으로 되돌린다(recover)"는 것이다.
> 선형 prior는 한 번 선형화된 점에 묶여 있어 pose가 크게 움직이면 부정확해진다. 이를 상대 pose /
> roll-pitch 같은 **재선형화 가능한 factor**로 바꾸면, 맵 최적화에서 pose가 크게 움직여도 factor를
> 매 iteration 다시 선형화할 수 있다.

전체 파이프라인:

```
[VIO 실행 중]
  슬라이딩 윈도우 최적화 → 오래된 KF marginalize
     → marginalization prior (abs_H, abs_b) 생성
     → MargData 로 포장해서 out_marg_queue 로 push  ┐
                                                     │ (디스크 저장 or 큐 전달)
[NfrMapper (오프라인 mapper 앱)]                      │
  MargData 로드 ◄──────────────────────────────────┘
     addMargData():
        processMargData()         : velocity/bias 소거 → pose-only prior 로 축소
        extractNonlinearFactors() : prior 를 H^-1=Σ 로 뒤집어 RelPose/RollPitch factor 복원
        frame_poses 에 pose 등록
  detect_keypoints() → match_stereo()/match_all() (BoW 루프클로저) → build_tracks() → setup_opt()
  optimize(): 시각 잔차 + RelPose factor + RollPitch factor 를 LM 으로 동시 최적화
```

---

## 1. Marginalization prior 란 무엇인가 (수식 기초)

최소제곱 문제는 상태 $x$ 에 대해

$$
E(x) = \tfrac{1}{2}\, r(x)^\top W\, r(x)
$$

를 최소화한다. $r$은 잔차(residual), $W$는 정보(가중) 행렬. 현재 추정치 $\bar{x}$ 근처에서
$\Delta x = x - \bar{x}$ 로 선형화하면 Gauss-Newton normal equation:

$$
H\,\Delta x = -b, \qquad H = J^\top W J, \qquad b = J^\top W\, r
$$

여기서 $J = \partial r / \partial x$. $H$는 **정보 행렬(information matrix)**, 그 역행렬
$\Sigma = H^{-1}$ 은 **공분산(covariance)** 이다. 이 관계가 이 문서 전체의 핵심이다.

### Marginalization = Schur complement

상태를 "남길 것 $x_k$ (keep)"과 "소거할 것 $x_m$ (marginalize)"로 나누면:

$$
\begin{bmatrix} H_{kk} & H_{km} \\ H_{mk} & H_{mm} \end{bmatrix}
\begin{bmatrix} \Delta x_k \\ \Delta x_m \end{bmatrix}
= -\begin{bmatrix} b_k \\ b_m \end{bmatrix}
$$

$x_m$ 을 소거하면 $x_k$ 에 대한 **새 prior**:

$$
\boxed{\;H^{*} = H_{kk} - H_{km} H_{mm}^{-1} H_{mk}, \qquad
b^{*} = b_k - H_{km} H_{mm}^{-1} b_m\;}
$$

이것이 basalt의 `marginalizeHelperSqToSq`가 하는 일이다
(`src/vi_estimator/marg_helper.cpp:102-118`):

```cpp
// H_mm 의 pseudo-inverse (Complete Orthogonal Decomposition)
auto H_mm_decomposition = abs_H.bottomRightCorner(marg_size, marg_size)
                              .completeOrthogonalDecomposition();
MatX H_mm_inv = H_mm_decomposition.pseudoInverse();

abs_H.topRightCorner(keep_size, marg_size) *= H_mm_inv;      // H_km * H_mm^-1

marg_H = abs_H.topLeftCorner(keep_size, keep_size);          // H_kk
marg_b = abs_b.head(keep_size);                              // b_k

marg_H -= abs_H.topRightCorner(keep_size, marg_size) *       // - H_km H_mm^-1 H_mk
          abs_H.bottomLeftCorner(marg_size, keep_size);
marg_b -= abs_H.topRightCorner(keep_size, marg_size) *       // - H_km H_mm^-1 b_m
          abs_b.tail(marg_size);
```

> permutation 행렬 $P$로 keep/marg 인덱스를 위/아래 블록으로 재배열한 뒤
> ($H \leftarrow P^\top H P$, $b \leftarrow P^\top b$) Schur complement를 적용한다.
> `H_mm`이 rank-deficient일 수 있으므로 일반 역행렬 대신 **pseudo-inverse (COD)** 를 쓴다.

---

## 2. VIO에서 prior를 어떻게 "뽑아서" 전달하나

### 2-1. VIO 내부: sqrt 형태의 정보 행렬

Basalt VIO는 수치 안정성을 위해 $H, b$ 대신 **square-root 형태** $J, r$ 를 다룬다
(`√-BA`, sqrt marginalization). 매 프레임 marginalize 직전에 문제를 선형화하고 QR로 landmark를 소거해
pose/속도/bias 부분만 남긴 $Q_2 J_p$, $Q_2 r$ 를 얻는다
(`src/vi_estimator/sqrt_keypoint_vio.cpp:729-740`):

```cpp
auto lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
    this, aom, lqr_options, &marg_data, &ild, &kfs_to_marg, ...);
lqr->linearizeProblem();
lqr->performQR();                             // landmark 소거 (nullspace projection)

if (is_lin_sqrt && marg_data.is_sqrt) {
  lqr->get_dense_Q2Jp_Q2r(Q2Jp_or_H, Q2r_or_b);   // 제곱근 형태 J, r
} else {
  lqr->get_dense_H_b(Q2Jp_or_H, Q2r_or_b);        // 직접 H, b
}
```

### 2-2. prior를 `MargData`로 포장 (여기가 핵심 전달 지점)

`src/vi_estimator/sqrt_keypoint_vio.cpp:752-782`:

```cpp
// Save marginalization prior
if (out_marg_queue && !kfs_to_marg.empty()) {
  MargData::Ptr m(new MargData);
  m->aom = aom;                                        // 상태 순서(AbsOrderMap)

  if (is_lin_sqrt && marg_data.is_sqrt) {
    m->abs_H = (Q2Jp_or_H.transpose() * Q2Jp_or_H).cast<double>();   // H = JᵀJ
    m->abs_b = (Q2Jp_or_H.transpose() * Q2r_or_b).cast<double>();    // b = Jᵀr
  } else {
    m->abs_H = Q2Jp_or_H.cast<double>();               // 이미 H
    m->abs_b = Q2r_or_b.cast<double>();                // 이미 b
  }

  assign_cast_map_values(m->frame_poses,  frame_poses);   // 현재 pose 추정치
  assign_cast_map_values(m->frame_states, frame_states);  // pose+vel+bias 상태
  m->kfs_all     = kf_ids_all;      // 이 prior에 관여하는 모든 KF id
  m->kfs_to_marg = kfs_to_marg;     // 이번에 실제 marginalize 될 KF id
  m->use_imu     = true;

  for (int64_t t : m->kfs_all)      // 재검출/루프클로저용 원본 이미지
    m->opt_flow_res.emplace_back(prev_opt_flow_res.at(t));

  out_marg_queue->push(m);          // ← 여기서 밖으로 전달
}
```

즉 **뽑아서 전달하는 것 = `MargData`** 이고 그 안의 핵심은:

| 필드 | 의미 | 수식 |
|---|---|---|
| `abs_H` | 정보 행렬 | $H = J^\top J$ |
| `abs_b` | 정보 벡터 | $b = J^\top r$ |
| `aom` | 상태 순서/크기 매핑 | 각 KF의 블록 시작 index, 크기(POSE 6 또는 POSE_VEL_BIAS 15) |
| `frame_poses` / `frame_states` | 선형화 기준점 $\bar{x}$ | pose(6), pose+vel+bias(15) |
| `kfs_all` | prior에 얽힌 모든 KF | — |
| `kfs_to_marg` | 이번에 소거될 KF | — |
| `opt_flow_res` | 원본 이미지 | 루프클로저 재검출용 |

`MargData`의 정의 (`include/basalt/utils/imu_types.h:333-346`):

```cpp
struct MargData {
  AbsOrderMap aom;
  Eigen::MatrixXd abs_H;
  Eigen::VectorXd abs_b;
  Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<double>> frame_states;
  Eigen::aligned_map<int64_t, PoseStateWithLin<double>> frame_poses;
  std::set<int64_t> kfs_all;
  std::set<int64_t> kfs_to_marg;
  bool use_imu;
  std::vector<OpticalFlowResult::Ptr> opt_flow_res;
};
```

### 2-3. 전달 경로

`out_marg_queue`에 push된 `MargData`는 `MargDataSaver`(`src/io/marg_data_io.cpp`)가 별도 스레드에서
`cereal`로 **디스크에 직렬화**한다 (`imu_types.h:377-387`에 serialize 정의). 나중에 오프라인 mapper가
`MargDataLoader`로 다시 읽어들여 `NfrMapper::addMargData()`에 넣는다 (`src/mapper.cpp:183-185`, `575-586`).

> 요약: **VIO(온라인) → MargData(H,b,pose,이미지) → 디스크 → NfrMapper(오프라인)**.
> VIO와 NfrMapper는 큐/파일로 느슨하게 연결돼 있어 서로 다른 프로세스로 돌 수 있다.

---

## 3. NfrMapper가 prior를 받아 처리 — `addMargData()`

`src/vi_estimator/nfr_mapper.cpp:60-80`:

```cpp
void NfrMapper::addMargData(MargData::Ptr& data) {
  processMargData(*data);                          // (A) vel/bias 소거 → pose-only
  bool valid = extractNonlinearFactors(*data);     // (B) prior → 비선형 factor 복원

  if (valid) {
    for (const auto& kv : data->frame_poses)       // (C) pose 등록
      frame_poses[kv.first] = PoseStateWithLin<double>(kv.second.getT_ns(),
                                                       kv.second.getPose());
    for (const auto& kv : data->frame_states)      // keyframe 상태도 pose 로 등록
      if (data->kfs_all.count(kv.first) > 0) {
        auto state = kv.second;
        frame_poses[kv.first] =
            PoseStateWithLin<double>(state.getState().t_ns, state.getState().T_w_i);
      }
  }
}
```

세 단계 (A)(B)(C)를 차례로 본다.

### 3-A. `processMargData()` — 속도/bias를 추가로 소거

VIO prior의 상태는 KF마다 pose(6) 또는 pose+velocity+bias(15, `POSE_VEL_BIAS_SIZE`)로 섞여 있다.
NfrMapper는 **지도 최적화에 pose(6)만 필요**하므로, 15-크기 상태에서 velocity/bias(9개)를 추가로
marginalize 해서 순수 pose prior로 축소한다 (`nfr_mapper.cpp:82-141`):

```cpp
for (const auto& kv : m.aom.abs_order_map) {
  if (kv.second.second == POSE_SIZE) {              // 이미 pose(6): 유지
    for (i=0..5) idx_to_keep.emplace(start + i);
    aom_new ... += POSE_SIZE;
  } else if (kv.second.second == POSE_VEL_BIAS_SIZE) {  // 15 상태
    if (m.kfs_all.count(kv.first) > 0) {            // 살릴 KF: pose(0..5) keep,
      for (i=0..5)  idx_to_keep.emplace(start + i); //          vel/bias(6..14) marg
      for (i=6..14) idx_to_marg.emplace(start + i);
      m.frame_poses[kv.first] = PoseStateWithLin<double>(m.frame_states.at(kv.first));
      m.frame_states.erase(kv.first);
    } else {                                        // 안 살릴 KF: 전체(0..14) marg
      for (i=0..14) idx_to_marg.emplace(start + i);
    }
  }
}

if (!idx_to_marg.empty()) {
  MargHelper<Scalar>::marginalizeHelperSqToSq(       // §1 의 Schur complement
      m.abs_H, m.abs_b, idx_to_keep, idx_to_marg, marg_H_new, marg_b_new);
  m.abs_H = marg_H_new;   m.abs_b = marg_b_new;   m.aom = aom_new;
}
```

또한 원본 이미지를 `img_data`에 보관한다 (`nfr_mapper.cpp:144-148`) — 뒤의 특징점 재검출용.

이 단계가 끝나면 `m.abs_H`, `m.abs_b`는 **오직 pose들에 대한** 정보 행렬/벡터가 된다.

### 3-B. `extractNonlinearFactors()` — prior를 비선형 factor로 복원 ★핵심★

여기가 "NFR"의 심장이다. **선형 pose prior $(H, b)$ 를 재선형화 가능한 비선형 factor들의
집합으로 되돌린다.** 아이디어: prior를 뒤집어 공분산 $\Sigma = H^{-1}$ 을 얻은 뒤, 우리가 원하는
factor(절대위치 / yaw / roll-pitch / 상대pose)로 잔차를 정의하고, 오차 전파(covariance propagation)로
그 factor의 공분산을 구해 정보 행렬 $\Omega = \Sigma_{\text{factor}}^{-1}$ 로 삼는다.

**Step 1 — prior 역변환으로 공분산 얻기** (`nfr_mapper.cpp:151-158`):

```cpp
Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr(m.abs_H);
if (qr.rank() != m.abs_H.cols()) return false;               // full-rank 아니면 폐기
Eigen::MatrixXd cov_old = qr.solve(Eigen::MatrixXd::Identity(asize, asize));
```

$$
\Sigma = H^{-1} \quad(\text{코드의 } \texttt{cov\_old})
$$

full-rank가 아니면(관측 부족) 이 MargData는 통째로 버린다(`return false`).

**Step 2 — 기준 키프레임(anchor) 선택** (`nfr_mapper.cpp:160-169`):

```cpp
int64_t kf_id = *m.kfs_to_marg.cbegin();                     // 소거될 KF 하나를 anchor 로
int kf_start_idx = m.aom.abs_order_map.at(kf_id).first;
Sophus::SE3d T_w_i_kf = state_kf.getPose();                  // 그 KF 의 절대 pose
Eigen::Vector3d pos = T_w_i_kf.translation();
Eigen::Vector3d yaw_dir_body = T_w_i_kf.so3().inverse() * Eigen::Vector3d::UnitX();
```

**Step 3 — RollPitch factor 복원** (`nfr_mapper.cpp:171-203`).

이 단계가 NFR에서 가장 핵심이면서 헷갈리는 곳이므로 아주 자세히 뜯어본다.

#### 3-B-Step3-0. 큰 그림 — 왜 이런 우회를 하나

- 가진 것: pose들에 대한 **정보 행렬** $H$ (`abs_H`).
- 원하는 것: anchor KF의 **roll-pitch에 대한 정보 행렬** $\Omega_{rp}$ (factor 가중치).
- 걸림돌: $H$는 pose를 **원래 6-DoF 좌표 `[translation(3), rotation(3)]`** 로 표현하는데, "roll-pitch"는
  rotation을 다시 **`[yaw(중력축 회전), roll-pitch(기울기)]`** 로 쪼갠 좌표다. → **좌표 변환 + 부분추출**이 필요.

핵심 전략은 다음 한 문장이다:

> **정보(H) 공간에서 부분추출(주변화)은 어렵다(Schur complement 필요). 하지만 공분산(Σ) 공간에서는
> 부분추출이 그냥 "블록 잘라내기"다.**

그래서 $H \to \Sigma \to$ (좌표변환) $\to$ (2×2 블록 추출) $\to$ 다시 역행렬 순서로 우회한다.

#### 3-B-Step3-1. `cov_old` = $\Sigma = H^{-1}$ 은 무엇의 공분산인가

$\Sigma$ 는 이 marg prior에 든 **모든 pose의 오차 $\delta x$ 에 대한 결합 공분산**이다. $\delta x$ 는 각 pose마다
6-DoF 접선벡터 `[δt_x,δt_y,δt_z, δθ_x,δθ_y,δθ_z]` 를 쌓은 것 (크기 `asize×asize`).

- gauge freedom(전역 위치 3 + yaw 1 = 4-DoF)이 남으면 $H$ 가 rank-deficient → $H^{-1}$ 없음. 그러나 VIO가
  **초기 프레임에 강한 pose prior**(`sqrt_keypoint_vio.cpp:86-108`, `vio_init_pose_weight`)를 걸어 gauge를
  고정하고 그 정보가 prior에 누적되므로 보통 full-rank가 되어 역행렬이 존재한다. 모자라면 폐기(`return false`).
- 이 gauge 고정 때문에 $\Sigma$ 에서 **position·yaw 분산은 크고**(약하게 묶임) **roll-pitch 분산은 작다**
  (IMU 중력으로 강하게 관측). 이것이 "roll-pitch만 절대 factor로 만드는" 정량적 근거다.

#### 3-B-Step3-2. `J` 는 "6-DoF 오차 → [위치, yaw, roll-pitch]" 좌표 변환

```cpp
absPositionError(T_w_i_kf, pos,          &d_pos_d_T_w_i);   // 3×6  위치
yawError       (T_w_i_kf, yaw_dir_body,  &d_yaw_d_T_w_i);   // 1×6  yaw
rollPitchError (T_w_i_kf, T_w_i_kf.so3(),&d_rp_d_T_w_i);    // 2×6  roll/pitch

Eigen::MatrixXd J; J.setZero(POSE_SIZE, asize);            // 6×asize, 전부 0으로 시작
J.block<3,6>(0, kf_start_idx) = d_pos_d_T_w_i;            // 행 0-2: position
J.block<1,6>(3, kf_start_idx) = d_yaw_d_T_w_i;            // 행 3:   yaw
J.block<2,6>(4, kf_start_idx) = d_rp_d_T_w_i;            // 행 4-5: roll/pitch

Sophus::Matrix6d cov_new = J * cov_old * J.transpose();    // 오차 전파!
```

포인트 1: **`J`는 `kf_start_idx` 열(anchor의 6칸)만 nonzero**, 나머지 pose 열은 0이다. anchor의
위치/yaw/roll-pitch는 오직 anchor 자신의 pose에만 의존하니까.

포인트 2: 선형화점(현재 추정치)에서 각 야코비안 값을 실제로 넣어보면 정체가 드러난다.

- **`absPositionError` → 행 0-2 (위치)**: $r_{\text{pos}} = t_{wi}-t^{\text{meas}}$, $\partial r/\partial\delta x = [\,I_3\ \ 0_3\,]$.
  → 행 0-2 = `[δt_x, δt_y, δt_z]` 그대로.
- **`yawError` → 행 3 (yaw)**: `yaw_dir_body = R_wi⁻¹·e_x` 라 선형화점에서 `tmp = R_wi·yaw_dir_body = e_x = (1,0,0)`.
  코드상 `d[3] = -tmp[2] = 0`, `d[5] = tmp[0] = 1` → 행 3 = `[0 0 0 | 0 0 1]` = **δθ_z (중력축 Z 둘레 = yaw)** 만.
- **`rollPitchError` → 행 4-5 (roll-pitch)**: 측정치로 현재 회전을 넣어(`R_w_i_meas=T_w_i_kf.so3()`) 선형화점에서
  `R = R_meas·R_wi⁻¹ = I`. 코드상 `(0,4)=R(0,0)=1`, `(1,3)=-R(1,1)=-1`, 나머지 0 → 행 4 = `δθ_y`, 행 5 = `-δθ_x`
  = **X,Y축 둘레 회전(중력에 대한 기울기)** 만.

정리하면 선형화점에서 anchor 6×6 블록 $J_a$ 는 사실상

$$
[\delta t_x,\delta t_y,\delta t_z,\ \delta\theta_x,\delta\theta_y,\delta\theta_z]
\;\longrightarrow\;
[\underbrace{\delta t_x,\delta t_y,\delta t_z}_{\text{위치}},\ \underbrace{\delta\theta_z}_{\text{yaw}},\ \underbrace{\delta\theta_y,-\delta\theta_x}_{\text{roll-pitch}}]
$$

로, **회전축을 yaw / roll-pitch로 갈라 재배열하는 변환**이다. rotation 블록을 직접 떼오지 않고 residual 함수로
$J$를 만드는 이유는 → **optimize에서 RollPitch factor를 선형화할 때 쓰는 `rollPitchError`의 야코비안과
정확히 같은 좌표계**로 공분산을 뽑기 위해서다. factor의 정보 행렬은 그 factor 자신의 잔차 좌표에서 표현돼야 한다.

> **표기 주의 (같은 기호가 재등장하는 이유):** 위 화살표 오른쪽은 새 좌표에 새 기호를 붙인 게 아니라,
> 출력 벡터 $y=J_a\,\delta x$ 의 각 칸을 **입력 성분 $\delta x$ 로 그대로 표현**한 것이다("새 성분 = 옛 성분 중
> 무엇과 같은가"). 그래서:
> - **위치(0-2)는 진짜 항등** — `absPositionError` 잔차가 $r=t_{wi}-t^{\text{meas}}$ 라 야코비안이 $[I_3\ 0]$,
>   위치 오차가 곧 $\delta t$ 자체다. 기호가 같은 게 맞다.
> - **회전은 항등이 아님** — 입력 $(\delta\theta_x,\delta\theta_y,\delta\theta_z)$ 가 출력에서
>   $(\delta\theta_z,\ \delta\theta_y,\ -\delta\theta_x)$ 로 **순서가 바뀌고 $x$에 부호가 붙는다.**
>
> 즉 선형화점에서 $J_a$ 는 부호 있는 순열행렬이다:
> $$
> J_a \approx \begin{bmatrix} I_3 & 0 \\ 0 & P \end{bmatrix},\qquad
> P = \begin{bmatrix} 0 & 0 & 1 \\ 0 & 1 & 0 \\ -1 & 0 & 0 \end{bmatrix}
> $$
> ($P$ 의 행 = [yaw, roll, pitch], 열 = $[\theta_x,\theta_y,\theta_z]$). 위치 블록만 항등이라 기호가 그대로일 뿐,
> 회전 블록은 재배열/부호반전되어 실제로는 완전히 같지 않다. (이 관계는 선형화점에서만 정확히 성립.)

#### 3-B-Step3-3. 오차 전파 $\Sigma_{\text{factor}} = J\,\Sigma\,J^\top$

선형 가우시안 변환: $\delta x \sim \mathcal{N}(0,\Sigma)$ 를 $y = J\,\delta x$ 로 보내면 $y \sim \mathcal{N}(0,\ J\Sigma J^\top)$.

$$
\Sigma_{\text{factor}} = J\, \Sigma\, J^\top \qquad(\texttt{cov\_new},\ 6\times 6,\ \text{좌표계 }[\text{위치},\text{yaw},\text{roll-pitch}])
$$

중요한 관찰: **$J$가 anchor 열만 nonzero라서**

$$
J\Sigma J^\top = J_a\,\Sigma_{aa}\,J_a^\top
$$

즉 전체 $\Sigma$ 중 **anchor의 6×6 대각 블록 $\Sigma_{aa}$ 하나만** 실제로 쓰인다. (다른 pose와의 교차공분산은
이 factor엔 안 들어가고, RelPose factor가 담당.)

#### 3-B-Step3-4. 2×2 블록 추출 → 역행렬 = $\Omega_{rp}$

```cpp
RollPitchFactor rpf;
rpf.t_ns = kf_id;
rpf.R_w_i_meas = T_w_i_kf.so3();                          // 측정치 = 현재 회전
if (!config.mapper_no_factor_weights)
  rpf.cov_inv = cov_new.block<2,2>(4,4).inverse();        // Ω = Σ_rp^-1
else
  rpf.cov_inv.setIdentity();
if (m.use_imu) roll_pitch_factors.emplace_back(rpf);       // IMU 있을 때만
```

$$
\Omega_{rp} = \big(\Sigma_{\text{factor}}[4{:}5,\,4{:}5]\big)^{-1}
$$

가우시안에서 **공분산의 부분블록 = 그 변수들의 주변(marginal) 공분산**이다. 즉 `cov_new[4:5,4:5]` 는
"위치와 yaw의 불확실성을 전부 적분해 없앤 뒤 남는 roll-pitch만의 공분산". 역행렬을 취하면 factor의
정보 행렬 $\Omega_{rp}$. (정보 $H$ 에서 직접 뽑으려면 Schur complement가 필요하지만, 공분산으로 뒤집으면
**2×2 블록 잘라내기만으로 주변화가 끝난다** — 이것이 $H\to\Sigma\to$블록$\to H$ 왕복의 이유.)

> **디테일:** $\Omega_{rp}$ 는 `cov_new`의 행/열 4-5만 쓰고, $\texttt{cov\_new}[4{:}5,4{:}5] = J_{rp}\,\Sigma\,J_{rp}^\top$
> 이므로 **위치(행0-2)·yaw(행3)는 계산해놓고 결과엔 안 쓰인다.** roll-pitch factor만 뽑는 이 경로에서
> `absPositionError`/`yawError` 호출은 사실상 잉여(위치/yaw factor도 뽑을 수 있게 일반적으로 짜둔 흔적).

> **왜 roll-pitch만?** IMU 가속도계가 중력 방향을 관측하므로 roll/pitch는 **절대적으로 관측 가능**
> (observable)하고 $\Sigma$ 에서 분산이 작다 → 강한 절대 factor로 의미 있음. 반면 position·yaw는 gauge
> freedom이라 분산이 크고 절대 factor로 만들면 잘못된 절대 기준에 끌려간다. 그래서 그 정보는 버리지 않고
> **RelPose factor**(상대 제약)로 넘긴다. IMU가 없으면(`!use_imu`) 중력 관측이 없어 roll-pitch도 절대
> 관측 불가 → 아예 안 만든다.

#### 3-B-Step3-5. RollPitch factor 는 한 KF에 대한 단항(unary) 절대 factor

이 factor는 다른 KF를 참조하지 않고 **오직 그 키프레임 하나의 절대 orientation**에만 걸린다. optimize의
기여 코드(`nfr_mapper.h:109-125`)를 보면 명확하다:

```cpp
const Sophus::SE3d& pose = frame_poses->at(rpf.t_ns).getPose();   // 딱 한 KF
int idx = this->aom.abs_order_map.at(rpf.t_ns).first;             // 그 KF 하나의 블록
Sophus::Vector2d res = rollPitchError(pose, rpf.R_w_i_meas, &J);  // 2차원 잔차
accum.addH(idx, idx, J.transpose()*rpf.cov_inv*J);               // idx,idx 대각 블록에만
accum.addB(idx, J.transpose()*rpf.cov_inv*res);
```

`addH(idx, idx, ...)` — **같은 인덱스에만** 누적된다. 두 KF를 잇는 RelPose factor가 교차항
`addH(i,j)` 를 만드는 것과 달리, 이건 **혼자 서 있는(unary) factor**다.

단, "절대 pose 전체"가 아니라 정확히는 6-DoF 중 아래 2-DoF만 제약한다:

| pose 성분 | 이 factor가 제약? |
|---|---|
| 위치 (x, y, z) | ❌ 전혀 안 건드림 |
| yaw (중력축 Z 둘레 회전) | ❌ |
| **roll, pitch (중력에 대한 기울기)** | ✅ 이 2-DoF만 |

즉 정확한 표현은 **"한 키프레임의 절대 orientation 중, 세계 중력 방향에 대한 기울기(roll·pitch) 2-DoF"** 이다.
의미: *"이 키프레임의 절대 기울기는 측정치 `R_w_i_meas` 만큼 세계 중력축에 대해 기울어 있어야 하고, 그
확신의 세기는 $\Omega_{rp}$."*

**왜 "절대"가 성립하나:** 중력은 세계의 절대 기준(−Z=위)이라, IMU가 중력을 관측하면 각 KF의 기울기는
다른 프레임 없이도 세계 좌표계에 **개별적으로** 못 박을 수 있다 → 단항 절대 factor 성립. 반면 yaw·위치는
절대 기준이 없는 gauge라 개별 KF에 절대값을 못 박으면 안 되고, 그 정보는 RelPose factor(상대 제약)로만 쓴다.

**Step 4 — RelPose factor 복원** (`nfr_mapper.cpp:205-239`).

anchor KF와 나머지 모든 KF 사이의 **상대 pose** 를 factor로 만든다:

```cpp
for (int64_t other_id : m.kfs_all) {
  if (m.frame_poses.count(other_id)==0 || other_id==kf_id) continue;

  Sophus::SE3d T_w_i_o = m.frame_poses.at(other_id).getPose();
  Sophus::SE3d T_kf_o  = T_w_i_kf.inverse() * T_w_i_o;      // 상대 pose 측정치
  int o_start_idx = m.aom.abs_order_map.at(other_id).first;

  Sophus::Matrix6d d_res_d_T_w_i, d_res_d_T_w_j;
  relPoseError(T_kf_o, T_w_i_kf, T_w_i_o, &d_res_d_T_w_i, &d_res_d_T_w_j);

  Eigen::MatrixXd J; J.setZero(POSE_SIZE, asize);          // 6×asize
  J.block<6,6>(0, kf_start_idx) = d_res_d_T_w_i;          // anchor 에 대한 ∂
  J.block<6,6>(0, o_start_idx)  = d_res_d_T_w_j;          // other 에 대한 ∂

  Sophus::Matrix6d cov_new = J * cov_old * J.transpose();  // 오차 전파
  RelPoseFactor rpf;
  rpf.t_i_ns = kf_id;  rpf.t_j_ns = other_id;
  rpf.T_i_j  = T_kf_o;                                      // 측정치
  rpf.cov_inv.setIdentity();
  if (!config.mapper_no_factor_weights)
    cov_new.ldlt().solveInPlace(rpf.cov_inv);              // Ω = Σ^-1 (LDLT)
  rel_pose_factors.emplace_back(rpf);
}
```

$$
T_{ij}^{\text{meas}} = T_{wi}^{-1} T_{wj}, \qquad
\Sigma_{ij} = J\,\Sigma\,J^\top, \qquad
\Omega_{ij} = \Sigma_{ij}^{-1}
$$

> 이렇게 하나의 marginalization prior가 → (RollPitch factor 1개) + (RelPose factor N-1개)로
> 분해된다. **`abs_H`(선형, 한 점 고정)** 정보가 **재선형화 가능한 상대 제약들**로 옮겨간 것.
> 이후 맵 최적화에서 pose가 크게 움직여도 이 factor들은 매 iteration 다시 선형화된다.

**같은 $\Sigma$, 다른 $J$:** `cov_old` $=\Sigma=H^{-1}$ 는 `extractNonlinearFactors` 맨 앞에서 **딱 한 번** 계산되고,
이 marg data 의 모든 factor 가 그 **하나의 $\Sigma$ 를 공유**한다. factor 마다 바뀌는 건 $J$(전파)뿐:

$$
\Sigma_{\text{factor}}^{(k)} = J_k\,\underbrace{\Sigma}_{\text{이 marg 안에서 고정}}\,J_k^\top,\qquad \Omega^{(k)}=\big(\Sigma_{\text{factor}}^{(k)}\big)^{-1}
$$

```
              ┌─ J_rp  →  J_rp Σ J_rpᵀ → Ω_rp   (RollPitch, anchor)
   Σ = H⁻¹ ──┼─ J_1   →  J_1  Σ J_1ᵀ  → Ω_1    (RelPose anchor↔KF1)
  (한 개 고정)├─ J_2   →  J_2  Σ J_2ᵀ  → Ω_2    (RelPose anchor↔KF2)
              └─ …                              …
```

"재료($\Sigma$)는 같고 렌즈($J$)만 갈아끼운다." 단 이건 **하나의 marg data 안에서만** — 다른 `addMargData` 는
자기 $H$ → 자기 $\Sigma$ 를 새로 갖는다. (factor 마다 측정치 $z$ 도 바뀌지만 그건 prior 가 아니라 pose 에서
읽은 값이라, prior 에서 뽑는 정보 $\Omega$ 에 관해서는 "$\Sigma$ 고정 + $J$ 만 변함"이 맞다.)

`config.mapper_no_factor_weights == true`면 모든 factor의 정보 행렬을 단위행렬로 둬서
(공분산 무시) 순수 기하 제약처럼 쓴다. 기본값은 false — 실제 불확실성을 반영.

### 3-C. pose 등록

factor가 유효하면 각 KF의 pose를 `frame_poses`에 넣는다(§3 상단 코드). 이게 맵 최적화의
**초기 추정치**이자 최적화 변수가 된다.

### 3-D. prior 처리의 출력물 정리

`addMargData`로 prior 하나를 처리하면 `NfrMapper`에 **딱 4가지**가 채워진다. (선형 prior $H,b$ 자체는
factor로 변환된 뒤 **버려진다** — 이게 핵심.)

| 출력물 (멤버) | 타입 | 개수 (marg 1개당) | 채워지는 곳 | 내용 |
|---|---|---|---|---|
| **`frame_poses`** | `map<int64_t, PoseStateWithLin>` | 관여 KF 전부 | `addMargData` (64-78) | 각 KF의 절대 pose(6-DoF). 맵 최적화의 **초기값이자 변수** |
| **`roll_pitch_factors`** | `vector<RollPitchFactor>` | 0 또는 1 (IMU & full-rank) | `extractNonlinearFactors` (190-203) | `{t_ns, R_w_i_meas, cov_inv(2×2)}` — anchor의 절대 기울기 |
| **`rel_pose_factors`** | `vector<RelPoseFactor>` | N−1 (anchor↔나머지) | `extractNonlinearFactors` (205-239) | `{t_i_ns, t_j_ns, T_i_j, cov_inv(6×6)}` — 상대 pose 제약 |
| **`img_data`** | `map<int64_t, OpticalFlowInput::Ptr>` | KF들의 이미지 | `processMargData` (144-148) | 원본 이미지 — 뒤의 특징점 재검출·루프클로저용 |

**중요 1 — $H,b$ 는 남지 않는다:** `processMargData`가 pose-only로 축소 → `extractNonlinearFactors`가
$\Sigma=H^{-1}$ 로 뒤집어 factor로 복원 → 원본 선형 prior는 사라진다. **"선형 정보덩어리 → 재선형화
가능한 factor들"** 로 바뀌는 것이고, 이후 최적화는 factor만 쓴다.

**중요 2 — 여러 marg data가 누적된다:** `main`에서 모든 marg data에 대해 `addMargData`를 반복 호출
(`mapper.cpp:183-185`)하므로 위 4개는 계속 쌓인다. `frame_poses`는 KF들의 합집합,
`roll_pitch_factors`/`rel_pose_factors`는 모든 marg에서 나온 factor들의 총합.

**중요 3 — 나온 factor 는 버려지지 않고 계속 쓴다:** `roll_pitch_factors`·`rel_pose_factors` 는 멤버 벡터라
persist 하며 `emplace_back` 으로 쌓이기만 한다(clear 안 함). `optimize()` 의 iteration 루프가 매 바퀴 이
factor 전체를 `parallel_reduce`(`nfr_mapper.cpp:284-285`)로 훑어 비용에 넣고, `optimize()` 자체도 여러 번
호출된다(`mapper.cpp:343-345`, `optimize→filter→optimize`). 즉 factor 의 **정의**(측정치 $z$·가중치 $\Omega$)는
추출 때 1회 확정 후 불변이지만, factor 는 **매 iter·매 optimize 재선형화(평가)되어 계속 재사용**된다.
한 번 쓰고 버려지는 건 factor 가 아니라 그 재료였던 raw prior $H,b$ 다(중요 1).

**중요 4 — factor 가 들고 있는 건 가중치 $\Omega$ 하나가 아니다:** 각 factor 는 **[어느 KF + 측정치 $z$ + 가중치
$\Omega$]** 3가지를 저장한다(`imu_types.h:348-365`). 가중치만으론 "얼마나 세게"는 알아도 "어디로 당길지"를
모른다.

| 항목 | RelPoseFactor | RollPitchFactor | 어떻게 정해지나 |
|---|---|---|---|
| ① 토폴로지 | `t_i_ns, t_j_ns` | `t_ns` | 어느 KF 에 거는지 |
| ② 측정치 $z$ | `T_i_j` $=T_{wi}^{-1}T_{wj}$ | `R_w_i_meas` | **추출 당시 pose 에서 읽음** |
| ③ 가중치 $\Omega$ | `cov_inv`(6×6) | `cov_inv`(2×2) | **KLD 로 계산**($\Lambda=(J\Sigma J^\top)^{-1}$) |

비용은 $r^\top\Omega r,\ r=h(x)\boxminus z$ — $z$ 없으면 잔차를 못 만들고, $\Omega$ 없으면 벌점 세기를 모른다.
(§3.5 에서 **계산해서 구한 미지수**는 $\Omega$ 뿐이라 "가중치만"으로 느끼기 쉽지만, $z$ 는 별도로 pose 에서
읽어와 함께 저장된다.)

**이 세 항목(①②③)은 추출 때 확정 후 optimize 내내 불변**이다. 변하는 것은 factor 에 저장된 값이 아니라
매 iter 계산되는 **pose → 잔차 $r=h(\text{pose})\boxminus z$ → 야코비안 $J$** 다($z$ 고정, pose 이동 → $r$ 변함).
용수철로: 자연길이 $z$·강성 $\Omega$·양 끝점은 고정, 끝점 pose 가 움직여 **힘 $r$ 만** 다시 계산. (여러
`addMargData` 로 새 factor 가 **추가**되긴 해도 기존 factor 의 세 값은 안 바뀐다.)

**아직 안 나오는 것:** `feature_corners`·`feature_matches`·`feature_tracks`·`lmdb`(랜드마크)는 이 단계에선
비어 있다. 이후 `detect_keypoints → match → build_tracks → setup_opt`(§5)에서 이미지로부터 새로 만든다.
정리하면 — **prior 처리** → (pose 초기값 + factor + 이미지), **이후 단계** → (이미지에서 시각 관측 생성).
이 둘이 합쳐져 `optimize()`의 3항 비용함수(§6)가 된다.

### 3-E. 왜 굳이 factor로 쪼개나 — raw prior 를 그대로 쓰면 안 되나?

#### 3-E-0. "raw prior 그대로 쓰기"가 정확히 뭔가

prior 는 $(H,b)$ 와 **선형화 기준점** $\bar x$(marginalize 당시 pose) 한 세트다. 그대로 쓴다는 건 현재 $x$ 에
대해 $E_{\text{prior}}(x)=\tfrac12\delta^\top H\delta + b^\top\delta,\ \delta=x\boxminus\bar x$ 라는 **$\bar x$ 에 얼어붙은
2차식(포물면)** 을 페널티로 쓰는 것. gradient $g=b+H\delta$. pose 가 $\bar x$ 근처면 되지만, mapper 의 본업
(**큰 루프 클로저 보정**)에서 두 가지로 망가진다.

#### 3-E-1. 재선형화 불가 — 큰 motion에서 수치적으로 틀린다

$H=J^\top WJ,\ b=J^\top W r(\bar x)$ 는 잔차의 **1차 테일러** $r(x)\approx r(\bar x)+J\delta x$ 다. $\|\delta x\|$ 가
작을 때만 유효. 회전 잔차 $r=\log(R_{\text{meas}}R^{-1})$ 에서 $R=\exp(\phi^\wedge)\bar R$ 로 움직이면 정확한
야코비안은 오른쪽 야코비안 $J_r(\phi)=I-\tfrac{1-\cos\|\phi\|}{\|\phi\|^2}\phi^\wedge+\tfrac{\|\phi\|-\sin\|\phi\|}{\|\phi\|^3}(\phi^\wedge)^2$
을 포함한다.

- $\phi\to0$: $J_r\to I$ (얼어붙은 $J$ OK). $\phi=30°(\approx0.52\,\text{rad})$: $J_r$ 가 $I$ 에서 수 % 벗어남.

루프 클로저가 수십 도를 보정하면 얼어붙은 $J$ 는 틀린 방향·크기의 gradient 를 준다. 게다가 tangent basis
자체가 pose 와 함께 회전해 $\bar x$ 에서 정의된 $\delta x$ 좌표축이 어긋난다 → **포물면 축이 어긋난 채 pose 를
엉뚱한 곳으로 당김**. 결과: 편향·비일관 해로 수렴하거나 루프와 싸운다. factor 는 매 iter $r,J$ 를 현재
pose 에서 다시 계산($J_r(\phi)$ 재평가)하므로 큰 보정에도 정확 — 이게 N**F**R.

#### 3-E-2. gauge 위조 — 관측성(observability) 관점 ★깊은 이유★

SLAM 의 잘 알려진 **marginalization inconsistency / FEJ 문제**다. 시각+IMU 문제엔 관측 불가 **gauge 4-DoF**
(전역 위치 3 + yaw 1; IMU 는 중력만 봐서 roll-pitch 는 관측 가능)가 있고, 참 정보행렬은 그 방향으로 rank 가
빠진다: $H\,v_{\text{gauge}}=0$.

**핵심 — nullspace 방향이 선형화점에 의존한다.** yaw nullspace 벡터는 모든 state 의 현재 pose 에 의존한다.
raw prior 의 $H$ 는 $\bar x$ 에서 만든 nullspace 를 갖는데 $x\neq\bar x$ 에서 평가하면 어긋난다:

$$
H\,v_{\text{gauge}}(x)\neq 0 \;\Rightarrow\; \text{gauge 방향에 가짜 rank(=없던 정보)를 지어냄}
$$

즉 몰라야 할 전역 yaw·위치에 정보를 위조 → 추정기 과신·비일관(Huang–Mourikis–Roumeliotis OC-EKF 가
다루는 문제).

- **FEJ(First-Estimate Jacobians):** 항상 첫 추정치에서 선형화해 nullspace 를 얼려 일관성 유지 — 하지만
  여전히 선형이라 3-E-1(정확도)은 남음.
- **NFR:** 상대 factor $T_{ij}=T_{wi}^{-1}T_{wj}$ 는 **전역 프레임과 무관**(gauge-invariant, 전체를 회전·이동해도
  불변)이라 gauge 방향에 정보를 아예 안 넣는다. 관측 가능한 roll-pitch 만 절대 factor 로 남기고, 매 iter
  재선형화라 nullspace 항상 정확. → 3-E-1·2 동시 해결.

#### 3-E-3. 왜 "여러 개"인가 — 결합 분포 재현

marg prior 는 N개 KF 의 **결합 가우시안** $\mathcal{N}(\bar x, H^{-1})$, $H\in\mathbb{R}^{6N\times6N}$ 이고 비대각
블록이 KF 끼리 상관(cross-covariance)을 담는다("anchor 가 이쪽으로 틀리면 KF5 도 저쪽으로").

- 1개 factor(두 노드 6차원)로는 N-노드 결합 구조를 못 담는다.
- **star 토폴로지**(anchor↔각 KF)가 anchor-중심 상관을 재현. 개수 $6(N-1)+2=6N-4$ = 관측 가능 자유도와
  일치(§3.5-3), gauge 4 는 비움.
- 왜 fully-connected 아니고 star? 소거되는 KF 를 anchor 로 한 star 가 자연스럽고 최소·충분. 더 촘촘한
  그래프면 KLD 는 줄지만 factor 가 많아짐. (per-factor GLC 근사 §3.5-4라 cross-factor 항은 버림.)

#### 3-E-4. 보너스 — 여러 prior + 새 관측 합성

mapper 는 여러 시점 prior + 루프 클로저 시각 관측을 한 최적화에 넣는다. raw prior 들은 각자 **다른 $\bar x$**
에 묶여 그냥 더하면 비일관. 모두 factor 로 바꾸면 **공통 현재 추정치에서 다 같이 재선형화**되어 하나의
일관된 그래프로 합쳐지고, 새 reprojection factor 와도 자연스럽게 섞인다.

#### 3-E-5. 구체적 실패 시나리오

한 바퀴 돌아온 로봇, 드리프트로 "지도상 2m·20° 어긋남" 누적 → 루프 클로저가 **궤적 전체를 회전·이동**해
닫아야 함.

- **raw 얼어붙은 prior:** 각 옛 prior 가 "KF_k 는 절대 pose $\bar X_k$ 였다"고 고집. 보정이 크니 (1) 선형 근사
  부정확 + (2) 전역 회전(gauge)에 저항 → 루프가 안 닫히거나 지도가 뒤틀리거나 발산.
- **NFR factor:** "KF_k↔KF_{k+1} 은 $\Delta$ 만큼"이라는 상대 제약은 전역 프레임과 무관 → 사슬 전체가 자유롭게
  회전·이동해 루프를 닫으며 **국소 형상 보존**. 매 iter 재선형화라 큰 보정에도 정확.

> **그래서 VIO 는 왜 raw prior 를 그대로 쓰나?** VIO 는 슬라이딩 윈도우라 pose 가 선형화점 근처에서만
> 조금 움직인다 → 재선형화·gauge 문제가 덜 심각. mapper 는 정반대(여러 prior + 큰 루프 보정)라 factor
> 분해가 필수다.

#### 3-E-6. "매번 재선형화하는데 왜 추출이 따로 필요한가?"

**추출과 재선형화는 다른 일을 한다. 재선형화는 이미 정의된 factor 를 평가할 뿐, factor 를 만들지 못한다.**

각 factor 는 $r(x)=h(x)\boxminus z$, 가중치 $\Omega$ 로 생겼다. $h(x)$=pose 함수(예 $T_{wi}^{-1}T_{wj}$),
$z$=측정치(`T_i_j`,`R_w_i_meas`), $\Omega$=`cov_inv`.

| | 추출 (`extractNonlinearFactors`, **1회**) | 재선형화 (`optimize` 루프, **매 iter**) |
|---|---|---|
| 하는 일 | factor **정의** | factor **평가** |
| 산출 | 토폴로지 + 측정치 $z$ + 가중치 $\Omega$ (= "무엇을 얼마나 세게") | $r=h(x_{\text{현재}})\boxminus z$, $J=\partial r/\partial x$ |

재선형화는 $z,\Omega,h$ 를 **필요로 한다** — 그걸 만들어주는 게 추출이다.

**raw prior 는 재선형화할 게 없다:** $E=\tfrac12\delta^\top H\delta+b^\top\delta$ 는 이미 $\delta$ 에 선형(gradient
$H\delta+b$), Hessian $H$ 가 **설계상 상수**라 "다시 선형화"해도 늘 같은 $H$ → 아무 일도 안 일어남(얼어붙은
포물면). 재선형화 이득을 보려면 정보를 **pose 의 비선형 함수 $h(x)$** 로 표현해야 하고, **추출이 바로 상수
$H$ 를 "$h+z+\Omega$" 로 되살리는 변환**이다.

> **비유:** raw prior = 바닥에 그려진 고정 포물면(재선형화해도 그대로). 추출 = 그게 담은 "상대 거리·기울기 +
> 강성"을 읽어 **용수철**로 다시 씀. 재선형화 = 그 용수철에 현재 pose 넣어 힘을 다시 계산. **용수철을 만드는
> 것**(추출)과 **힘을 다시 재는 것**(재선형화)은 다른 일.

**더 깊은 이유 — 원본 관측은 이미 버려졌다:** "그냥 원본 VIO 문제를 재선형화하지?" → 원본 측정(landmark·IMU)은
윈도우 유지하려 이미 marginalize 로 버려졌다. 버린 건 재선형화 못 한다. 추출은 그걸 **소수의 factor 로 압축
복원**해 "작으면서(bounded) 재선형화 가능"을 동시에 얻는다.

| 방식 | 재선형화 가능? | 크기 |
|---|---|---|
| 원본 관측 다 보관 | ✅ | ❌ 무한정 |
| raw prior $H,b$ | ❌ 얼어붙음 | ✅ 작음 |
| **NFR factor** | ✅ | ✅ 작음 |

---

## 3.4. §3.5를 읽기 위한 배경 이론 (Gaussian · 정보행렬 · KLD 기초)

§3.5의 유도가 막히면, 보통 아래 7개 벽돌 중 하나가 안 깔려 있어서다. 순서대로 쌓으면 §3.5가
"당연한 결론"으로 읽힌다. (이미 아는 벽돌은 건너뛰어도 된다.)

### BB1. 가우시안의 두 얼굴 — 공분산형(moment) vs 정보형(canonical)

같은 가우시안을 두 가지로 쓸 수 있다.

$$
p(x) \propto \exp\!\Big(-\tfrac12 (x-\mu)^\top \Sigma^{-1}(x-\mu)\Big)
\;=\;\exp\!\Big(-\tfrac12 x^\top \Omega\, x + \eta^\top x\Big)
$$

- **공분산형(moment form):** 평균 $\mu$, 공분산 $\Sigma$. "얼마나 **퍼졌나**(불확실한가)"를 직접 말함.
- **정보형(canonical form):** 정보행렬 $\Omega=\Sigma^{-1}$ (precision/information matrix), 정보벡터 $\eta=\Omega\mu$.
  "얼마나 **확신하나**"를 말함. $\Omega$ 가 크다 = 확신이 크다 = 분포가 좁다.

| | 공분산형 | 정보형 |
|---|---|---|
| 핵심량 | $\mu,\ \Sigma$ | $\eta,\ \Omega=\Sigma^{-1}$ |
| 의미 | 퍼짐(불확실성) | 확신(정밀도) |
| 코드 대응 | `cov_old` $=\Sigma$ | `abs_H` $=\Omega=H$, `abs_b` $=b\approx\eta$ |

**이 문서의 모든 "$H$ 뒤집어 $\Sigma$" 는 정보형 ↔ 공분산형 왕복**이다. 왜 왕복하냐가 BB4의 핵심.

### BB2. 최소제곱 = 가우시안 (음의 로그우도)

비용 $E(x)=\tfrac12\, r(x)^\top W\, r(x)$ 를 최소화하는 문제는, 확률로 보면 $p(x)\propto e^{-E(x)}$ 인
가우시안을 최대화하는 것과 같다(음의 로그우도 = 비용). 현재점에서 $r\approx r_0 + J\,\delta x$ 로 선형화하면

$$
E \approx \tfrac12 (r_0 + J\delta x)^\top W (r_0 + J\delta x),\qquad
\frac{\partial E}{\partial \delta x}=0 \;\Rightarrow\; \underbrace{(J^\top W J)}_{H}\,\delta x = -\underbrace{J^\top W r_0}_{b}
$$

즉 **최소제곱 문제 하나 = 정보행렬 $H=J^\top W J$ 를 가진 가우시안 하나**다. 직관: "**비용함수의 곡률
(2차 미분) = 정보 = 정밀도**". 바닥이 뾰족할수록(곡률 큼) 최소점을 확신함(정보 큼). 이게 §1에서 봤던
$H=J^\top W J$ 가 왜 "정보행렬"인지의 이유.

### BB3. 측정은 정보를 **더한다** ($\Omega_s = \sum J_i^\top\Lambda_i J_i$)

독립 측정들의 우도는 **곱**, 로그우도는 **합**, 따라서 정보행렬도 **합**이다. factor $i$ 의 잔차가
$r_i\approx J_i\,\delta x$ 이고 그 측정 노이즈의 정밀도(=믿음)가 $\Lambda_i$ 면, 이 factor가 더하는 정보는
$J_i^\top \Lambda_i J_i$. 전부 합치면

$$
\Omega_s=\sum_i J_i^\top \Lambda_i J_i = J^\top \Lambda J
\qquad(J=\text{야코비안 세로로 쌓기},\ \Lambda=\mathrm{blkdiag}(\Lambda_i))
$$

**이것이 §3.5-1의 $\Omega_s$ 식의 정체다.** 여기서 $\Lambda_i$ 가 "이 factor를 얼마나 믿나"이고, 이게 바로
NFR이 정하려는 미지수(코드의 `cov_inv`). "정보 = 곡률"(BB2)이니, factor를 세게 믿을수록($\Lambda_i$ 큼)
비용 바닥이 그 방향으로 뾰족해진다.

### BB4. 선형변환·주변화·조건화 — 어느 형에서 쉬운가 ★가장 중요★

여기가 "왜 $H\!\to\!\Sigma$ 로 뒤집나"의 답이다.

- **선형변환** $y=J\,\delta x$: 공분산형에서 $\Sigma_y=J\Sigma J^\top$ (쉬움). ← Step3-3에서 쓴 오차 전파.
- **주변화(marginalization, 변수 일부를 적분해 없앰):** **공분산형에서는 그냥 해당 블록만 잘라내면 끝.**
  정보형에서는 Schur complement 필요(§1).

> **공분산 전파 — 왜 $J$ 로 양쪽을 감싸나(sandwich):** $y=J\delta x$ 의 공분산은 $J\Sigma$ (한 번 곱)이 아니라
> **$J\Sigma J^\top$** 이다. 공분산 정의가 벡터 두 벌의 곱이라: $y-\bar y=J(\delta x-\overline{\delta x})$ 이므로
> $\Sigma_y=\mathbb{E}[(y-\bar y)(y-\bar y)^\top]=\mathbb{E}[J(\cdots)(\cdots)^\top J^\top]=J\,\Sigma\,J^\top$ — 왼쪽 벌은
> $J$, 오른쪽(전치) 벌은 $J^\top$ 를 뒤집어쓴다. 1D 로는 $\mathrm{Var}(ax)=a^2\mathrm{Var}(x)$ 의 "제곱"이 다차원에서
> $J\cdots J^\top$ 샌드위치로 나타난 것. 차원: $\Sigma$($n{\times}n$, 상태좌표) → $J\Sigma J^\top$($m{\times}m$, factor
> 좌표). $J\Sigma$ 만 하면 일반적으로 비대칭이라 공분산이 아니다. 코드: `cov_new = J * cov_old * J.transpose()`.
- **조건화(conditioning, 변수 일부를 값으로 고정):** 정보형에서 쉬움(블록 추출), 공분산형에서 복잡.

| 연산 | 공분산형 $\Sigma$ | 정보형 $\Omega$ |
|---|---|---|
| 선형변환 $y=J x$ | $J\Sigma J^\top$ ✅쉬움 | 복잡 |
| **주변화(없애기)** | **블록 추출** ✅쉬움 | Schur complement |
| 조건화(고정) | 복잡 | 블록 추출 ✅쉬움 |

$\Rightarrow$ §3-B가 roll-pitch만 남기려면 위치·yaw를 **주변화**해야 하는데, 주변화는 **공분산형에서 블록
잘라내기**가 제일 쉽다. 그래서 $H\to\Sigma=H^{-1}$ 로 뒤집고, factor 좌표로 $J\Sigma J^\top$ 전파한 뒤,
원하는 블록만 떼서 다시 $\Omega=(\cdot)^{-1}$ 로 정보형에 되돌린다. **이 왕복의 목적이 곧 "쉬운 주변화"**다.

#### BB4-보충: "factor 좌표로 변환"이 무슨 뜻인가

두 개의 좌표계(공간)를 구분하면 명확해진다.

- **상태 좌표(state coords):** $\Sigma$ 가 사는 공간. 모든 KF의 pose 오차 $\delta x=[\delta t,\delta\theta,\dots]$
  를 쌓은 것 — "각 KF가 절대적으로 얼마나 틀렸나". 크기 = `asize`.
- **factor 좌표(residual coords):** factor가 **실제로 재는 양** = 자기 잔차 $r$. RelPose면 "두 KF 사이 상대
  pose"(6), RollPitch면 "기울기"(관심 블록 2). 전체 상태가 아니라 그 factor가 보는 몇 차원뿐이다.

잔차는 상태의 함수 $r=h(x)$ 이고 선형화하면 $\delta r = J\,\delta x$, 즉 **$J=\partial r/\partial x$ 는
"상태 좌표 → factor 좌표"로 가는 번역기(사전)** 다. 그래서

$$
\Sigma_{\text{factor}} = J\,\Sigma\,J^\top
\quad(\text{상태좌표 불확실성}\ \Sigma\ \longrightarrow\ \text{factor좌표 불확실성})
$$

는 BB4의 선형변환 규칙 그대로다. 차원도 상태공간(asize)에서 그 factor가 보는 차원(6 또는 2)으로 사영된다.

**왜 꼭 필요?** `optimize()`에서 factor는 잔차 $r$(예: `rollPitchError`가 주는 2-벡터)을 $\Omega=$`cov_inv`
로 가중한다. 이 $\Omega$ 는 반드시 **잔차 $r$ 과 같은 좌표**로 표현돼야 하는데, 우리가 가진 $\Sigma$ 는 상태
좌표다. 그래서 $J$ 로 번역해 factor 좌표로 옮긴 뒤 뒤집어야($\Omega=\Sigma_{\text{factor}}^{-1}$) factor가
바로 쓸 수 있다.

**비유:** $\Sigma$ 는 "미터·라디안(상태 단위)"로 잰 불확실성, factor는 "상대 pose·기울기"라는 다른 단위로
말한다. $J\Sigma J^\top$ 은 그 **단위(좌표계) 환산**이고 $J$ 가 환산율이다.

### BB5. KLD 가우시안 닫힌형 유도

#### BB5-0. 그 전에: $D_{KL}(p\|q)=\mathbb{E}_p[\ln p-\ln q]$ 는 어디서 오나

**표면(정의의 재작성):** 원 정의 $D_{KL}(p\|q)=\int p\ln\frac pq\,dx$ 에 $\ln\frac pq=\ln p-\ln q$ 와
$\int p\,f = \mathbb{E}_p[f]$ 를 넣으면 곧바로 $\mathbb{E}_p[\ln p-\ln q]$. 한 줄씩 보면:

- **Step 0 (기댓값 정의):** $X\sim p$ 일 때 임의 함수 $g$ 에 대해 $\mathbb{E}_p[g(X)]=\int p(x)\,g(x)\,dx$
  ("$g$ 값을 확률 $p$ 로 가중해 더함"). 이 한 줄이 전부다. (기호 $\mathbb{E}$ = 기댓값/expectation,
  아래첨자 $p$ = 분포 $p$ 기준. 책에 따라 $E,\ \mathrm{E},\ \langle\cdot\rangle$ 로도 쓴다.)

  > **기댓값 = 가중평균 (모르면 여기부터):** 주사위 눈 평균 $\frac{1+\cdots+6}{6}=3.5$ 는 사실
  > "각 눈 $\times$ 확률 $\frac16$ 을 다 더한 것". 확률이 값마다 다르면 각 값에 **그 값의 확률**을 곱해 더한다:
  > $\mathbb{E}[X]=\sum_x x\,p(x)$ (예: 100원 딸 확률 0.1 → $100\cdot0.1=10$원). 값 대신 함수값을 평균내면
  > $\mathbb{E}[g(X)]=\sum_x g(x)\,p(x)$. 값이 **연속**이면 합 $\sum$ 이 적분 $\int$ 으로 바뀔 뿐:
  > $\mathbb{E}_p[g(X)]=\int g(x)\,p(x)\,dx$. 뜻은 그대로 "$p$ 로 가중한 $g$ 의 평균".
- **Step 1:** KLD 적분 안은 $\underbrace{p(x)}_{\text{가중치}}\times\underbrace{\ln\frac{p(x)}{q(x)}}_{g(x)}$ — Step 0 모양.
- **Step 2:** $\ln\frac{p}{q}=\ln p-\ln q$ (로그 나눗셈=뺄셈).
- **Step 3:** $D_{KL}=\int p(x)\,[\ln p(x)-\ln q(x)]\,dx$.
- **Step 4:** $g=\ln p-\ln q$ 로 보면 Step 0 정의에 의해 $=\mathbb{E}_p[\ln p-\ln q]$.
- **Step 5:** 선형성으로 $=\mathbb{E}_p[\ln p]-\mathbb{E}_p[\ln q]=-H(p)-(-H(p,q))=H(p,q)-H(p)$.

> **헷갈림 방지 — $p$ 가 두 번 나오는 것:** $\mathbb{E}_p[\ln p]=\int p(x)\ln p(x)\,dx$ 에서 바깥 $p$(아래첨자)는
> "어느 분포로 가중하나"(가중치), 안의 $\ln p$ 는 "표본을 어느 함수에 넣나"(함수). 같은 $p$ 가 두 역할을 할
> 뿐 모순 아니다. 숫자 예: $p=[.5,.5],q=[.9,.1]$ → $D_{KL}=.5\ln\frac{.5}{.9}+.5\ln\frac{.5}{.1}=0.511>0$.

하지만 "왜 이 형태냐"의 진짜 답은 정보이론이다.

**근본(여분의 놀람):** 확률 $q(x)$ 사건의 **놀람(정보량)** 을 $-\ln q(x)$ 로 둔다(흔하면 덜 놀람; 로그는
독립사건의 놀람이 더해지게 함). 현실이 $p$ 를 따를 때 두 평균 놀람:

| 양 | 정의 | 의미 |
|---|---|---|
| 엔트로피 $H(p)$ | $\mathbb{E}_p[-\ln p]$ | 참 모델 $p$ 를 쓸 때의 **최소** 평균 놀람 |
| 교차엔트로피 $H(p,q)$ | $\mathbb{E}_p[-\ln q]$ | 틀린 모델 $q$ 를 쓸 때의 평균 놀람 |

> **부호 주의:** 엔트로피는 $\ln p$ 가 아니라 $-\ln p$ 의 기댓값이다. 즉 $H(p)=\mathbb{E}_p[-\ln p]=-\mathbb{E}_p[\ln p]$
> — $\mathbb{E}_p[\ln p]$ 그대로면 $-H(p)$ 다. 마이너스가 붙는 이유: $p\le1$ 이라 $\ln p\le0$(음수)이므로 놀람
> $-\ln p\ge0$ 을 양수로 만든다. 흔한 일($p$ 큼)은 덜 놀람, 드문 일은 많이 놀람 → **엔트로피 = 평균 놀람
> = 불확실성**. 예: 공정한 동전 $[.5,.5]\Rightarrow H=-\ln.5=0.693$(최대), 편향 $[.9,.1]\Rightarrow0.325$,
> 확정 $[1,0]\Rightarrow0$. 두 정의 모두 $-\ln(\cdot)$ 의 기댓값이고 안에 $p$ 냐 $q$ 냐만 다르다.

$$
D_{KL}(p\|q) = H(p,q)-H(p) = \mathbb{E}_p[-\ln q]-\mathbb{E}_p[-\ln p] = \mathbb{E}_p[\ln p-\ln q]
$$

즉 **"현실은 $p$ 인데 $q$ 라 착각해서 사건당 낭비하는 정보량(여분의 놀람/비트)"**. 부호화로 보면 "$q$ 용
코드북을 $p$ 데이터에 써서 버리는 비트 수", 통계로 보면 "데이터가 $q$ 아닌 $p$ 에서 왔다는 샘플당
로그우도비 증거량" $\big(\frac1n\sum\ln\frac{p}{q}\to D_{KL}\big)$ — NFR이 최소화하는 게 바로 이 "근사와 참의
구별 가능성"이다.

**왜 ≥ 0 (Jensen/Gibbs):** $-\ln$ 볼록성으로
$D_{KL}=\mathbb{E}_p[-\ln\frac qp]\ge -\ln\mathbb{E}_p[\frac qp]=-\ln\int q=-\ln 1=0$, 등호는 $p=q$ 일 때만.
(비대칭이라 진짜 거리는 아님.)

#### BB5-1. 가우시안 닫힌형

$D_{KL}(p\|q)=\mathbb{E}_p[\ln p-\ln q]$. 가우시안 로그밀도
$\ln p = -\tfrac12(x-\mu)^\top\Sigma_p^{-1}(x-\mu)-\tfrac12\ln((2\pi)^k\det\Sigma_p)$ 를 넣고 **평균이 같다**($\mu_p=\mu_q$)고 두면:

> **뒤의 $(2\pi)^k\det\Sigma$ 항이 뭔가 (정규화 상수):** 가우시안 밀도는
> $p(x)=\frac{1}{\sqrt{(2\pi)^k\det\Sigma}}\exp(-\tfrac12(x-\mu)^\top\Sigma^{-1}(x-\mu))$ 이고, 앞의
> $\frac{1}{\sqrt{(2\pi)^k\det\Sigma}}$ 가 **$\int p\,dx=1$ 을 만드는 정규화 상수**($x$ 와 무관)다. 로그를 씌우면
> $\ln\frac{1}{\sqrt Z}=-\tfrac12\ln Z$ 라 그 항이 나온다. 1D의 익숙한 $\frac{1}{\sqrt{2\pi\sigma^2}}$ 를 k차원으로
> 일반화한 것: $\sigma^2\to\det\Sigma$(퍼짐의 부피), $2\pi\to(2\pi)^k$(차원마다 하나, 가우시안 적분
> $\int e^{-t^2/2}dt=\sqrt{2\pi}$ 에서 옴).
> **KLD에서 왜 사라지나:** $p,q$ 가 같은 차원 $k$ 라 $\ln p-\ln q$ 의 정규화 차이
> $=\tfrac12\ln\frac{(2\pi)^k\det\Sigma_q}{(2\pi)^k\det\Sigma_p}=\tfrac12\ln\frac{\det\Sigma_q}{\det\Sigma_p}$ 로
> **$(2\pi)^k$ 가 약분**된다. 그래서 아래 최종식엔 $2\pi$ 가 없고 $\det$ 비만 남는다.

- $\mathbb{E}_p[(x-\mu)^\top\Sigma_q^{-1}(x-\mu)] = \mathrm{tr}(\Sigma_q^{-1}\Sigma_p)$ (트레이스 트릭 + $\mathbb{E}_p[(x-\mu)(x-\mu)^\top]=\Sigma_p$)
- $\mathbb{E}_p[(x-\mu)^\top\Sigma_p^{-1}(x-\mu)] = \mathrm{tr}(I_k)=k$

> **트레이스 트릭 유도** ($y\equiv x-\mu$): 준비물 — (a) 스칼라 $y^\top\Sigma_q^{-1}y$($1\times1$)는 자기
> trace와 같다, (b) 순환성 $\mathrm{tr}(AB)=\mathrm{tr}(BA)$, (c) $\mathbb{E}_p[yy^\top]=\Sigma_p$(공분산 정의).
> $$
> \mathbb{E}_p[y^\top\Sigma_q^{-1}y]
> \overset{(a)}{=}\mathbb{E}_p[\mathrm{tr}(y^\top\Sigma_q^{-1}y)]
> \overset{(b)}{=}\mathbb{E}_p[\mathrm{tr}(\Sigma_q^{-1}yy^\top)]
> =\mathrm{tr}(\Sigma_q^{-1}\underbrace{\mathbb{E}_p[yy^\top]}_{\Sigma_p})
> =\mathrm{tr}(\Sigma_q^{-1}\Sigma_p)
> $$
> 핵심: 원식은 $y$ 가 $\Sigma_q^{-1}$ 를 양쪽에서 감싸 기댓값을 못 넣는데, trace로 돌려 $yy^\top$ 을 한 덩어리로
> 모으면 거기에만 $\mathbb{E}_p[yy^\top]=\Sigma_p$ 를 꽂을 수 있다. (인덱스 검증:
> $\mathbb{E}_p[\sum_{ij}A_{ij}y_iy_j]=\sum_{ij}A_{ij}(\Sigma_p)_{ij}=\mathrm{tr}(A\Sigma_p)$, $A=\Sigma_q^{-1}$.)
> 두 번째 줄은 $\Sigma_q\to\Sigma_p$ 특수경우: $\mathrm{tr}(\Sigma_p^{-1}\Sigma_p)=\mathrm{tr}(I_k)=k$.

정리하면

$$
2\,D_{KL}(p\|q) = \mathrm{tr}(\Sigma_q^{-1}\Sigma_p) - k + \ln\frac{\det\Sigma_q}{\det\Sigma_p}
= \mathrm{tr}(\Sigma_q^{-1}\Sigma_p) - k - \ln\det(\Sigma_q^{-1}\Sigma_p)
$$

**항의 직관:** $\mathrm{tr}(\Sigma_q^{-1}\Sigma_p)$ 는 "q의 자로 잰 p의 퍼짐"(q가 너무 좁으면 폭발) — **과신 벌점**.
$-\ln\det$ 항은 q가 너무 넓으면 벌점 — **과소확신 벌점**. 둘의 균형점이 $\Sigma_q=\Sigma_p$ 에서 $D_{KL}=0$.

### BB6. 필요한 행렬미분 두 개

§3.5-2에서 $\Lambda$ 로 미분할 때 쓰는 공식(둘 다 표준):

$$
\frac{\partial}{\partial\Lambda}\,\mathrm{tr}(J^\top\Lambda J\,\Sigma_t)
\;\overset{\text{cyclic}}{=}\; \frac{\partial}{\partial\Lambda}\,\mathrm{tr}(\Lambda\, J\Sigma_t J^\top) = J\Sigma_t J^\top
$$

$$
\frac{\partial}{\partial\Lambda}\,\ln\det(J^\top\Lambda J) = J\,(J^\top\Lambda J)^{-1} J^\top
\qquad(\text{연쇄법칙} + \partial\ln\det B/\partial B = B^{-1})
$$

이 둘을 빼서 0 으로 놓은 게 §3.5-2의 정지조건이다.

### BB7. gauge · observability · nullspace

- **gauge freedom(게이지 자유도):** 전역 기준을 임의로 정할 수 있는 방향. VIO/시각에서는 **절대 원점 위치(3)
  + 중력축 둘레 절대 방위(yaw,1) = 4-DoF**. 카메라·IMU는 "세계 어디에, 어느 방위로" 있는지 절대값을 못
  본다(가속도계는 **중력 방향만** 봄 → roll-pitch만 절대 관측 가능).
- 정보행렬에서 gauge 방향 $v$ 는 $H v = 0$ (null space) → **rank 부족**. 단, VIO는 초기 prior로 gauge를
  고정하므로 보통 full-rank(§3-B-Step3-1).
- **observable(관측 가능) 방향 = null space에 직교 = $J$ 의 행공간.** §3.5-3의 "행공간에서만 매칭"이 이 말.

> **§3.4 → §3.5 다리:** BB2·BB3으로 "$\Omega=J^\top\Lambda J$ 가 factor들이 만든 가우시안"임을, BB1·BB4로
> "왜 $\Sigma$ 로 뒤집어 블록을 다루는지"를, BB5·BB6으로 "KLD를 $\Lambda$ 로 미분하면 무엇이 나오는지"를,
> BB7로 "gauge 때문에 $J$ 가 비정방이어도 되는 이유"를 깔았다. 이제 §3.5는 이 벽돌들을 조립하는 것뿐이다.

---

## 3.5. 이론적 배경 — 왜 이게 KLD 최소화인가 ★NFR의 근본★

§3-B의 "$H\to\Sigma\to J\Sigma J^\top\to$ 역행렬" 절차가 **왜 하필 그 형태**인지는, NFR(Non-linear Factor
Recovery)의 원 이론인 **KL Divergence 최소화**(Mazuran et al., *Nonlinear Graph Sparsification for SLAM*,
RSS 2014; 그리고 그 전신인 Carlevaris-Bianco & Eustice의 GLC)로 설명된다. 이 절은 그 유도를 담는다.

### 3.5-0. KLD(Kullback-Leibler Divergence) 기본

두 확률분포 $p, q$ 가 있을 때 KLD는 "$p$ 를 $q$ 로 근사했을 때 잃는 정보량"이다:

$$
D_{KL}(p\,\|\,q) = \int p(x)\,\ln\frac{p(x)}{q(x)}\,dx = \mathbb{E}_p\!\left[\ln\frac{p}{q}\right] \ \ge 0
$$

- 항상 0 이상이고, $p=q$ 일 때만 0. 비대칭($D_{KL}(p\|q)\neq D_{KL}(q\|p)$).
- **평균이 같은** 두 가우시안 $p=\mathcal{N}(\mu,\Sigma_p)$, $q=\mathcal{N}(\mu,\Sigma_q)$ 사이에서는 닫힌 형태:

$$
2\,D_{KL}(p\,\|\,q) = \mathrm{tr}\!\big(\Sigma_q^{-1}\Sigma_p\big) - k - \ln\det\!\big(\Sigma_q^{-1}\Sigma_p\big)
$$

($k$ = 차원). 즉 두 공분산이 얼마나 다른지를 재는 척도다.

### 3.5-1. NFR 문제를 KLD로 정식화

- **목표 분포(target)** $p$: marginalization prior. 접선공간(오차 $\delta x$) 기준, 평균 0, 정보 행렬
  $\Omega_t = H$ (`abs_H`), 공분산 $\Sigma_t = H^{-1}$ (`cov_old`). 이것이 "참"이고 조밀(dense)하다.
- **근사 분포(approx)** $q$: 우리가 고른 **성긴(sparse) 비선형 factor들**이 만드는 분포. 각 factor $i$ 의
  잔차를 선형화하면 $r_i \approx J_i\,\delta x$ 이고(측정치를 현재 추정치로 잡아 평균 0), 정보 행렬은

$$
\Omega_s = \sum_i J_i^\top \Lambda_i\, J_i = J^\top \Lambda\, J
$$

  여기서 $J$ 는 모든 factor 야코비안을 세로로 쌓은 것, $\Lambda = \mathrm{blkdiag}(\Lambda_i)$ 는 우리가
  **정하려는 미지수 = 각 factor의 정보 행렬**(코드의 `cov_inv`).

**문제:** *factor 토폴로지(누가 누구와 연결되는지, 즉 $J$)를 고정한 채, $D_{KL}(p\|q)$ 를 최소화하는
$\Lambda$ 를 구하라.* 이게 NFR이다.

### 3.5-2. 최소화 → 왜 $\Lambda = (J\Sigma_t J^\top)^{-1}$ 이 나오나

$\Sigma_p=\Sigma_t$, $\Sigma_q=\Omega_s^{-1}=(J^\top\Lambda J)^{-1}$ 를 위 KLD 식에 넣으면($\Lambda$ 무관 상수 생략):

$$
2\,D_{KL} = \underbrace{\mathrm{tr}\big(J^\top\Lambda J\,\Sigma_t\big)}_{(1)} \;-\; \underbrace{\ln\det\big(J^\top\Lambda J\big)}_{(2)} \;+\; \text{const}
$$

$\Lambda$ 로 미분해 0 으로 둔다(행렬 미분 공식 $\partial\,\mathrm{tr}(J^\top\Lambda J\,\Sigma_t)/\partial\Lambda = J\Sigma_t J^\top$,
$\partial\ln\det(J^\top\Lambda J)/\partial\Lambda = J(J^\top\Lambda J)^{-1}J^\top$):

$$
J\,\Sigma_t\,J^\top - J\,(J^\top\Lambda J)^{-1} J^\top = 0
\quad\Longrightarrow\quad
J\big[\Sigma_t - (J^\top\Lambda J)^{-1}\big]J^\top = 0
$$

$J$ 가 정방·가역이면(측정 자유도 = 상태 자유도) $\Sigma_t = (J^\top\Lambda J)^{-1}$, 정리하면

$$
\boxed{\;\Lambda = \big(J\,\Sigma_t\,J^\top\big)^{-1}\;}
$$

**이것이 코드의 정체다.** `cov_new = J * cov_old * J^T` 가 $J\Sigma_t J^\top$, 이를 역행렬 낸
`cov_new.inverse()`(또는 LDLT solve)가 $\Lambda$ = factor의 `cov_inv`. 즉 §3-B의 절차는
**"이 factor 토폴로지로 target 분포를 KLD 최소로 재현하는 정보 행렬"** 을 계산하는 것이다. $J$ 가
가역일 때는 KLD = 0, 즉 **완벽 복원**(정보 손실 없음).

### 3.5-3. gauge가 있는 경우(비정방 $J$) — observable 부분만 매칭

basalt의 실제 토폴로지는 **star 그래프**다: anchor에 RollPitch factor 1개(2-DoF) + anchor↔다른 KF마다
RelPose factor(6-DoF). 전체 측정 자유도 $= 2 + 6(N-1) = 6N-4$, 상태 자유도 $=6N$. **딱 4 모자란다** —
바로 전역 gauge 4-DoF(위치 3 + yaw 1). 그래서 stacked $J$ 는 정방이 아니고 4차원 null space(gauge)를 가진다.

이 경우 위 정지조건 $J[\Sigma_t - \Sigma_s]J^\top=0$ 은 "$J$ 의 **행공간(=관측 가능 방향)** 에서만 $\Sigma_s=\Sigma_t$
를 요구"한다. gauge 방향은 애초에 factor로 묶을 수 없으니(묶으면 안 되니) 그대로 두고, **관측 가능한
부분공간에서만 target을 정확히 재현**한다. 여전히 해는 같은 형태 $\Lambda=(J\Sigma_t J^\top)^{-1}$ 이다.
이래서 §3-B에서 roll-pitch(2-DoF, 관측 가능)만 절대 factor로 만들고 위치·yaw(gauge)는 버렸던 것이 —
KLD 관점에서도 정확히 맞는 선택이 된다.

### 3.5-4. basalt는 factor별 독립 복원(GLC 방식)

이론적으로 $\Lambda=(J\Sigma_t J^\top)^{-1}$ 를 **전체 $J$** 로 한 번에 풀면 결과는 일반적으로
**블록대각이 아니다**(factor끼리 교차항 발생). 하지만 basalt는 각 factor를 **따로** 처리한다 — factor $i$ 마다
자기 야코비안 $J_i$ 와 전체 $\Sigma_t$ 로

$$
\Lambda_i = \big(J_i\,\Sigma_t\,J_i^\top\big)^{-1}
$$

를 독립적으로 계산한다(코드에서 RollPitch·RelPose 각각 `cov_new = J*cov_old*J^T` 후 inverse). 이는
전체 KLD 해에서 **교차 블록을 버린 블록대각 근사** = GLC(Generic Linear Constraints)식 "per-factor" 복원이다.
factor들의 행공간이 $\Sigma_t$-직교($J_i\Sigma_t J_j^\top=0$)일 때 정확하고, 아니면 좋은 근사다. 그 대가로
factor 그래프가 성기게(sparse) 유지되어 맵 최적화가 빨라진다 — 이것이 애초에 sparsification을 하는 이유.

> **한 줄 정리:** marg prior는 "관측된 것들의 조밀한 가우시안"이다. NFR은 그것을 *"내가 고른 몇 개의
> 상대/절대 factor로 만든 가우시안"* 으로 **KLD 최소로 바꿔치기**하는 것이고, 그 최적 가중치가
> $\Lambda=(J\Sigma J^\top)^{-1}$ — 즉 "$H$ 뒤집어 $\Sigma$ 얻고 → factor 좌표로 전파 $J\Sigma J^\top$ →
> 다시 뒤집기"다. 코드가 그렇게 생긴 근본 이유가 이것이다.

---

## 4. 4개의 잔차/야코비안 상세 (`utils/nfr.h`)

`extractNonlinearFactors`와 `optimize`가 공유하는 잔차 함수들. 모두 `T_w_i` = world→imu pose,
접선공간(tangent) 순서는 **[translation(3), rotation(3)]**.

### 4-1. `relPoseError` (`nfr.h:43-73`) — 상대 pose 잔차

$$
r_{\text{rel}} = \log\!\big(T_{ij}^{\text{meas}} \cdot (T_{wj}^{-1} T_{wi})\big) \in \mathbb{R}^6
$$

```cpp
Sophus::SE3d   T_j_i = T_w_j.inverse() * T_w_i;
Sophus::Vector6d res = Sophus::se3_logd(T_i_j * T_j_i);   // log-map (decoupled)
```

야코비안은 오른쪽 역야코비안 $J_r^{-1}$ 과 adjoint로 구성:

```cpp
Sophus::rightJacobianInvSE3Decoupled(res, J);
Eigen::Matrix3d R = T_w_i.so3().inverse().matrix();
Adj = blockdiag(R, R);
*d_res_d_T_w_i =  J * Adj;
// other 쪽은 translation 커플링 항 추가 후 부호 반전
Adj.topRightCorner<3,3>() = SO3::hat(T_j_i.inverse().translation()) * R;
*d_res_d_T_w_j = -J * Adj;
```

### 4-2. `absPositionError` (`nfr.h:75-84`) — 절대 위치 잔차

$$
r_{\text{pos}} = t_{wi} - t^{\text{meas}} \in \mathbb{R}^3, \qquad
\frac{\partial r}{\partial T_{wi}} = [\,I_{3}\ \ 0_{3}\,]
$$

(translation 부분만 항등, rotation 부분 0). `extractNonlinearFactors`에서 **공분산 전파용**으로만
쓰이고 factor로 저장되진 않는다 (yaw/position은 gauge라 버림).

### 4-3. `yawError` (`nfr.h:86-100`) — yaw 잔차

body 축 $d$ 를 world로 돌린 뒤 **y성분**을 잔차로:

$$
r_{\text{yaw}} = \big(R_{wi}\, d\big)_y, \qquad d = R_{wi}^{-1}\,e_x
$$

```cpp
Eigen::Vector3d tmp = curr_R_w_i * yaw_dir_body;   // d = R^-1 e_x 로 잡았으므로 tmp≈e_x
double res_yaw = tmp[1];
(*d_res_d_T_w_i)[3] = -tmp[2];    // ∂/∂(rot_x)
(*d_res_d_T_w_i)[5] =  tmp[0];    // ∂/∂(rot_z)
```

역시 공분산 전파에만 사용.

#### 왜 e_x 를 곱하나 (e_z 가 아니라)?

yaw 는 **세계 Z(중력)축 둘레의 회전**이다. 어떤 축 둘레의 회전을 감지하려면 **그 축과 나란하지 않은
(수직인) 벡터**가 어떻게 움직이는지를 봐야 한다:

- Z축 둘레로 돌리면 → **수평면(XY)에 있는 벡터**들이 움직인다.
- 반면 **Z축 자신(e_z)** 은 Z 회전에 대해 **불변**이다(회전축은 그 회전에 안 움직임). 그래서 e_z 를 쓰면
  yaw 정보가 **0** 이 된다.

그래서 수평 기준 벡터 **e_x** 를 쓴다. 작은 yaw $\delta\psi$ 에 대해 $R_z(\delta\psi)\,e_x \approx (1,\ \delta\psi,\ 0)$
이라 **Y성분 ≈ yaw 각도**가 된다(그래서 `res_yaw = tmp[1]`).

**야코비안으로 확인:** 세계좌표 섭동 $R_{wi}\to\exp(\delta\theta^\wedge)R_{wi}$ 에서
$\delta\text{tmp}=\delta\theta\times\text{tmp}$, 그 Y성분은

$$
\delta(\text{res}_{yaw}) = (\delta\theta\times\text{tmp})_y
= \delta\theta_z\,\text{tmp}_x - \delta\theta_x\,\text{tmp}_z
$$

→ 코드의 `d[5]=tmp[0]`, `d[3]=-tmp[2]` 와 일치. 선형화점에서 `tmp=e_x=(1,0,0)` 이므로
$\partial/\partial\delta\theta_z = 1$(yaw만 감지), $\partial/\partial\delta\theta_x = 0$. 만약 **e_z** 를 썼다면
`tmp=e_z=(0,0,1)` 이라 $\partial/\partial\delta\theta_z = \text{tmp}_x = 0$ → yaw 를 전혀 못 재고 오히려
$\delta\theta_x$(roll)를 재게 된다. 즉 e_z 는 yaw 에 대해 정확히 잘못된 선택.

**예쁜 대칭:** yaw 는 "수평면 내 회전"이라 **수평** 프로브(e_x)로, roll-pitch 는 "수직축의 기울어짐"이라
**수직** 프로브(e_z, §4-4)로 잰다. 서로 짝을 이룬다.

| | 재는 것 | 프로브 벡터 | 읽는 성분 |
|---|---|---|---|
| yaw | Z축 둘레 회전 | 수평 e_x | Y |
| roll-pitch | Z축의 기울어짐 | 수직 e_z(중력) | X, Y |

### 4-4. `rollPitchError` (`nfr.h:102-119`) — roll/pitch 잔차

world의 중력 방향(-Z)을, 측정 회전과 현재 회전의 차이로 돌렸을 때 **x,y 성분**:

$$
r_{rp} = \Big(\big(R_{wi}^{\text{meas}}\, R_{wi}^{-1}\big)\,(-e_z)\Big)_{xy} \in \mathbb{R}^2
$$

```cpp
Eigen::Matrix3d R = (R_w_i_meas * T_w_i.so3().inverse()).matrix();
Eigen::Vector3d res = R * (-Eigen::Vector3d::UnitZ());
// 야코비안: rotation 부분만 채움
(*d_res_d_T_w_i)(0,3) = -R(0,1);  (*d_res_d_T_w_i)(1,3) = -R(1,1);
(*d_res_d_T_w_i)(0,4) =  R(0,0);  (*d_res_d_T_w_i)(1,4) =  R(1,0);
return res.head<2>();
```

> 현재 회전이 측정과 일치하면 $R = I$ 라 $R(-e_z) = -e_z$, x,y 성분 = 0 → 잔차 0.
> 중력(−Z)에서 얼마나 기울었는지를 x,y 두 성분으로 측정 = roll·pitch.

---

## 5. 루프 클로저 & 트랙 생성 (시각 관측 복원)

prior에서 factor를 복원하는 것과 **별개로**, NfrMapper는 저장해 둔 이미지로 새 시각 관측을 만든다.
VIO가 실시간 제약으로 놓친 과거-현재 매칭(루프)을 여기서 찾는다.

1. **`detect_keypoints()`** (`nfr_mapper.cpp:455-511`): 각 이미지에서 코너 검출 →
   angle/descriptor 계산 → unproject로 3D 방향 → **BoW(HashBow) 벡터** 계산 후 DB에 등록.

   > **descriptor 는 어디서/어떻게 뽑나** (`detect_keypoints` 안, 프레임마다 3단계, 실제 구현은
   > `src/utils/keypoints.cpp`):
   > - **① 코너 검출** `detectKeypointsMapping` (keypoints.cpp:136): 16→8bit 변환 후 OpenCV
   >   `goodFeaturesToTrack`(Shi-Tomasi)로 코너 위치만 (`mapper_detection_num_points` 개). VIO 의 실시간
   >   FAST-격자 검출과 달리 매퍼는 이 함수를 쓴다.
   > - **② 방향** `computeAngles` (keypoints.cpp:252): 패치의 **intensity centroid** 모멘트 $m_{01},m_{10}$ →
   >   `atan2(m01,m10)`. ORB 방향과 동일, descriptor 회전보정용(회전 불변성).
   > - **③ descriptor** `computeDescriptors` (keypoints.cpp:283) ★: **256비트 회전보정 BRIEF(=ORB 식)**.
   >   ORB 표준 `pattern_31` 샘플 쌍 256개를 ②의 각도로 회전시킨 뒤 두 점 밝기 비교
   >   `descriptor[i] = img(a) < img(b)` → `std::bitset<256>` (`corner_descriptors`).
   >
   > 이후 **매칭**은 Hamming 거리(XOR+popcount, `matchFastHelper` keypoints.cpp:325) + second-best ratio,
   > **BoW** 는 이 256비트를 `HashBow::compute_bow` 로 hash-word 화해 루프클로저 DB 에 등록(§5-1).

2. **`match_stereo()`** (`513-553`): 같은 시각 좌/우 카메라 매칭. essential matrix 기반
   inlier 필터(`findInliersEssential`). `T_i_j = T_c0_c1` 은 캘리브레이션으로 이미 알고 있음.

3. **`match_all()`** (`555-669`): 각 프레임의 BoW로 DB를 질의(`querry_database`)해
   유사 프레임 후보 뽑고 → descriptor 매칭 → **RANSAC 기하 검증**(`findInliersRansac`).
   이것이 **루프 클로저** 매칭.

4. **`build_tracks()`** (`671-695`): inlier 매칭들을 이어붙여 여러 프레임에 걸친 **feature track**
   생성 (union-find류 `TrackBuilder`), 충돌 트랙 제거, 최소 길이 필터.

5. **`setup_opt()`** (`697-756`): 각 track의 첫 관측을 host로 잡고, 두 관측 사이 baseline이 충분하면
   **삼각측량(triangulate)** 해 랜드마크(역깊이 파라미터화, `StereographicParam`)를 만들어
   `lmdb`(landmark DB)에 등록. 관측들도 `addObservation`으로 등록.

---

## 5-1. 루프 디텍션(loop detection) 상세 — HashBow 동작 원리

> Q: "basalt에서 루프 디텍팅은 어떻게 하나?" 에 대한 정리.

basalt는 **DBoW2 같은 사전학습(vocabulary tree) BoW를 쓰지 않는다.** 대신 자체
**`HashBow`**(`include/basalt/hash_bow/hash_bow.h`)라는 *training-free* hash 기반 bag-of-words로
루프 후보를 찾는다. vocabulary 파일이 필요 없고, descriptor 비트를 그대로 해시로 쓴다.

또한 루프 디텍션은 **실시간 VIO(`vio` 앱)에는 없다.** 오직 **오프라인 매핑 앱(`mapper`,
`src/mapper.cpp`)** 의 `NfrMapper` 안에서만 동작한다. 즉 VIO가 실시간 제약 때문에 못 잡은 과거-현재
연결을, 매핑 단계에서 BoW로 다시 찾는 구조다.

### (1) descriptor → hash word: `compute_hash()`

- 코너 descriptor는 256비트(`std::bitset<256>`, ORB류). `HashBow<256>`.
- 미리 정해진 **랜덤 비트 순열**(`random_bit_permutation[512]`, 하드코딩 상수)에서 앞쪽
  `num_bits`개 비트만 뽑아 `FeatureHash`(`std::bitset<32>`, `FEATURE_HASH_MAX_SIZE=32`)로 축약한다.
- `num_bits = config.mapper_bow_num_bits`(기본 **16**). 즉 256비트 중 지정된 16개 비트만 골라 만든
  16비트 값이 그 feature의 "word"가 된다. → word 공간 크기 $2^{16}$.
- 학습된 클러스터 중심이 아니라 **"고정 순열로 고른 비트 = word"** 이므로 사전(vocabulary)이 불필요.

### (2) 프레임 → BoW 벡터: `compute_bow()`

- 한 프레임의 모든 descriptor를 hash로 바꾸고, 같은 word끼리 개수를 센 뒤 **L1 정규화**한
  희소 벡터 `HashBowVector = vector<pair<FeatureHash,double>>` 를 만든다 (term-frequency).

### (3) DB 등록: `add_to_database()` — inverted index

- `inverted_index: word → [(TimeCamId, weight), ...]` 형태의 **역색인**(TBB concurrent map).
- 각 프레임의 각 word를, 그 word를 가진 프레임 목록에 추가. 질의 때 후보 프레임을 빠르게 모으기 위함.

### (4) 후보 질의: `querry_database()` (오타지만 코드상 함수명 그대로)

- 질의 프레임의 BoW 각 word에 대해 역색인을 훑어, word를 공유하는 프레임들에 점수를 누적.
- 점수식(코드 `hash_bow.h:85-93`)은 **L1 거리 기반 유사도**:
  누적한 $\sum(|q_w - v_w| - |q_w| - |v_w|)$ 을 $-\tfrac12$ 배 → 두 L1-정규화 벡터의
  **공유 word에서의 근접도**. 값이 클수록 유사.
- `max_t_ns` 인자로 **자기 시점보다 과거 프레임만**(`frame_id < max_t_ns`) 후보로 제한 → 미래 매칭 방지.
- 상위 `num_results`개만 `partial_sort`로 반환.

### (5) `match_all()` 에서의 루프 디텍션 파이프라인 (`nfr_mapper.cpp:555-669`)

```
각 프레임 tcid 마다:
  querry_database(bow, mapper_num_frames_to_match=30, results, &tcid.frame_id)
    → 과거 프레임 중 BoW 유사도 상위 30개 후보
  for 후보 otcid:
    if otcid != 자기자신 and score > mapper_frames_to_match_threshold(0.04):
        (i, j) 매칭 대기열 ids_to_match 에 추가       # ← 여기까지가 "loop detection"
── 병렬 기하 검증 ──
  for 각 후보쌍:
    matchDescriptors(hamming ≤ 70, ratio 1.2)
    if matches > mapper_min_matches(20):
        findInliersRansac(threshold mapper_ransac_threshold=5e-5)   # ← 기하 검증 = loop closure 확정
    if inliers 있음: feature_matches 에 등록
```

- **탐지(detection)** = BoW 질의로 유사 과거 프레임 후보를 뽑는 단계.
- **확정(verification/closure)** = descriptor 매칭 + **RANSAC 기하 검증**으로 실제 대응이 있는지 검사.
  살아남은 매칭만 `feature_matches`가 되고, 이후 `build_tracks()`에서 track으로 이어져
  `optimize()`의 reprojection 잔차로 들어가 **전역 궤적을 실제로 당겨 맞춘다**(loop closure 효과).

### (6) 관련 config (기본값은 `src/utils/vio_config.cpp:94-102`, 데이터셋 json에도 동일)

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `mapper_bow_num_bits` | 16 | descriptor 256비트 중 word로 쓸 비트 수(→ $2^{16}$ word) |
| `mapper_num_frames_to_match` | 30 | 프레임당 BoW 상위 후보 개수 |
| `mapper_frames_to_match_threshold` | 0.04 | 후보 채택 BoW 유사도 임계값 |
| `mapper_detection_num_points` | 800 | 프레임당 검출 코너 수 |
| `mapper_max_hamming_distance` | 70 | descriptor 매칭 허용 해밍거리 |
| `mapper_second_best_test_ratio` | 1.2 | 1st/2nd best ratio 테스트 |
| `mapper_min_matches` | 20 | 기하 검증 시도 최소 매칭 수 |
| `mapper_ransac_threshold` | 5e-5 | RANSAC inlier 임계값 |

> **요약**: basalt의 루프 디텍션 = (오프라인 매퍼에서) 프레임 descriptor를 고정 비트순열로
> 16비트 hash-word로 만들어 inverted index에 쌓고 → 각 프레임이 과거 프레임을 BoW L1 유사도로 질의해
> 후보를 얻고 → descriptor 매칭 + RANSAC 기하 검증으로 확정한다. 사전학습 vocabulary가 필요 없는
> HashBow 방식이며, 확정된 매칭은 track→reprojection 잔차로 전역 최적화에 반영된다.

---

## 6. 맵 최적화 `optimize()` — 모든 걸 하나로 푼다

`nfr_mapper.cpp:244-423`. 최종 비용함수는 **세 항의 합**:

$$
E(\mathcal{X}) =
\underbrace{\sum \rho\big(r_{\text{vis}}^\top \Sigma_{\text{px}}^{-1} r_{\text{vis}}\big)}_{\text{reprojection}}
+ \underbrace{\sum_{ij} r_{ij}^\top \Omega_{ij}\, r_{ij}}_{\text{RelPose factors}}
+ \underbrace{\sum_{k} r_{rp,k}^\top \Omega_{rp,k}\, r_{rp,k}}_{\text{RollPitch factors}}
$$

- $r_{\text{vis}}$: reprojection 잔차 (Huber $\rho$ robust) — `linearizeHelper` / `lmdb` 관측에서.
- $r_{ij}, r_{rp,k}$: §3-B에서 복원한 factor들 (§4의 잔차).

#### 선형화·재선형화란? (§3-E에서 계속 나온 개념)

문제는 **비선형**(잔차 $r(x)$ 가 SE3 pose 에 대해 휘어짐)이라 한 방에 못 푼다. 그래서 **반복**한다: 현재 추정치
$x_k$ 에서 곡선을 1차 직선으로 근사($r(x)\approx r(x_k)+J\,\delta x$, $J=\partial r/\partial x|_{x_k}$) → 그 직선 문제 풀어
한 걸음 $\Delta$ → 이동. 이 "곡선→직선 갈아끼우기"가 **선형화**, 야코비안 $J$ 는 "지금 지점의 기울기".

한 걸음 옮기면 $x_k\to x_{k+1}$ 로 pose 가 바뀌고, 곡선이 휘었으니 **기울기 $J$ 도 달라진다.** 그래서 다음 걸음
전에 **새 위치에서 $r,J$ 를 다시 계산** = **재선형화**. $J_0=J(x_0)\to J_1=J(x_1)\to\cdots$ 매 iter 새로 잰다.

> **비유(안개 속 하산):** 발밑 경사 보고 한 걸음 → 멈춰 새 위치 경사 다시 보고 또 한 걸음 → … 매번 다시
> 재는 게 재선형화. **얼어붙은 prior 는 들머리 경사를 하산 내내 그대로 씀** — 멀리 가면 틀림(§3-E-1).
> 1D 예: $\sin\theta=0.5$ 풀 때 $J=\cos\theta$ 는 $\theta{=}0{\to}1.0$, $\theta{=}0.5{\to}0.88$ 로 매번 달라진다.

**코드에서:** `optimize()` 의 iteration 루프(`nfr_mapper.cpp:252`)가 매 바퀴 `linearizeHelper`(시각)와
`relPoseError`/`rollPitchError`(factor)를 **현재 `frame_poses` 로** 호출해 $r,J$ 를 새로 뽑는다 = factor 재선형화.
raw prior 의 $H,b$ 는 marginalize 때 한 번 만든 뒤 안 바뀌니 재선형화가 없다.

#### 최적화 파라미터는 무엇인가 (factor는 변수가 아니다)

**factor를 뽑는다고 새 최적화 변수가 생기지 않는다.** factor는 기존 pose를 잇는 **제약(비용 항)** 일 뿐,
타임스탬프로 pose를 참조한다(RollPitch→`t_ns`, RelPose→`t_i_ns,t_j_ns`). 실제로 움직이는 변수 $\mathcal{X}$ 는:

| 최적화 파라미터 | 값이 움직이나 |
|---|---|
| KF pose `frame_poses` (6-DoF) | ✅ 매 iter `applyInc` (`nfr_mapper.cpp:320-323`, `!isLinearized()`) |
| landmark `lmdb` (역깊이) | ✅ `updatePoints` |
| ~~velocity, bias~~ | — 애초에 없음 (`processMargData`가 주변화) |

**단, VIO와 비교하면 파라미터 집합 자체는 바뀐다:** VIO는 pose+velocity+bias+landmark 를 최적화하지만,
NfrMapper는 **velocity·bias가 빠지고(주변화 → pose 팩터 정보로 흡수), landmark는 이미지에서 재생성**,
결과적으로 **pose + (새) landmark** 만 최적화한다.

**factor의 "측정치"는 고정 상수:** 최적화 중 pose 값은 움직이고 factor는 매 iter 재선형화되지만(N**F**R의
핵심), factor가 든 측정치 `T_i_j`·`R_w_i_meas`·`cov_inv` 는 **추출 시점에 얼어붙은 상수**다. 잔차 $r$ =
"움직이는 pose가 이 고정 측정치에서 얼마나 벗어났나". 즉 값이 바뀌는 건 pose·landmark(변수)이지,
factor의 target이 아니다.

### 6-1. 상태 순서 구성 (`nfr_mapper.cpp:245-250`)

```cpp
AbsOrderMap aom;
for (const auto& kv : frame_poses) {                 // 모든 KF pose 를 변수로
  aom.abs_order_map[kv.first] = std::make_pair(aom.total_size, POSE_SIZE);
  aom.total_size += POSE_SIZE;                        // KF 당 6 DoF
}
```

### 6-2. 선형화 + 정보 행렬 누적 (`251-299`)

```cpp
// (1) 시각(reprojection) 항: landmark 를 상대좌표로 선형화
linearizeHelper(rld_vec, lmdb.getObservations(), rld_error);

// (2) 세 항을 하나의 accumulator 로 병렬 누적
MapperLinearizeAbsReduce<SparseHashAccumulator<double>> lopt(aom, &frame_poses);
tbb::parallel_reduce(range,  lopt);                   // 시각 항 (RelLinData)
if (config.mapper_use_factors) {
  tbb::parallel_reduce(range1, lopt);                 // RollPitch factors
  tbb::parallel_reduce(range2, lopt);                 // RelPose factors
}
```

각 factor가 $H, b$에 기여하는 방식은 `MapperLinearizeAbsReduce`의 operator들
(`nfr_mapper.h:109-155`). 예를 들어 RelPose factor:

```cpp
Sophus::Vector6d res = relPoseError(rpf.T_i_j, pose_i, pose_j, &Ji, &Jj);
accum.addH(idx_i, idx_i, Ji.transpose()*rpf.cov_inv*Ji);   // Hii += JiᵀΩJi
accum.addH(idx_i, idx_j, Ji.transpose()*rpf.cov_inv*Jj);   // Hij += JiᵀΩJj
accum.addH(idx_j, idx_i, Jj.transpose()*rpf.cov_inv*Ji);   // Hji
accum.addH(idx_j, idx_j, Jj.transpose()*rpf.cov_inv*Jj);   // Hjj
accum.addB(idx_i, Ji.transpose()*rpf.cov_inv*res);         // bi += JiᵀΩr
accum.addB(idx_j, Jj.transpose()*rpf.cov_inv*res);         // bj
rel_error += res.transpose()*rpf.cov_inv*res;              // χ²
```

이것은 §1의 $H = J^\top W J$, $b = J^\top W r$ 를 factor별로 조립하는 것. $W$ 자리에
§3-B에서 복원한 `cov_inv`($\Omega$)가 들어간다. RollPitch factor도 동일한 방식(2×6 야코비안).

### 6-3. Levenberg–Marquardt 풀이 (`304-407`)

```cpp
Eigen::VectorXd Hdiag_lambda = Hdiag * lambda;                // damping
for (i) Hdiag_lambda[i] = max(Hdiag_lambda[i], min_lambda);
const Eigen::VectorXd inc = lopt.accum.solve(&Hdiag_lambda);  // (H+λD)Δ = -b
```

$$
(H + \lambda\, \mathrm{diag}(H))\,\Delta x = -b
$$

증분 적용:

```cpp
for (auto& kv : frame_poses) {                       // pose 업데이트
  int idx = aom.abs_order_map.at(kv.first).first;
  kv.second.applyInc(-inc.segment<POSE_SIZE>(idx));  // T ← T ⊞ (−Δ)
}
tbb::parallel_for(..., updatePoints(aom, rld, inc, lmdb));   // landmark 업데이트
```

그 뒤 세 항의 새 오차를 다시 계산해 감소하면 채택(λ↓), 증가하면 기각·복원(λ↑):

```cpp
computeError(after_update_vision_error);             // 시각 항
computeRelPose(after_rel_error);                     // §nfr_mapper.cpp:430-441
computeRollPitch(after_roll_pitch_error);            // §443-453
double f_diff = error_total - after_error_total;
if (f_diff < 0) { lambda = min(max_lambda, lambda_vee*lambda); lambda_vee*=2; restore(); }
else            { lambda = max(min_lambda, lambda/3); lambda_vee=2; step=true; }
```

`config.mapper_use_lm == false`면 순수 Gauss-Newton(고정 damping)으로 한 번에 푼다(`382-407`).
`max_inc < 1e-5`면 수렴으로 보고 종료.

### 6-4. `computeRelPose` / `computeRollPitch` (오차만 계산)

`nfr_mapper.cpp:430-453` — 야코비안 없이 잔차만 평가해 χ² 누적. LM의 채택/기각 판정에 쓰인다:

$$
E_{\text{rel}} = \sum_{ij} r_{ij}^\top \Omega_{ij} r_{ij}, \qquad
E_{rp} = \sum_k r_{rp,k}^\top \Omega_{rp,k} r_{rp,k}
$$

---

## 7. 전체 데이터/수식 흐름 요약

```
VIO (sqrt_keypoint_vio.cpp)
 └ marginalize: linearizeProblem + performQR → Q2Jp, Q2r
    └ H = Q2Jpᵀ Q2Jp,  b = Q2Jpᵀ Q2r         ← 여기서 "prior 를 뽑음"
       └ MargData{abs_H, abs_b, aom, frame_poses/states, kfs_*, images}
          └ out_marg_queue → 디스크 (cereal)

NfrMapper (nfr_mapper.cpp)
 └ addMargData
    ├ processMargData        : vel/bias 소거 (Schur) → pose-only  H*, b*
    ├ extractNonlinearFactors:
    │    Σ = H*⁻¹
    │    RollPitch : Σ_f = J Σ Jᵀ,  Ω_rp = (Σ_f[4:5,4:5])⁻¹   (IMU 있을 때)
    │    RelPose   : T_ij = T_wi⁻¹ T_wj,  Σ_ij = J Σ Jᵀ, Ω_ij = Σ_ij⁻¹
    └ frame_poses 에 초기 pose 등록
 └ detect → match(stereo/all, BoW 루프클로저) → build_tracks → setup_opt(삼각측량)
 └ optimize (LM):
      E = Σ ρ(r_vis Σpx⁻¹ r_vis) + Σ r_ijᵀ Ω_ij r_ij + Σ r_rpᵀ Ω_rp r_rp
      (H+λD)Δ = −b,  반복
```

**한 줄 요약**: VIO는 오래된 키프레임을 소거하면서 정보 행렬 $H=J^\top J$, $b=J^\top r$ 를
`MargData`로 뽑아 전달하고, NfrMapper는 이를 $\Sigma=H^{-1}$ 로 뒤집어 **상대 pose·roll/pitch
비선형 factor**로 복원한 뒤, BoW로 새로 찾은 시각 관측과 함께 하나의 LM 최소제곱으로 전역 지도를
다시 최적화한다.
