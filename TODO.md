# TODO

## pose rel cache (RelPoseLinCache) 재구현

feature/FEJ에서 시도했던 relative-pose 캐시 리팩터를 되돌리고(2026-07-21) 나중에
다시 구현하기로 함. 다시 만들 때 참고할 설계 요지:

- `ceres::EvaluationCallback`로 등록되는 `RelPoseLinCache`가 프레임 페어별
  상대 카메라 포즈 `T_t_h`(target(obs) <- host)를 evaluation마다 한 번만 계산해
  같은 페어의 모든 관측이 공유하도록 함 (basalt의 `RelPoseLin` 대응).
- 캐시 엔트리 `RelPoseLin`: 값 `T_t_h`는 현재 iterate를 추적, 야코비안
  `d_rel_d_h` / `d_rel_d_t`는 **FEJ 고정 선형화점**에서 평가.
- `BearingCost`는 `*_lin` optional 포즈를 직접 들고 있는 대신 `const RelPoseLin*`
  하나만 읽도록 단순화. manifold->ambient 변환용 `Jr(so3)` chart 인자는 현재
  iterate 기준이라 cost 쪽에 그대로 둠 (캐시에 넣지 않음).

현재 main의 FEJ는 리팩터 이전 방식(BearingCost가 `T_w_b_*_lin` optional 포즈로
`compute_rel_pose`를 직접 호출)으로 동작 중.
