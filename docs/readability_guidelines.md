# Readability Guidelines (refactor/readability)

Shared rubric for the conservative, behavior-preserving readability pass over
`cpp/`. Both the Claude editors and the codex reviewer judge every batch against
this document. If a change is not justified by a rule here, it does not belong
in this pass.

## Prime directive: behavior is frozen

This is a numerical SLAM/VIO codebase (Ceres optimization, marginalization,
FEJ). Bit-for-bit runtime behavior must not change. Therefore **do not**:

- Reorder floating-point accumulations, matrix operations, or map/set iteration
  that feeds numerical results.
- Change evaluation order of Ceres parameter/residual block registration.
- Change container types where iteration order is observable (e.g. `std::map`
  vs `std::unordered_map` used to build a Hessian).
- Alter arithmetic, thresholds, tolerances, or default values.
- Change public/ABI-visible signatures unless the batch is explicitly a
  signature-cleanup batch and all call sites move with it.
- "Fix" a suspected bug. If you find one, record it in the batch notes for the
  human — do not silently change behavior under the banner of readability.

Every batch must keep the build green and all gtests passing. A batch that
cannot be shown behavior-preserving is rejected.

## Allowed changes (this pass)

1. **Naming** — rename locals/params/helpers for clarity; match surrounding
   convention (`snake_case` locals, `k`-prefixed constants, trailing-underscore
   members as already used in the codebase). No cross-file public renames in the
   conservative pass.
2. **Comments** — add a comment only where intent is non-obvious (the *why*, not
   the *what*). Remove stale/misleading comments. Keep density consistent with
   the file.
3. **Dead / commented-out code** — do **not** delete outright. Per the agreed
   policy, move genuinely useful debug output behind a proper logging path: a
   compile-time-guarded debug helper (e.g. a `debug_log_*` free function or a
   `VLOG`/`LogD`-style guarded call) so it can be re-enabled without hand-
   editing. Truly obsolete scaffolding with no debug value is removed, and the
   removal is called out in batch notes (git history retains it).
4. **Magic numbers** — promote unexplained literals to named `constexpr`
   constants with the same value. Do not touch values already sourced from
   config structs.
5. **Small helper extraction** — extract a well-named local lambda or `static`
   helper when it removes duplication or names a step, *without* changing the
   sequence of operations. Prefer file-local (`namespace {}`) helpers.
6. **Local structure** — early-return flattening, scoping a variable tighter,
   splitting a dense expression across named intermediates — only when it does
   not change evaluation order or results.

## Explicitly out of scope (conservative pass)

- Splitting files, moving helpers into headers, changing interfaces.
- Splitting long functions across translation units.
- Performance changes, algorithm changes, container-type swaps.
- clang-format churn beyond the lines you already touch.

## Per-batch checklist (editor + codex reviewer)

- [ ] Build green (`cmake --build build/readability_baseline`).
- [ ] `optimizer_tests` and `core_tests` pass.
- [ ] Diff contains only rule-justified changes; no arithmetic/order changes.
- [ ] No dead code deleted that had debug value (moved behind logger instead).
- [ ] Batch notes list any suspected bugs found (not fixed).
- [ ] Reviewer confirms behavior-preservation, not just style.

## Batch granularity

One module (or one large file) per batch, ordered low-risk → high-risk:
`utils → config → camera_model → database → feature_tracking → device →
optimizer → odometry`. Each batch is its own commit and its own codex review.
