# Big Buff Dual-Blade Stability Implementation Plan

**Goal:** Prevent the large-buff output from switching between two simultaneously lit blades while retaining the small-buff image tracker as the primary fallback.

**Architecture:** A lock-held target cannot enter the generic wide-gate jump confirmation path. The variable-speed branch requires a longer clean warm-up and can replace the shared small-buff prediction only when both predicted phases agree within a one-blade-safe angular limit.

## Task 1: Lock-ownership regression

- [ ] Add a failing unit test proving a held target lock does not permit generic jump confirmation.
- [ ] Add the minimal lock-state query and use it in the recognizer jump path.
- [ ] Run `buff_detection_stabilizer_test`.

## Task 2: Big-prediction output gate

- [ ] Add a failing unit test for phase-agreement acceptance and rejection.
- [ ] Add a small header-only angle agreement helper and register its test target.
- [ ] Require the gate before the large variable-speed result replaces the small tracker point.
- [ ] Increase large-model warm-up to a stable processing-time window.

## Task 3: Verification

- [ ] Run the focused native tests, Python source-contract test, and `git diff --check`.
- [ ] Build the `Fu` target when the local catkin build environment is available.
