---

## Sprint 0 — Project kickoff and contracts lock (M0)

**Goal:** Make the rules unambiguous and enforceable before code scales.

**Scope**

* Write `docs/contracts.md` + `core/contracts.hpp` covering:

  * GCR-1..6 (timing terms, frames, ω semantics, tolerances/thresholds, deterministic iterative math)
  * FR-14 fail-fast time policy (including FR-14a when adopted RIC frames are configured)
  * Adopted RIC frame compatibility contract (time-tag rules, centering tolerance, orthonormality/handedness checks, ω coordinate declaration)
  * DM-6 central body/μ contract
  * FR-15 time grid semantics
* Write `docs/determinism.md` (toolchain flags, FP settings, runtime invariants)
* Write `docs/performance_budget.md` (GCR-2 table + measurement method)

**Definition of Done**

* Contracts compile into code as constants/types.
* Threshold names and ownership are finalized (even if some numeric values are placeholders, they must exist and be single-source).
* Build profiles exist: `deterministic` vs `release` (fast-math explicitly off).

**Deliverables**

* Docs + headers + a “contract smoke test” unit test that prints/validates constants.

---

## Sprint 1 — Core data model, buffers, indexing, publication (M1)

**Goal:** Implement the data plane (prealloc + atomic publish + stable indexing) with tests.

**Scope**

* `core/types.hpp`, `core/prediction_buffer.*` (include DM-4 bullseye provenance fields: `bullseye_chief_source_id`, `bullseye_frame_source_id`, `bullseye_frame_mode`)
* `core/vehicle_index_map.*`
* `core/publisher.*` double buffer
* Time grid generator complying with FR-15 (includes sample at `t0`)
* Unit tests:

  * buffer layout/stride
  * `MAX_STEPS >= 61` for v1 bounds
  * seqno monotonicity
  * stable indexing across run + restart serialization format (even if restart invariants are deferred)

**Definition of Done**

* No heap allocations in steady-state paths (instrumented or verified with allocator hooks).
* Atomic publish is race-safe (even if single-threaded now).

**Deliverables**

* A small “dummy predictor” that fills buffers deterministically and publishes.

---

## Sprint 2 — Chief + RIC frame providers + inertial frame enforcement (Phase 2 partial)

**Goal:** Get chief state plumbing correct (including fail-fast time and frame-id checks), and introduce the adopted-RIC frame provider path (interface + validation harness).

**Scope**

* Provider interfaces:
  * `core/chief_state_provider.hpp`
  * `core/bullseye_frame_provider.hpp` (adopted RIC)
  * `core/bullseye_frame_validator.hpp` (adopted-frame validation per FR-1b)
* Implement chief providers:

  * `integration/provider_jeod.cpp` (stub/mocked if JEOD isn’t available in CI)
  * `core/provider_cartesian.cpp`
  * `core/provider_twobody.cpp` (pick one method and lock it)

* Implement adopted-RIC frame providers (stubs acceptable if JEOD isn’t available in CI):

  * `integration/frame_provider_jeod.cpp` (bind/alias to an existing JEOD RIC/LVLH-equivalent frame as declared)
  * `core/frame_provider_cartesian.cpp` (user-supplied time-tagged/current RIC frame)

* Add enforcement:

  * chief provider frame ID match to `INERTIAL_FRAME_ID`
  * inertialness verification hooks (JEOD integration frame check; external kinematic check if available)
  * FR-14a time-tag exactness for adopted frame providers (no silent “nearest time” behavior)
  * adopted-frame validation: time tag, centering vs chief, right-handed + orthonormal, ω coordinate declaration

* Tests:

  * deterministic provider outputs (chief + adopted-frame providers)
  * FR-14 fail-fast behavior (chief request `t0` not available → fail path)
  * FR-14a fail-fast behavior (frame request `t0` not available → fail path)
  * frame-id mismatch → fail path
  * adopted-frame validation pass/fail cases (time tag mismatch, centering mismatch, orthonormality failure)

**Definition of Done**

* Providers return deterministic `r,v` + `frame_id` (chief) and deterministic adopted-frame tuples (origin + orientation + optional ω_RIC + time tag), and fail deterministically on contract violations.

**Deliverables**

* Provider + frame-provider test harness with reproducible inputs and golden outputs.

---


## Sprint 3 — Bullseye provisioning (constructed + adopted) + transforms + deterministic degeneracy fallback (M2)

**Goal:** Make the Bullseye frame and Option B transforms correct and testable for both constructed and adopted frame modes.

**Scope**

* `core/bullseye_frame.*` (new or refactor):

  * supports `frame_mode = CONSTRUCTED | ADOPTED`
  * in ADOPTED mode: pulls frame from `core/bullseye_frame_provider.hpp` and validates via `core/bullseye_frame_validator.*`
  * exposes uniform getters (origin/orientation/ω_RIC if available) and provenance (`bullseye_chief_source_id`, `bullseye_frame_source_id`, `bullseye_frame_mode`)

* `core/bullseye_frame_math.*`:

  * DCM computation (constructed mode)
  * ω computation and **ω_RIC** output

* `core/frame_transforms.*` implementing FR-4a..c with ω_RIC (Option B), operating on the Bullseye frame produced (constructed or adopted)

* ω policy in adopted mode (v1):

  * if adopted frame provides ω_RIC with a coordinate declaration, use it
  * if not provided, compute deterministic baseline ω_RIC from chief

* Deterministic degeneracy fallback (D) with thresholds `r_min_m`, `h_min`

* Tests:

  * orthonormality + det(C) ≈ +1 within GCR-5
  * round-trip inertial ↔ RIC within GCR-5 (constructed and adopted)
  * finite-difference check validating Option B semantics
  * adopted-frame validation failure behavior (invalid/degraded vs fail-fast per policy)
  * transition trigger tests when switching `frame_mode` (constructed↔adopted) and/or `frame_source_id` (FR-9a)
  * degeneracy cases: no NaNs; degraded flag set

**Definition of Done**

* Bullseye provisioning is deterministic and the transform suite passes numerical regression tests in both constructed and adopted modes.

**Deliverables**

* A “frame demo” that prints basis vectors, ω_RIC, provenance (mode + ids), and round-trip errors.

---


## Sprint 4 — End-to-end predictor with HCW (M3)

**Goal:** First working system: scheduler-independent orchestrator + HCW predictions + atomic publish.

**Scope**

* `core/relative_predictor.*` wired end-to-end:

  * read truth vehicle states (stubbed interface if needed)
  * query chief provider
  * update bullseye (constructed or adopted mode)
  * compute vehicle relative state (Option B)
  * generate HCW trajectory on FR-15 time grid
  * fill metadata and publish atomically
* Quality flags:

  * `CONTROL_ACTIVE_UNMODELED` (hook only; FR-8b)
  * status codes for provider fail-fast paths
* Tests:

  * deterministic output: same inputs → bitwise identical buffers (FR-10A)
  * fail-fast provider case: no normal publish
  * basic sanity against known HCW cases (unit-level math checks)

**Definition of Done**

* “One command” run produces published predictions at 2 Hz with correct metadata (including DM-4 bullseye provenance fields).

**Deliverables**

* Minimal integration harness runnable in CI (no JEOD required).

---

## Sprint 5 — Model selector state machine + debounce + source transition modes (B + C)

**Goal:** Control-plane maturity: stable model switching and explicit source transitions.

**Scope**

* `models/model_selector.*` implementing FR-12a..d

  * thresholds: `e_hcw_enter/exit`, `rho_hcw_enter/exit`, optional `T_hcw_max`
  * hold-time `MODEL_HOLD_SEC`
  * cause codes
* Source transition implementation in predictor (FR-9):

  * triggers: changes in `bullseye_chief_source_id`, `bullseye_frame_source_id`, or `bullseye_frame_mode`
  * modes: `HARD_SWITCH`, `BLEND_ORIGIN`, `BLEND_ORIENTATION`, `BLEND_BOTH`
  * metrics: origin jump, attitude jump, optional ω jump
  * flagging: set `BULLSEYE_SOURCE_SWITCHED`; apply degraded policy for `TRANSITION_DEGRADED_TICKS` when bounds exceeded
* Tests:

  * hysteresis + hold-time prevents flapping
  * scripted source switch sets flags/metrics and degrades when bounds exceeded
  * blend convergence monotonicity test (for blend modes)

**Definition of Done**

* Switching behavior is deterministic, measurable, and produces stable outputs.

**Deliverables**

* A repeatable scenario script that triggers both model switching and source switching.

---

## Sprint 6 — YA/STM implementation with determinism hardening + JEOD/Trick integration (M4 + M5 partial)

**Goal:** Add YA model safely; start real integration.

**Scope**

* `models/model_ya_stm.*` with A requirements:

  * fixed-iteration Kepler solver (`KEPLER_ITERS`)
  * deterministic clamping for inverse trig inputs
  * deterministic tie-breaks
* Tests:

  * YA determinism test (bitwise, deterministic build)
  * YA cross-platform tolerance test harness wired to GCR-5 (even if run only on one platform in CI for now)
  * YA→HCW convergence regression for small eccentricity
* Integration start:

  * `integration/trick_jobs.cpp` skeleton with job ordering
  * `integration/bullseye_jeod_adapter.cpp` (constructed: register; adopted: bind/alias)
  * `integration/frame_provider_jeod.cpp` (adopted JEOD frame provider)

**Definition of Done**

* YA runs end-to-end behind selector and meets determinism rules in deterministic mode.

**Deliverables**

* “Model suite” binary that can run HCW-only, YA-only, or selector-driven.

---

## Sprint 7 — Integration hardening + perf/determinism harnesses + evidence package (M5/M6)

**Goal:** Produce acceptance evidence and ensure operational readiness.

**Scope**

* Truth comparison harness: RMS/P95/P99 tables
* Performance harness per GCR-2 (no heap)
* Determinism harness:

  * FR-10A bitwise on same machine
  * FR-10B tolerance checks using GCR-5
* JEOD/Trick integration completion + checkpoint/restart validation per existing IR-3/DM-5 (with **strict buffer/seqno invariants still deferred**)

**Definition of Done**

* A repeatable report artifact (logs + summary tables) that demonstrates compliance with key NFR/FR items.

---

## Notes on what’s explicitly deferred

* **Restart/publish strict invariants (E)**: you’ll still do restart validation, but you won’t lock the stronger “front/back buffer selection + seqno continuity” policy as a v1 acceptance gate.