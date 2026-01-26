# Contracts — Bullseye (RIC) Frame + Relative Predictor (v1)

Namespace: bullseye_pred::contracts

Purpose: This document is the single source of truth for runtime semantics and
enforcement rules for v1. All rules here are normative and must be enforced
via core/contracts.hpp.

Sprint 0 Policy:
- Numeric values may be placeholders.
- Names, ownership, units, and failure modes are locked.
- No other document or source may redefine these contracts.

----------------------------------------------------------------
1. Timing Terms (GCR-1)
----------------------------------------------------------------

1.1 Definitions

Simulation integrator step:
The base numerical integration step of the simulation engine.

Bullseye update step:
An update of the bullseye frame state (constructed or adopted) for a specific
effective time.

Predictor tick:
The periodic event at which relative predictions are computed, using truth
at time t0.

Publish:
The atomic publication of output buffers produced by a predictor tick.

1.2 Terminology rule

Unless explicitly stated otherwise, “update tick” refers to the predictor tick.

1.3 Timing enforcement rule

For a given predictor tick:
- the chief state,
- the bullseye origin, orientation, and omega,
- all transforms and predictions

shall correspond to the same effective time t0.

----------------------------------------------------------------
2. Inertial Frame Contract (GCR-3)
----------------------------------------------------------------

2.1 Configured inertial frame

All internal Cartesian states shall be expressed in a single configured
inertial frame:

INERTIAL_FRAME_ID

2.2 Same-frame enforcement

All source providers shall tag output states with a frame_id.
The system shall verify:

frame_id == INERTIAL_FRAME_ID

before use. If this check fails, the provider output shall not be used
for that predictor tick.

2.3 Inertialness verification

The system shall verify that INERTIAL_FRAME_ID is inertial.

JEOD simulations:
INERTIAL_FRAME_ID shall correspond to a JEOD integration frame. This is
verified at initialization.

Non-JEOD / external sources:
If frame kinematics are available, the frame shall be treated inertial iff:

||Omega|| <= Omega_tol

where:
- Omega is the angular velocity of INERTIAL_FRAME_ID relative to the
  simulation inertial root,
- Omega_tol is the authoritative tolerance kOmegaTolRadPerSec (see 4.3.2).

Failure handling is governed by the fail-fast policy (Section 6).

----------------------------------------------------------------
3. RIC Velocity Semantics — Option B (GCR-4)
----------------------------------------------------------------

3.1 Definition

The system defines relative velocity in the RIC frame as:

v_RIC = d/dt ( r_RIC )

where r_RIC are relative position coordinates expressed in the rotating
RIC frame.

3.2 Omega coordinate semantics (v1)

The bullseye frame shall provide angular velocity:

omega_RIC — angular velocity of the RIC frame expressed in RIC coordinates.

This ensures the Option B velocity transform is well-defined and unambiguous.

----------------------------------------------------------------
4. Authoritative Tolerances & Threshold Policy (GCR-5)
----------------------------------------------------------------

4.1 Single authoritative set

The system shall define exactly one authoritative set of numeric tolerances
and thresholds. All runtime logic, validation, and tests shall reference
these names.

4.2 Vector tolerance form (absolute + relative)

Vector tolerances shall use:

||Delta x|| <= x_abs + x_rel * ||x||

----------------------------------------------------------------
4.3 Tolerance Inventory (Authoritative)
----------------------------------------------------------------

4.3.1 Vector absolute + relative tolerances

kRoundTripPos_m
  Applies to: FR-4d round-trip position
  Units: meters

kRoundTripVel_mps
  Applies to: FR-4d round-trip velocity
  Units: meters per second

kAdoptedCentering_m
  Applies to: FR-1b.2 adopted-frame centering
  Units: meters

kCrossPlatformPos_m
  Applies to: FR-10B cross-platform position consistency
  Units: meters

kCrossPlatformVel_mps
  Applies to: FR-10B cross-platform velocity consistency
  Units: meters per second

----------------------------------------------------------------
4.3.2 Scalar tolerances / bounds
----------------------------------------------------------------

kDcmOrthonormality
  Applies to: basis orthonormality check (e.g. ||C^T C - I||)
  Units: dimensionless

kDeterminantOne
  Applies to: right-handedness (det(C) ~= +1)
  Units: dimensionless

kOmegaTolRadPerSec
  Applies to: inertialness kinematic check (GCR-3c)
  Units: radians per second

----------------------------------------------------------------
4.3.3 Adopted-frame time tagging
----------------------------------------------------------------

kAdoptedTimeTag_sec
  Applies to: FR-1b.1 adopted-frame time tag match
  Units: seconds
  Notes: exact match required in v1

----------------------------------------------------------------
4.3.4 Source transition bounds
----------------------------------------------------------------

kTransitionOriginJump_m
  Applies to: FR-9c origin continuity
  Units: meters

kTransitionAttitudeJump_rad
  Applies to: FR-9c attitude continuity
  Units: radians

kTransitionOmegaJump_radps
  Applies to: FR-9c omega continuity
  Units: radians per second

----------------------------------------------------------------
4.3.5 Degeneracy thresholds
----------------------------------------------------------------

kRmin_m
  Applies to: degeneracy handling
  Units: meters

kHmin
  Applies to: degeneracy handling
  Units: meters^2 per second

----------------------------------------------------------------
4.3.6 Model selector thresholds (names locked in Sprint 0)
----------------------------------------------------------------

kE_HcwEnter
kE_HcwExit
kRho_HcwEnter
kRho_HcwExit
kModelHold_sec
kTransitionDegradedTicks

----------------------------------------------------------------
5. Deterministic Iterative Math Policy (GCR-6)
----------------------------------------------------------------

5.1 Fixed iteration rule

In deterministic mode, iterative numerical methods in the runtime path
shall:
- use a fixed iteration count,
- avoid data-dependent early exit.

5.2 Deterministic clamping

Inputs to inverse trigonometric functions derived from floating arithmetic
shall be deterministically clamped to the valid domain prior to evaluation.

----------------------------------------------------------------
6. Provider Time Policy — Fail-Fast (FR-14 / FR-14a)
----------------------------------------------------------------

6.1 Chief provider (FR-14)

The chief provider shall supply state for exactly the requested time t0.
If it cannot, the predictor tick shall fail-fast and shall not publish
a normal prediction.

6.2 Adopted RIC frame provider (FR-14a)

When Adopted RIC mode is configured, the frame provider shall supply the
adopted frame for exactly t0 or fail-fast.

6.3 Fail-fast outcomes

Fail-fast behavior shall be one of:

- ABORT_TICK
- PUBLISH_INVALID

----------------------------------------------------------------
7. Adopted RIC Compatibility Contract (FR-1b)
----------------------------------------------------------------

At predictor tick time t0, an adopted RIC frame shall satisfy:

1. Time tag:
   Explicitly corresponds to t0.

2. Centering:
   Origin equals chief position at t0 within kAdoptedCentering_m.

3. Declaration and quality:
   - frame_kind = BULLSEYE_RIC
   - axis order {R, I, C}
   - right-handed and orthonormal within tolerances.

4. Omega declaration:
   If omega is provided, its coordinate expression shall be declared and
   compatible with omega_RIC.

Violation handling follows the fail-fast policy (Section 6), unless
explicitly configured otherwise.

----------------------------------------------------------------
8. Central Body / Mu Contract (DM-6)
----------------------------------------------------------------

8.1 JEOD simulations

- Central body identity is specified by a JEOD body string.
- A JEOD RelState defines the relationship between the central body and
  the chief.
- Mu shall be obtained from JEOD for that same body identity.

8.2 Non-JEOD simulations

- Mu is provided as a single configured constant.
- The system assumes the provided chief state is consistent with that mu
  and central body.

----------------------------------------------------------------
Sprint 0 Exit Condition
----------------------------------------------------------------

This document is Sprint 0 complete once mirrored exactly into
core/contracts.hpp and covered by a contract smoke test.
