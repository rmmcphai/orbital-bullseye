# Orbital Bullseye — v1 Requirements

## Scope (v1)
This document defines the **v1 contractual requirements** for the Orbital Bullseye reference frame system.
It reflects the post-Correction state (Updates 1–3 complete).

Out of scope for v1:
- TLE / SGP4 sources
- Non-RIC frames
- Non-deterministic propagation
- Multi-body gravity

---

## FR-1: Bullseye Frame Definition
- The Bullseye frame SHALL be a continuously defined RIC frame centered on a chief object.
- The frame SHALL be constructible from a chief inertial Cartesian state.
- The frame MAY be adopted from an external provider if it satisfies validation rules.

## FR-1b: Adopted Frame Validation
An adopted RIC frame SHALL:
1. Declare frame kind = `BullseyeRIC`
2. Declare axis order = `RIC`
3. Be centered on the chief within scaled tolerance
4. Declare ω in RIC coordinates if provided
5. Match the requested time tag (exact match in v1)

If validation fails:
- The system SHALL fallback to a constructed frame when possible
- The output SHALL be marked degraded

---

## FR-2: Chief Sources (v1)
v1 SHALL support the following chief sources:
- User-provided Cartesian chief state (exact time)
- Internal two-body propagator

---

## FR-14 / FR-14a: Time Policy
- All providers SHALL deliver state at exactly `t0`
- Time mismatch SHALL fail fast (abort tick)

---

## FR-X: Determinism
- All math SHALL be deterministic given identical inputs
- Fixed iteration counts SHALL be used where applicable

---

## Non-Functional Requirements
- C++17
- No dynamic allocation in inner loops
- No hidden global state
