# Orbital Bullseye — Implementation Notes (v1)

## Current State
As of Correction Updates 1–3:
- Namespace: `bullseye_pred`
- Canonical math/types: `core/types.hpp`
- Canonical constants: `core/constants.hpp`
- Canonical contracts: `core/contracts.hpp`

---

## Frame Providers
- CartesianChiefProvider
- TwoBodyChiefProvider

All providers enforce exact-time semantics.

---

## Adopted Frame Handling
- Adopted frames are validated via `bullseye_frame_validator`
- Validation is contract-driven (no hardcoded tolerances)
- On failure:
  - Constructed fallback is used
  - Output is marked degraded with reason code

---

## Tolerances
Tolerances are:
- Dimensionless where possible
- Scaled using abs-floor + relative components
- Valid across LEO → GEO and high-eccentricity orbits

---

## Known Gaps (Intentional)
- No unified BullseyeFrame object yet
- No predictor integration yet
- No TLE support (v2)
