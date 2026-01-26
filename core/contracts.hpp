#pragma once
/**
 * @file contracts.hpp
 * @brief Single-source constants and types enforcing v1 contracts.
 *
 * Mirrors docs/contracts.md. Numeric values may be placeholders in Sprint 0,
 * but names and ownership are locked.
 */

#include <cstdint>

namespace bullseye_pred {
namespace contracts {

// ---------------------------------
// 1) Timing Terms (GCR-1)
// ---------------------------------
namespace Timing {
inline constexpr double kPredictorNominalPeriodSec = 0.5;  // nominal; may be configured elsewhere
inline constexpr bool   kProvidersRequireExactT0   = true; // FR-14 / FR-14a
} // namespace Timing

// ---------------------------------
// 2) Inertial Frame Contract (GCR-3)
// ---------------------------------
namespace Frames {
inline constexpr const char* kInertialFrameId = "INERTIAL_FRAME_ID"; // configured string id
} // namespace Frames

// ---------------------------------
// 3) RIC Velocity Semantics (GCR-4)
// ---------------------------------
namespace Ric {

enum class OmegaConvention : std::uint8_t {
  // ω is expressed in RIC coordinates (required by Option B transform in v1).
  kOmegaExpressedInRIC = 0,
};

inline constexpr OmegaConvention kOmegaConvention = OmegaConvention::kOmegaExpressedInRIC;

} // namespace Ric

// ---------------------------------
// 4) Tolerances & Thresholds (GCR-5)
// ---------------------------------
namespace Tol {

// Vector abs+rel tolerance (||Δx|| <= abs + rel*||x||)
struct VecAbsRel {
  double abs = 0.0;
  double rel = 0.0;
};

// ---- Vector abs+rel tolerances ----
inline constexpr VecAbsRel kRoundTripPos_m       {1.0e-9,  1.0e-12}; // placeholder
inline constexpr VecAbsRel kRoundTripVel_mps     {1.0e-9,  1.0e-12}; // placeholder
inline constexpr VecAbsRel kAdoptedCentering_m   {1.0e-6,  1.0e-12}; // placeholder
inline constexpr VecAbsRel kCrossPlatformPos_m   {1.0e-6,  1.0e-9};  // placeholder
inline constexpr VecAbsRel kCrossPlatformVel_mps {1.0e-6,  1.0e-9};  // placeholder

// ---- Scalar tolerances / bounds ----
inline constexpr double kDcmOrthonormality = 1.0e-12; // placeholder
inline constexpr double kDeterminantOne    = 1.0e-12; // placeholder

// ---- Inertialness kinematic check tolerance (GCR-3c) ----
inline constexpr double kOmegaTolRadPerSec = 1.0e-12; // placeholder

// ---- Adopted-frame time tagging ----
inline constexpr double kAdoptedTimeTag_sec = 0.0; // 0.0 implies exact match required (placeholder semantics)

// ---- Source transition bounds ----
inline constexpr double kTransitionOriginJump_m     = 1.0;    // placeholder
inline constexpr double kTransitionAttitudeJump_rad = 1.0e-3; // placeholder
inline constexpr double kTransitionOmegaJump_radps  = 1.0e-3; // placeholder

// ---- Degeneracy thresholds ----
inline constexpr double kRmin_m = 1.0;    // placeholder
inline constexpr double kHmin   = 1.0e-6; // placeholder (units: m^2/s)

// ---- Model selector thresholds ----
inline constexpr double      kE_HcwEnter    = 1.0e-3;  // placeholder
inline constexpr double      kE_HcwExit     = 2.0e-3;  // placeholder
inline constexpr double      kRho_HcwEnter  = 1.0e-3;  // placeholder
inline constexpr double      kRho_HcwExit   = 2.0e-3;  // placeholder
inline constexpr double      kModelHold_sec = 5.0;     // placeholder
inline constexpr std::int32_t kTransitionDegradedTicks = 2; // placeholder

} // namespace Tol

// ---------------------------------
// 5) Deterministic Iterative Math (GCR-6)
// ---------------------------------
namespace Det {
inline constexpr std::int32_t kKeplerIters   = 16;   // placeholder fixed-iteration count
inline constexpr double       kTrigClampEps  = 1e-15; // placeholder clamp epsilon
} // namespace Det

// ---------------------------------
// 6) Provider Time Policy — Fail-Fast (FR-14 / FR-14a)
// ---------------------------------
namespace TimePolicy {

enum class OnTimeMismatch : std::uint8_t {
  kAbortTick = 0,
  kPublishInvalid = 1,
};

inline constexpr OnTimeMismatch kOnTimeMismatch = OnTimeMismatch::kAbortTick;

} // namespace TimePolicy

// ---------------------------------
// 7) Adopted RIC Compatibility (FR-1b)
// ---------------------------------
namespace Adopted {

enum class FrameKind : std::uint8_t {
  kUnknown = 0,
  kBullseyeRic = 1,
};

enum class AxisOrder : std::uint8_t {
  kRIC = 0, // R, I, C axes in that order.
};

inline constexpr bool      kRequireFrameKindDeclaration = true;
inline constexpr FrameKind kRequiredFrameKind           = FrameKind::kBullseyeRic;
inline constexpr AxisOrder kRequiredAxisOrder           = AxisOrder::kRIC;

// v1: ω is expected to be expressed in RIC coordinates if provided
inline constexpr Ric::OmegaConvention kRequiredOmegaConvention = Ric::kOmegaConvention;

} // namespace Adopted

// ---------------------------------
// 8) Central Body / μ Contract (DM-6)
// ---------------------------------
namespace Grav {

inline constexpr double kMuM3PerS2 = 3.986004418e14; // Earth placeholder
inline constexpr const char* kCentralBodyId = "EARTH"; // placeholder (JEOD uses body string)

} // namespace Grav

// ---------------------------------
// Sprint 0 contract compile-time sanity checks
// ---------------------------------
static_assert(Det::kKeplerIters > 0, "Deterministic Kepler iteration count must be > 0.");
static_assert(Tol::kModelHold_sec >= 0.0, "Model hold time must be non-negative.");
static_assert(Tol::kOmegaTolRadPerSec >= 0.0, "Omega tolerance must be non-negative.");
static_assert(static_cast<std::uint8_t>(Ric::kOmegaConvention) == 0,
              "Omega convention enum layout changed; update contracts and users accordingly.");

} // namespace contracts
} // namespace bullseye_pred
