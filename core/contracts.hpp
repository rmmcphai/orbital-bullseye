#pragma once
/**
 * @file contracts.hpp
 * @brief Single-source constants and types enforcing v1 contracts.
 *
 * Mirrors docs/contracts.md. Numeric values are v1 defaults; ownership and names are locked.
 *
 * Notes (Correction Update 3):
 * - Tolerances are valid across LEO→GEO and high-e orbits by using abs floors + relative scaling.
 * - Degeneracy thresholds use dimensionless h_hat = |r×v|/(|r||v|) where possible.
 * - Adopted-frame declaration requirements use canonical enums from core/types.hpp
 */

#include <cstdint>
#include <cstddef>

#include "core/types.hpp"

namespace bullseye_pred
{
namespace contracts
{

// ---------------------------------
// 1) Timing Terms (GCR-1)
// ---------------------------------
namespace Timing
{
inline constexpr double kPredictorNominalPeriodSec = 0.5; // nominal; may be configured elsewhere
inline constexpr bool kProvidersRequireExactT0 = true;    // FR-14 / FR-14a
} // namespace Timing

// ---------------------------------
// 2) Inertial Frame Contract (GCR-3)
// ---------------------------------
namespace Frames
{
inline constexpr const char* kInertialFrameId = "INERTIAL_FRAME_ID"; // configured string id
} // namespace Frames

// ---------------------------------
// 3) RIC Velocity Semantics (GCR-4)
// ---------------------------------
namespace Ric
{

enum class OmegaConvention : std::uint8_t
{
    // ω is expressed in RIC coordinates (required by Option B transform in v1).
    kOmegaExpressedInRIC = 0,
};

inline constexpr OmegaConvention kOmegaConvention = OmegaConvention::kOmegaExpressedInRIC;

} // namespace Ric

// ---------------------------------
// 4) Tolerances & Thresholds (GCR-5)
// ---------------------------------
namespace Tol
{

// Vector abs+rel tolerance (||Δx|| <= abs + rel*||x||)
struct VecAbsRel
{
    double abs = 0.0;
    double rel = 0.0;
};

// ---- Dimensionless tolerances / bounds (geometry) ----
inline constexpr double kDcmOrthonormality = 1.0e-12;
inline constexpr double kDeterminantOne = 1.0e-12;

// ---- Scaled abs+rel tolerances (LEO→GEO + high-e safe) ----
// Floors prevent false failures due to plumbing / numeric noise.
// Rel terms keep scaling sane when |x| grows.
inline constexpr VecAbsRel kRoundTripPos_m{1.0e-3, 1.0e-12};   // 1 mm floor
inline constexpr VecAbsRel kRoundTripVel_mps{1.0e-6, 1.0e-12}; // 1 µm/s floor

// Adopted-frame centering check (origin_i vs chief.r_i); scale with |r|
inline constexpr VecAbsRel kAdoptedCentering_m{1.0e-3, 1.0e-12}; // 1 mm floor

// Cross-platform comparisons: leave slightly looser by default.
// (If you later enforce bit-for-bit determinism, tighten explicitly.)
inline constexpr VecAbsRel kCrossPlatformPos_m{1.0e-3, 1.0e-11};   // default
inline constexpr VecAbsRel kCrossPlatformVel_mps{1.0e-6, 1.0e-11}; // default

// ---- Helper: compute an abs tolerance scaled by a reference norm ----
inline constexpr double scaled_abs(double abs_floor, double rel, double ref_norm) noexcept
{
    const double scaled = ref_norm * rel;
    return (scaled > abs_floor) ? scaled : abs_floor;
}

// ---- Inertialness kinematic check tolerance (GCR-3c) ----
inline constexpr double kOmegaTolRadPerSec = 1.0e-12; // may tune later

// ---- Adopted-frame time tagging ----
// 0.0 implies exact match required (FR-14a). Non-zero allows |Δt| <= tol.
inline constexpr double kAdoptedTimeTag_sec = 0.0;

// ---- Source transition bounds ----
inline constexpr double kTransitionOriginJump_m = 1.0;        // placeholder
inline constexpr double kTransitionAttitudeJump_rad = 1.0e-3; // placeholder
inline constexpr double kTransitionOmegaJump_radps = 1.0e-3;  // placeholder

// ---- Degeneracy thresholds ----
// Chief state sanity floors (avoid divide-by-zero, nonsense)
inline constexpr double kRmin_m = 1.0;
inline constexpr double kVmin_mps = 1.0e-6;

// Dimensionless: h_hat = |r×v| / (|r||v|) = sin(theta). If too small, RIC is ill-defined.
inline constexpr double kHhatMin = 1.0e-10;

// Back-compat placeholder (older code may reference kHmin as |r×v| threshold).
// Prefer kHhatMin for orbit-regime invariance.
inline constexpr double kHmin = 1.0e-6; // legacy placeholder (units: m^2/s)

// ---- Model selector thresholds ----
inline constexpr double kE_HcwEnter = 1.0e-3;               // placeholder
inline constexpr double kE_HcwExit = 2.0e-3;                // placeholder
inline constexpr double kRho_HcwEnter = 1.0e-3;             // placeholder
inline constexpr double kRho_HcwExit = 2.0e-3;              // placeholder
inline constexpr double kModelHold_sec = 5.0;               // placeholder
inline constexpr std::int32_t kTransitionDegradedTicks = 2; // placeholder

} // namespace Tol

// ---------------------------------
// 5) Deterministic Iterative Math (GCR-6)
// ---------------------------------
namespace Det
{
inline constexpr std::int32_t kKeplerIters = 16; // placeholder fixed-iteration count
inline constexpr double kTrigClampEps = 1e-15;   // placeholder clamp epsilon
} // namespace Det

// ---------------------------------
// 6) Provider Time Policy — Fail-Fast (FR-14 / FR-14a)
// ---------------------------------
namespace TimePolicy
{

enum class OnTimeMismatch : std::uint8_t
{
    kAbortTick = 0,
    kPublishInvalid = 1,
};

inline constexpr OnTimeMismatch kOnTimeMismatch = OnTimeMismatch::kAbortTick;

enum class OnProviderFailure : std::uint8_t
{
    kAbortTick = 0,
    kPublishInvalid = 1,
};

inline constexpr OnProviderFailure kOnProviderFailure = OnProviderFailure::kAbortTick;

} // namespace TimePolicy

// ---------------------------------
// 7) Adopted RIC Compatibility (FR-1b) + Degraded Policy (Correction Update 3)
// ---------------------------------
namespace Adopted
{
// Require canonical declarations on adopted frames.
inline constexpr bool kRequireFrameKindDeclaration = true;
inline constexpr bullseye_pred::FrameKind kRequiredFrameKind = bullseye_pred::FrameKind::kBullseyeRIC;
inline constexpr bullseye_pred::AxisOrder kRequiredAxisOrder = bullseye_pred::AxisOrder::kRIC;

// v1: ω is expected to be expressed in RIC coordinates if provided
inline constexpr Ric::OmegaConvention kRequiredOmegaConvention = Ric::kOmegaConvention;

enum class OnAdoptedInvalid : std::uint8_t
{
    kAbortTick = 0,
    kFallbackConstructedDegraded = 1,
};

inline constexpr OnAdoptedInvalid kOnAdoptedInvalid = OnAdoptedInvalid::kFallbackConstructedDegraded;

enum class DegradeReason : std::uint32_t
{
    kNone = 0u,
    kAdoptedInvalid = 1u << 0,
    kDegenerateChief = 1u << 1,
    kProviderJitter = 1u << 2,
};

inline constexpr DegradeReason operator|(DegradeReason a, DegradeReason b) noexcept
{
    return static_cast<DegradeReason>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
}
inline constexpr DegradeReason& operator|=(DegradeReason& a, DegradeReason b) noexcept
{
    a = a | b;
    return a;
}
inline constexpr bool any(DegradeReason r) noexcept
{
    return static_cast<std::uint32_t>(r) != 0u;
}

} // namespace Adopted

// ---------------------------------
// 8) Central Body / μ Contract (DM-6)
// ---------------------------------
namespace Grav
{
inline constexpr double kMuM3PerS2 = 3.986004418e14;   // Earth placeholder
inline constexpr const char* kCentralBodyId = "EARTH"; // placeholder (JEOD uses body string)
} // namespace Grav

// ---------------------------------
// Sprint 0/Correction contract compile-time sanity checks
// ---------------------------------
static_assert(Det::kKeplerIters > 0, "Deterministic Kepler iteration count must be > 0.");
static_assert(Tol::kModelHold_sec >= 0.0, "Model hold time must be non-negative.");
static_assert(Tol::kOmegaTolRadPerSec >= 0.0, "Omega tolerance must be non-negative.");
static_assert(static_cast<std::uint8_t>(Ric::kOmegaConvention) == 0,
              "Omega convention enum layout changed; update contracts and users accordingly.");

} // namespace contracts
} // namespace bullseye_pred
