// core/bullseye_frame.hpp
#pragma once

/**
 * @file bullseye_frame.hpp
 * @brief Bullseye frame product: constructed RIC from chief or adopted external RIC.
 *
 * Policy:
 * - If adopted is enabled and valid: use it.
 * - If adopted is enabled but invalid: follow contracts::Adopted::kOnAdoptedInvalid
 *   (v1: fallback to constructed and mark degraded).
 */

#include <cstdint>

#include "core/bullseye_frame_math.hpp"
#include "core/bullseye_frame_provider.hpp"
#include "core/bullseye_frame_validator.hpp"
#include "core/chief_state_provider.hpp"
#include "core/contracts.hpp"
#include "core/types.hpp"

namespace bullseye_pred
{

enum class BullseyeFrameMode : std::uint8_t
{
    kConstructedOnly = 0,
    kAdoptedPrefer = 1,
};

struct BullseyeFrameSnapshot final
{
    double time_tag{0.0};

    Vec3 origin_i{};
    Mat3 C_from_ric_to_inertial{Mat3::identity()};

    bool has_omega{false};
    Vec3 omega_ric{};
    OmegaCoords omega_coords{OmegaCoords::kUnspecified};

    FrameKind frame_kind{FrameKind::kBullseyeRIC};
    AxisOrder axis_order{AxisOrder::kRIC};

    const char* inertial_frame_id{nullptr};
    const char* adopted_frame_source_id{nullptr};

    bool used_adopted{false};
    contracts::Adopted::DegradeReason degraded{contracts::Adopted::DegradeReason::kNone};
    ProviderStatus status{};
};

class BullseyeFrame final
{
  public:
    BullseyeFrame(IChiefStateProvider& chief_provider, IBullseyeFrameProvider* adopted_provider,
                  BullseyeFrameMode mode = BullseyeFrameMode::kAdoptedPrefer,
                  FrameValidationTolerances tol = FrameValidationTolerances{}) noexcept
        : chief_(chief_provider), adopted_(adopted_provider), mode_(mode), tol_(tol)
    {
    }

    [[nodiscard]] BullseyeFrameSnapshot update(double t0) noexcept;

  private:
    IChiefStateProvider& chief_;
    IBullseyeFrameProvider* adopted_{nullptr};
    BullseyeFrameMode mode_{BullseyeFrameMode::kAdoptedPrefer};
    FrameValidationTolerances tol_{};
};

} // namespace bullseye_pred
