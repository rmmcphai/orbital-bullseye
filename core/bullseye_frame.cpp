// core/bullseye_frame.cpp

#include "core/bullseye_frame.hpp"

namespace bullseye_pred
{

static BullseyeFrameSnapshot from_constructed(const ChiefState& chief,
                                              contracts::Adopted::DegradeReason degraded) noexcept
{
    BullseyeFrameSnapshot out;
    const ConstructedRicFrame c = construct_ric_from_chief(chief);

    out.time_tag = c.time_tag;
    out.origin_i = c.origin_i;
    out.C_from_ric_to_inertial = c.C_from_ric_to_inertial;
    out.has_omega = c.has_omega;
    out.omega_ric = c.omega_ric;
    out.omega_coords = c.omega_coords;
    out.frame_kind = c.frame_kind;
    out.axis_order = c.axis_order;
    out.inertial_frame_id = chief.frame_id;
    out.used_adopted = false;
    out.degraded = degraded;
    out.status = c.status;
    return out;
}

static BullseyeFrameSnapshot from_adopted(const ChiefState& chief, const AdoptedRicFrame& f) noexcept
{
    BullseyeFrameSnapshot out;
    out.time_tag = f.time_tag;
    out.origin_i = f.origin_i;
    out.C_from_ric_to_inertial = f.C_from_ric_to_inertial;
    out.has_omega = f.has_omega;
    out.omega_ric = f.omega_ric;
    out.omega_coords = f.omega_coords;
    out.frame_kind = f.frame_kind;
    out.axis_order = f.axis_order;
    out.inertial_frame_id = chief.frame_id;
    out.adopted_frame_source_id = f.frame_source_id;
    out.used_adopted = true;
    out.degraded = contracts::Adopted::DegradeReason::kNone;
    out.status.code = ProviderCode::kOk;
    return out;
}

BullseyeFrameSnapshot BullseyeFrame::update(double t0) noexcept
{
    BullseyeFrameSnapshot out;

    const ChiefState chief = chief_.get(t0);
    if (!chief.status.ok() || chief.frame_id == nullptr)
    {
        out.status.code = chief.status.ok() ? ProviderCode::kInvalidInput : chief.status.code;
        return out;
    }

    if (mode_ == BullseyeFrameMode::kAdoptedPrefer && adopted_ != nullptr)
    {
        const AdoptedRicFrame frame = adopted_->get(t0);

        const FrameValidationResult v =
            validate_adopted_bullseye_ric_frame(t0, chief, frame, tol_);

        if (v.status.ok())
        {
            return from_adopted(chief, frame);
        }

        if (contracts::Adopted::kOnAdoptedInvalid ==
            contracts::Adopted::OnAdoptedInvalid::kAbortTick)
        {
            out.status = v.status;
            return out;
        }

        contracts::Adopted::DegradeReason d = contracts::Adopted::DegradeReason::kAdoptedInvalid;
        BullseyeFrameSnapshot constructed = from_constructed(chief, d);
        if (!constructed.status.ok())
        {
            constructed.degraded |= contracts::Adopted::DegradeReason::kDegenerateChief;
        }
        return constructed;
    }

    BullseyeFrameSnapshot constructed =
        from_constructed(chief, contracts::Adopted::DegradeReason::kNone);
    if (!constructed.status.ok())
    {
        constructed.degraded |= contracts::Adopted::DegradeReason::kDegenerateChief;
    }
    return constructed;
}

} // namespace bullseye_pred
