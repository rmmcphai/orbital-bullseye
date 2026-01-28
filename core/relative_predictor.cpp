// core/relative_predictor.cpp

/**
 * @file relative_predictor.cpp
 * @brief HCW end-to-end predictor implementation.
 */

#include "core/relative_predictor.hpp"

#include <algorithm>
#include <cmath>

#include "core/bullseye_frame_math.hpp"
#include "core/frame_transforms.hpp"
#include "models/model_hcw.hpp"

namespace bullseye_pred
{

static inline bool finite3(const Vec3& v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

/**
 * @brief Compute mean motion n [rad/s] for HCW.
 *
 * Priority:
 * 1) Use frame omega if present and in RIC coordinates.
 * 2) Fallback to n = |r×v| / |r|^2 from chief state.
 */
static inline bool compute_mean_motion(const ChiefState& chief,
                                       const BullseyeFrameSnapshot& frame,
                                       double& out_n) noexcept
{
    if (frame.has_omega && frame.omega_coords == OmegaCoords::kOmegaRIC && std::isfinite(frame.omega_ric.z) &&
        frame.omega_ric.z > 0.0)
    {
        out_n = frame.omega_ric.z;
        return true;
    }

    if (!finite3(chief.r_i) || !finite3(chief.v_i))
    {
        return false;
    }
    const double r_norm = norm(chief.r_i);
    if (!(r_norm > 0.0) || !std::isfinite(r_norm))
    {
        return false;
    }
    const Vec3 h = cross(chief.r_i, chief.v_i);
    const double h_norm = norm(h);
    const double n = h_norm / (r_norm * r_norm);
    if (!(n > 0.0) || !std::isfinite(n))
    {
        return false;
    }
    out_n = n;
    return true;
}

void RelativePredictor::step(double t0, double horizon_sec, double cadence_sec) noexcept
{
    const auto grid = make_time_grid(horizon_sec, cadence_sec);
    if (grid.tau.empty())
    {
        return; // fail-fast: no publish
    }

    // Query chief (exact-time semantics enforced by provider).
    const ChiefState chief = chief_.get(t0);
    if (!chief.status.ok() || chief.frame_id == nullptr)
    {
        return; // fail-fast: no publish
    }

    // Update bullseye frame snapshot at t0.
    const BullseyeFrameSnapshot frame = bullseye_.update(t0);
    if (!frame.status.ok())
    {
        return; // fail-fast: no publish
    }

    double n_radps = 0.0;
    if (!compute_mean_motion(chief, frame, n_radps))
    {
        return; // fail-fast: no publish
    }

    // Build transform pieces.
    const Mat3 C_r2i = frame.C_from_ric_to_inertial;
    const Mat3 C_i2r = transpose(C_r2i);

    // Prepare HCW.
    ModelHCW model;
    HcwParams params{};
    params.n_radps = n_radps;

    // Write output.
    auto& buf = pub_.begin_write();

    const std::size_t steps = std::min<std::size_t>(grid.tau.size(), MAX_STEPS);
    const std::size_t nveh = std::min<std::size_t>(map_.size(), MAX_VEHICLES);

    for (std::size_t i = 0; i < nveh; ++i)
    {
        const auto vid = map_.id_at(i);
        if (!vid.has_value())
            continue;
        const VehicleState dep = veh_.get(*vid,t0);
        if (!dep.status.ok() || dep.frame_id == nullptr)
        {
            continue; // skip vehicle; still can publish others deterministically
        }

        // Require same inertial frame id as chief for v1.
        // (If you later support cross-frame inputs, this becomes a conversion hook.)
        // String compare is avoided here; you can enforce equality by pointer identity
        // if your system guarantees canonical frame-id pointers.
        if (dep.frame_id != chief.frame_id)
        {
            continue;
        }

        // Initial relative state in RIC, Option-B.
        const RelState rel = inertial_to_ric_relative(dep.r_i, dep.v_i, chief.r_i, chief.v_i, C_i2r, frame.omega_ric);

        RelStateRic x0{};
        x0.r_ric = rel.r;
        x0.v_ric = rel.v;

        // Predict positions only into buf.positions[i][k].
        // PredictionBuffer has fixed storage; use a Span over the first `steps` elements.
        auto result = model.predict_hcw(x0, params, grid, Span<Vec3>{buf.positions[i].data(), steps}, Span<Vec3>{nullptr, 0});
        if (result.code != ModelCode::kOk)
        {
            // Leave this vehicle row as-is (deterministic skip on failure).
            continue;
        }
    }

    // Publish snapshot (sets seqno and t0).
    pub_.publish(t0);
}

} // namespace bullseye_pred
