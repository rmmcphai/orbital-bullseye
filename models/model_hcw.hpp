// models/model_hcw.hpp
#pragma once

#include "models/relative_model.hpp"

namespace bullseye_pred
{

class ModelHCW final : public IRelativeModel
{
  public:
    [[nodiscard]] Result predict_hcw(const RelStateRic& x0_ric,
                                    const HcwParams& params,
                                    const TimeGrid& grid,
                                    Span<Vec3> out_r_ric,
                                    Span<Vec3> out_v_ric) const noexcept override;
};

} // namespace bullseye_pred
