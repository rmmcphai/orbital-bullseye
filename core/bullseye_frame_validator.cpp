// core/bullseye_frame_validator.cpp
#include "core/bullseye_frame_validator.hpp"

#include <cmath>

namespace bullseye_pred {
namespace {

inline bool finite_vec(const Vec3& v) noexcept {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool finite_mat(const Mat3& C) noexcept {
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (!std::isfinite(C(r, c))) return false;
    }
  }
  return true;
}

inline double max_abs_CCt_minus_I(const Mat3& C) noexcept {
  // Compute M = C*C^T - I and return max absolute element.
  const Mat3 Ct = transpose(C);
  const Mat3 CCt = mul(C, Ct);

  double max_abs = 0.0;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      const double ideal = (r == c) ? 1.0 : 0.0;
      const double e = CCt(r, c) - ideal;
      const double ae = std::fabs(e);
      if (ae > max_abs) max_abs = ae;
    }
  }
  return max_abs;
}

inline double center_bound(const Vec3& chief_r_i,
                           double abs_m,
                           double rel) noexcept {
  const double r = norm(chief_r_i);
  return abs_m + rel * r;
}

}  // namespace

FrameValidationResult validate_adopted_bullseye_ric_frame(
    double t0,
    const ChiefState& chief,
    const AdoptedRicFrame& frame,
    const FrameValidationTolerances& tol) noexcept {
  FrameValidationResult out{};

  // Basic status checks.
  if (!chief.status.ok()) {
    out.status.code = ProviderCode::kInvalidInput;
    out.reason = FrameValidationReason::kChiefNotOk;
    return out;
  }
  if (!frame.status.ok()) {
    out.status.code = ProviderCode::kInvalidInput;
    out.reason = FrameValidationReason::kFrameNotOk;
    return out;
  }

  // Finite checks up front to avoid NaN comparisons producing false negatives silently.
  if (!std::isfinite(t0) || !std::isfinite(chief.time_tag) || !std::isfinite(frame.time_tag) ||
      !finite_vec(chief.r_i) || !finite_vec(chief.v_i) ||
      !finite_vec(frame.origin_i) || !finite_mat(frame.C_from_ric_to_inertial)) {
    out.status.code = ProviderCode::kInvalidInput;
    out.reason = FrameValidationReason::kNonFinite;
    return out;
  }
  if (frame.has_omega) {
    if (!finite_vec(frame.omega_ric)) {
      out.status.code = ProviderCode::kInvalidInput;
      out.reason = FrameValidationReason::kNonFinite;
      return out;
    }
  }

  // Exact-time policy (FR-14a style for adopted frame).
  if (!(frame.time_tag == t0)) {
    out.status.code = ProviderCode::kTimeMissing;
    out.reason = FrameValidationReason::kTimeMismatch;
    return out;
  }

  // Declaration checks (FR-1b.3).
  if (frame.frame_kind != FrameKind::kBullseyeRIC || frame.axis_order != AxisOrder::kRIC) {
    out.status.code = ProviderCode::kInvalidInput;
    out.reason = FrameValidationReason::kBadDeclaration;
    return out;
  }

  // Centering check (FR-1b.2).
  const Vec3 d = frame.origin_i - chief.r_i;
  const double err = norm(d);
  const double bound = center_bound(chief.r_i, tol.center_abs_m, tol.center_rel);
  if (!(err <= bound)) {
    out.status.code = ProviderCode::kInvalidInput;
    out.reason = FrameValidationReason::kCenteringMismatch;
    return out;
  }

  // Orthonormality check (FR-1b.3 + NFR-5).
  const double ortho = max_abs_CCt_minus_I(frame.C_from_ric_to_inertial);
  if (!(ortho <= tol.ortho_max_abs)) {
    out.status.code = ProviderCode::kInvalidInput;
    out.reason = FrameValidationReason::kNotOrthonormal;
    return out;
  }

  // Right-handedness check via determinant close to +1.
  const double dC = det(frame.C_from_ric_to_inertial);
  if (!std::isfinite(dC) || !(std::fabs(dC - 1.0) <= tol.det_one_abs)) {
    out.status.code = ProviderCode::kInvalidInput;
    out.reason = FrameValidationReason::kNotRightHanded;
    return out;
  }

  // ω declaration check (FR-1b.4, GCR-4 usage).
  if (frame.has_omega) {
    if (frame.omega_coords != OmegaCoords::kOmegaRIC) {
      out.status.code = ProviderCode::kInvalidInput;
      out.reason = FrameValidationReason::kOmegaBadDeclaration;
      return out;
    }
  }

  out.status.code = ProviderCode::kOk;
  out.reason = FrameValidationReason::kOk;
  return out;
}

}  // namespace bullseye_pred
