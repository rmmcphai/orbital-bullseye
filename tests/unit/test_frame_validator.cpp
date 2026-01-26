// tests/unit/test_frame_validator.cpp

#include <catch2/catch_test_macros.hpp>

#include "core/bullseye_frame_validator.hpp"

using bullseye_pred::AdoptedRicFrame;
using bullseye_pred::AxisOrder;
using bullseye_pred::ChiefState;
using bullseye_pred::FrameKind;
using bullseye_pred::FrameValidationReason;
using bullseye_pred::FrameValidationTolerances;
using bullseye_pred::Mat3;
using bullseye_pred::OmegaCoords;
using bullseye_pred::ProviderCode;
using bullseye_pred::ProviderStatus;
using bullseye_pred::Vec3;
using bullseye_pred::validate_adopted_bullseye_ric_frame;

static ChiefState make_ok_chief(double t0, const Vec3& r_i) {
  ChiefState c{};
  c.time_tag = t0;
  c.r_i = r_i;
  c.v_i = Vec3{0.0, 0.0, 0.0};
  c.frame_id = "INERTIAL";
  c.status.code = ProviderCode::kOk;
  return c;
}

static AdoptedRicFrame make_ok_frame(double t0, const Vec3& origin_i) {
  AdoptedRicFrame f{};
  f.time_tag = t0;
  f.origin_i = origin_i;
  f.C_from_ric_to_inertial = Mat3::identity();
  f.has_omega = false;
  f.omega_coords = OmegaCoords::kUnspecified;
  f.frame_kind = FrameKind::kBullseyeRIC;
  f.axis_order = AxisOrder::kRIC;
  f.frame_source_id = "USER_FRAME";
  f.status.code = ProviderCode::kOk;
  return f;
}

TEST_CASE("Adopted frame validator: pass case", "[validator]") {
  const double t0 = 100.0;
  const Vec3 rc{7000e3, 0.0, 0.0};

  const ChiefState chief = make_ok_chief(t0, rc);
  AdoptedRicFrame frame = make_ok_frame(t0, rc);

  FrameValidationTolerances tol{};
  tol.center_abs_m = 1e-9;
  tol.center_rel = 0.0;
  tol.ortho_max_abs = 1e-15;
  tol.det_one_abs = 1e-15;

  const auto res = validate_adopted_bullseye_ric_frame(t0, chief, frame, tol);
  REQUIRE(res.status.code == ProviderCode::kOk);
  REQUIRE(res.reason == FrameValidationReason::kOk);
}

TEST_CASE("Adopted frame validator: time mismatch -> kTimeMissing", "[validator]") {
  const double t0 = 100.0;
  const Vec3 rc{1.0, 2.0, 3.0};

  const ChiefState chief = make_ok_chief(t0, rc);
  AdoptedRicFrame frame = make_ok_frame(/*time_tag=*/101.0, rc);

  const auto res = validate_adopted_bullseye_ric_frame(t0, chief, frame, FrameValidationTolerances{});
  REQUIRE(res.status.code == ProviderCode::kTimeMissing);
  REQUIRE(res.reason == FrameValidationReason::kTimeMismatch);
}

TEST_CASE("Adopted frame validator: bad declaration rejected", "[validator]") {
  const double t0 = 100.0;
  const Vec3 rc{1.0, 2.0, 3.0};

  const ChiefState chief = make_ok_chief(t0, rc);
  AdoptedRicFrame frame = make_ok_frame(t0, rc);
  frame.axis_order = AxisOrder::kUnspecified;

  const auto res = validate_adopted_bullseye_ric_frame(t0, chief, frame, FrameValidationTolerances{});
  REQUIRE(res.status.code == ProviderCode::kInvalidInput);
  REQUIRE(res.reason == FrameValidationReason::kBadDeclaration);
}

TEST_CASE("Adopted frame validator: centering mismatch rejected", "[validator]") {
  const double t0 = 100.0;
  const Vec3 rc{1000.0, 0.0, 0.0};

  const ChiefState chief = make_ok_chief(t0, rc);
  AdoptedRicFrame frame = make_ok_frame(t0, Vec3{1000.1, 0.0, 0.0});  // 0.1 m offset

  FrameValidationTolerances tol{};
  tol.center_abs_m = 1e-3;  // 1 mm
  tol.center_rel = 0.0;

  const auto res = validate_adopted_bullseye_ric_frame(t0, chief, frame, tol);
  REQUIRE(res.status.code == ProviderCode::kInvalidInput);
  REQUIRE(res.reason == FrameValidationReason::kCenteringMismatch);
}

TEST_CASE("Adopted frame validator: orthonormality violation rejected", "[validator]") {
  const double t0 = 100.0;
  const Vec3 rc{1.0, 0.0, 0.0};

  const ChiefState chief = make_ok_chief(t0, rc);
  AdoptedRicFrame frame = make_ok_frame(t0, rc);

  // Perturb one element so CCt deviates from I.
  frame.C_from_ric_to_inertial(0, 0) = 1.0;
  frame.C_from_ric_to_inertial(0, 1) = 1e-6;

  FrameValidationTolerances tol{};
  tol.ortho_max_abs = 1e-12;  // too strict for 1e-6 perturbation

  const auto res = validate_adopted_bullseye_ric_frame(t0, chief, frame, tol);
  REQUIRE(res.status.code == ProviderCode::kInvalidInput);
  REQUIRE(res.reason == FrameValidationReason::kNotOrthonormal);
}

TEST_CASE("Adopted frame validator: left-handed / det!=+1 rejected", "[validator]") {
  const double t0 = 100.0;
  const Vec3 rc{1.0, 0.0, 0.0};

  const ChiefState chief = make_ok_chief(t0, rc);
  AdoptedRicFrame frame = make_ok_frame(t0, rc);

  // Flip one axis => det = -1
  frame.C_from_ric_to_inertial(2, 2) = -1.0;

  FrameValidationTolerances tol{};
  tol.det_one_abs = 1e-12;

  const auto res = validate_adopted_bullseye_ric_frame(t0, chief, frame, tol);
  REQUIRE(res.status.code == ProviderCode::kInvalidInput);
  REQUIRE(res.reason == FrameValidationReason::kNotRightHanded);
}

TEST_CASE("Adopted frame validator: omega present but wrong coords rejected", "[validator]") {
  const double t0 = 100.0;
  const Vec3 rc{1.0, 0.0, 0.0};

  const ChiefState chief = make_ok_chief(t0, rc);
  AdoptedRicFrame frame = make_ok_frame(t0, rc);
  frame.has_omega = true;
  frame.omega_ric = Vec3{0.1, 0.2, 0.3};
  frame.omega_coords = OmegaCoords::kOmegaInertial;  // wrong

  const auto res = validate_adopted_bullseye_ric_frame(t0, chief, frame, FrameValidationTolerances{});
  REQUIRE(res.status.code == ProviderCode::kInvalidInput);
  REQUIRE(res.reason == FrameValidationReason::kOmegaBadDeclaration);
}
