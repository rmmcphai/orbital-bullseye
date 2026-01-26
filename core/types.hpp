// core/types.hpp
#pragma once

/**
 * @file types.hpp
 * @brief Core math/data types used by providers, validators, and publication.
 *
 * ## DCM naming convention (REQUIRED)
 * We use the semantic name:
 *
 *   C_from_a_to_b
 *
 * meaning it maps coordinates expressed in frame `a` into coordinates expressed in frame `b`:
 *
 *   x_b = C_from_a_to_b * x_a
 *
 * Example: C_from_ric_to_inertial transforms a vector from RIC components to inertial components.
 */

#include <array>
#include <cmath>
#include <cstdint>

namespace bullseye_pred {

// -----------------------------
// Basic math types (minimal)
// -----------------------------

struct Vec3 final {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  constexpr Vec3() = default;
  constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

[[nodiscard]] inline constexpr Vec3 operator+(const Vec3& a, const Vec3& b) noexcept {
  return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}
[[nodiscard]] inline constexpr Vec3 operator-(const Vec3& a, const Vec3& b) noexcept {
  return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}
[[nodiscard]] inline constexpr Vec3 operator*(double s, const Vec3& v) noexcept {
  return Vec3{s * v.x, s * v.y, s * v.z};
}
[[nodiscard]] inline constexpr Vec3 operator*(const Vec3& v, double s) noexcept {
  return s * v;
}

[[nodiscard]] inline constexpr double dot(const Vec3& a, const Vec3& b) noexcept {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
[[nodiscard]] inline constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
  return Vec3{
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x};
}
[[nodiscard]] inline double norm(const Vec3& v) noexcept { return std::sqrt(dot(v, v)); }

// 3x3 matrix stored row-major: m[r][c]
struct Mat3 final {
  std::array<std::array<double, 3>, 3> m{{
      std::array<double, 3>{{1.0, 0.0, 0.0}},
      std::array<double, 3>{{0.0, 1.0, 0.0}},
      std::array<double, 3>{{0.0, 0.0, 1.0}},
  }};

  constexpr Mat3() = default;

  static constexpr Mat3 identity() noexcept { return Mat3{}; }

  [[nodiscard]] constexpr double operator()(int r, int c) const noexcept { return m[r][c]; }
  constexpr double& operator()(int r, int c) noexcept { return m[r][c]; }
};

[[nodiscard]] inline Vec3 mul(const Mat3& A, const Vec3& v) noexcept {
  return Vec3{
      A(0, 0) * v.x + A(0, 1) * v.y + A(0, 2) * v.z,
      A(1, 0) * v.x + A(1, 1) * v.y + A(1, 2) * v.z,
      A(2, 0) * v.x + A(2, 1) * v.y + A(2, 2) * v.z};
}

[[nodiscard]] inline Mat3 mul(const Mat3& A, const Mat3& B) noexcept {
  Mat3 C;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += A(r, k) * B(k, c);
      }
      C(r, c) = s;
    }
  }
  return C;
}

[[nodiscard]] inline Mat3 transpose(const Mat3& A) noexcept {
    Mat3 T;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            T(r, c) = A(c, r);
        }
    }
    return T;
}

[[nodiscard]] inline double det(const Mat3& A) noexcept {
  // Row-major determinant.
  const double a = A(0, 0), b = A(0, 1), c = A(0, 2);
  const double d = A(1, 0), e = A(1, 1), f = A(1, 2);
  const double g = A(2, 0), h = A(2, 1), i = A(2, 2);
  return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

// -----------------------------
// Core enums / status
// -----------------------------

enum class ProviderCode : std::uint8_t {
  kOk = 0,
  kTimeMissing,
  kFrameMismatch,
  kNotAvailable,
  kInvalidInput,
  kInternalError,
};

enum class OmegaCoords : std::uint8_t {
  kUnspecified = 0,
  kOmegaRIC,       // ω expressed in RIC coordinates (required for Option B usage).
  kOmegaInertial,  // present for completeness; not used in v1 transforms.
};

enum class FrameKind : std::uint8_t {
  kUnspecified = 0,
  kBullseyeRIC,
};

enum class AxisOrder : std::uint8_t {
  kUnspecified = 0,
  kRIC,  // {R, I, C}
};

struct ProviderStatus final {
  ProviderCode code{ProviderCode::kOk};

  [[nodiscard]] constexpr bool ok() const noexcept { return code == ProviderCode::kOk; }
};

// -----------------------------
// Common payloads (Sprint 2)
// -----------------------------

struct ChiefState final {
  double time_tag{0.0};
  Vec3 r_i{};  // Chief position in configured inertial frame components.
  Vec3 v_i{};  // Chief velocity in configured inertial frame components.
  // v1: frame identity is a string (JEOD frame name or user-defined inertial frame id).
  // Keep this header dependency-free; use string_view to avoid owning memory.
  // NOTE: providers must ensure the view outlives the use site (usually static/config strings).
  const char* frame_id{nullptr};
  ProviderStatus status{};
};

struct AdoptedRicFrame final {
  double time_tag{0.0};
  Vec3 origin_i{};  // Origin in inertial components.

  // Canonical orientation for v1: RIC -> inertial
  // x_i = C_from_ric_to_inertial * x_ric
  Mat3 C_from_ric_to_inertial{Mat3::identity()};

  bool has_omega{false};
  Vec3 omega_ric{};                 // Valid only if has_omega==true.
  OmegaCoords omega_coords{OmegaCoords::kUnspecified};

  FrameKind frame_kind{FrameKind::kUnspecified};
  AxisOrder axis_order{AxisOrder::kUnspecified};

  const char* frame_source_id{nullptr};  // Optional provenance (string view style).
  ProviderStatus status{};
};

}  // namespace bullseye_pred
