#pragma once

#include <Eigen/Dense>
#include <nonstd/expected.hpp>
#include <units/units.h>

namespace pathplanning {
using Real = double;

template <size_t Rows, size_t Cols> using Mat = Eigen::Matrix<Real, Rows, Cols>;
using Mat3                                    = Mat<3, 3>;
using Mat4                                    = Mat<4, 4>;
template <size_t Dim> using Vec               = Mat<Dim, 1>;
using Vec2                                    = Vec<2>;
using Vec3                                    = Vec<3>;
template <size_t Dim> using Veci              = Eigen::Matrix<int, Dim, 1>;
using Vec2i                                   = Veci<2>;
using Vec3i                                   = Veci<3>;
using Quat                                    = Eigen::Quaternion<Real>;
using AngAx                                   = Eigen::AngleAxis<Real>;
using Affine                                  = Eigen::Transform<Real, 3, Eigen::Affine>;

using namespace units::literals;
using namespace units;
using namespace units::time;
using namespace units::energy;

using TimePoint = seconds;
using TimeDiff  = seconds;
using seconds_t = Real;

using namespace nonstd;

} // namespace pathplanning