#ifndef DISCREGRID_COMMON_HPP
#define DISCREGRID_COMMON_HPP

#include <Eigen/Dense>

namespace Discregrid {
#ifdef USE_DOUBLE
typedef double Real;
#else
typedef float Real;
#endif

using Vector3r = Eigen::Matrix<Real, 3, 1, Eigen::DontAlign>;
using Matrix3r = Eigen::Matrix<Real, 3, 3, Eigen::DontAlign>;
using AlignedBox3r = Eigen::AlignedBox<Real, 3>;
}  // namespace Discregrid

#endif
