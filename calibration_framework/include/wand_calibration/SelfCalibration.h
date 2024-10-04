#pragma once

#include <Eigen/Eigen>
#include <tuple>

/**
 * @brief Performs the self-calibration introduced in: Sturm. On focal length
 * calibration from two views. CVPR 2001
 *
 * @param[in] F Fundamental matrix
 * @param[in] w Widht of the camera frame
 * @param[in] h Height of the camera frame
 */
namespace self_calibration {
std::tuple<Eigen::Matrix3d, Eigen::Matrix3d> selfCalibrate(Eigen::Matrix3d F,
                                                           int w, int h);

Eigen::Matrix3d normalizeNorm(Eigen::Matrix3d A);

Eigen::Matrix3d stabilize(Eigen::Matrix3d x, double tol = 1e-6);
}  // namespace self_calibration
