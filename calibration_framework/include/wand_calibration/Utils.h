#pragma once

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <tuple>
#include <vector>

struct CameraPose {
  size_t camera_index;
  Sophus::SE3d pose;
};

namespace utils {

void filterPoints(
    const std::vector<std::tuple<bool, Eigen::Vector2d>>& raw_pts1,
    const std::vector<std::tuple<bool, Eigen::Vector2d>>& raw_pts2,
    std::vector<cv::Point2d>& f_pts1, std::vector<cv::Point2d>& f_pts2);

void filterPoints(const std::vector<std::tuple<bool, Eigen::Vector3d>>& raw_pts,
                  std::vector<Eigen::Vector3d>& f_pts,
                  std::vector<int>& inliers);

Eigen::Matrix3d orthogonalize_rotation(const Eigen::Matrix3d& R);
/**
 * @brief Filter out points without a final observation
 *
 * @param[in] raw_pts Points to be filtered
 * @param[out] f_pts Output
 */
void filterPoints(const std::vector<std::tuple<bool, Eigen::Vector3d>>& raw_pts,
                  std::vector<Eigen::Vector3d>& f_pts);

void undistortPoint(const Eigen::Vector2d& dist_point,
                    Eigen::Vector2d& undist_point,
                    const std::array<double, 4>& d, const Eigen::Matrix3d& K);

void distortPoint(const Eigen::Vector2d& undist_point,
                  Eigen::Vector2d& dist_point, const std::array<double, 4>& d,
                  const Eigen::Matrix3d& K);

/**
 * @brief Calculates a 3D point by triangulation given the 2D observation from
 * a pair of cameras
 *
 * @param[out] triang_pt 3D point
 * @param[in] pt1 2D observation of the first camera
 * @param[in] pt2 2D observation of the second camera
 * @param[in] P1 Projection matrix of the first camera
 * @param[in] P2 Projection matrix of the second camera
 */
void calculate3dPoint(Eigen::Vector3d& triang_pt, const Eigen::Vector2d& pt1,
                      const Eigen::Vector2d& pt2,
                      const Eigen::Matrix<double, 3, 4>& P1,
                      const Eigen::Matrix<double, 3, 4>& P2);

bool valid3dPose(Eigen::Vector3d& pose);

/**
 * @brief Calculates the distance error between the markers of the wand
 *
 * @param[in] pts Wand markers in 3D
 * @return The error of the wand marker distances
 */
double getWandDistanceError(std::vector<Eigen::Vector3d>& pts);

Eigen::Matrix<double, 3, 4> getProjectionMat(
    const std::vector<Eigen::Matrix3d>& K,
    const std::vector<CameraPose>& extrinsics, const int cam_i);

/**
 * @brief Calculates the 3D points by triangulation given the 2D observation
 * from a pair of cameras
 *
 * @param[out] points3d 3D points
 * @param[in] K Camera matricess
 * @param[in] extrinsics Extrinsics
 * @param[in] data_points 2D observations
 * @param[in] cam1 Index of the first camera
 * @param[in] cam2 Index of the second camera
 */
void calculate3dPoints(
    std::vector<std::tuple<bool, Eigen::Vector3d>>& points3d,
    const std::vector<Eigen::Matrix3d>& K,
    const std::vector<std::array<double, 4>>& d,
    const std::vector<CameraPose>& extrinsics,
    const std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>>&
        data_points,
    int cam1, int cam2);
}  // namespace utils
