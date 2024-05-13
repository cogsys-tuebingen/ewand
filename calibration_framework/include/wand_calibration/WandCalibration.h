#pragma once

#include <rapidcsv.h>
#include <wand_calibration/BundleAdjustment.h>
#include <wand_calibration/Initialization.h>

#include <Eigen/Eigen>
#include <array>
#include <iostream>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <string>
#include <tuple>
#include <vector>

/**
 * @brief Main class for the wand calibration.
 */
class WandCalibration {
 public:
  WandCalibration();

  WandCalibration(const OptimizationOptions options,
                  const bool self_calibration);

  void addCameraData(const std::string csv_file,
                     const std::array<int, 2> camera_dimensions,
                     const Eigen::Matrix3d K,
                     const std::array<double, 4> distortions);

  void addCameraData(const std::string csv_file,
                     const std::string intriniscs_file);

  /**
   * @brief Main method managing the whole calibration
   *        * Get initial camera matrix and extrinsics
   *        * Calculates the 3D points from 2D observations from camera pairs
   *        * Runs the bundle adjustment
   */
  void calibrate();

  const Eigen::Matrix3d getK(size_t camera_index) const;

  const std::vector<Eigen::Matrix3d> getK() const;

  const std::array<double, 4> getDistortion(std::size_t camera_index) const;

  const std::vector<std::array<double, 4>> getDistortions() const;

  const std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>>
  getDataPoints() const;

 private:
  /// Flag indicating if we use self calibration to determine the intrinsics
  const bool self_calibration_;

  /// Flag indicating that we use the ground truth extrinsics (Used with the
  /// synthetic data generation script)
  const bool extrinsics_gt_;

  std::vector<double> getReprojectionError();

  /// Object handling the bundle adjustment
  BundleAdjustment bundle_adjustment_;

  /// Object handling the initialization (fundamental-, essential-matrix, camera
  /// matrix and extrinsics)
  Initialization initialization_;

  /// Data points [cam_number][image_number][{x,y}][i] (for every camera and
  /// image, a x-, y- vector)
  std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>> data_points_;

  /// Camera Matrix (for every camera)
  std::vector<Eigen::Matrix3d> K_;

  std::vector<std::array<int, 2>> camera_dimensions_;

  /// Distortion d0, d1, d2, d3 (for every camera the distortion)
  std::vector<std::array<double, 4>> distortions_;

  /// Extrinsic SE3 (for every camera) using Sophus
  std::vector<CameraPose> extrinsics_;

  std::vector<std::tuple<bool, Eigen::Vector3d>> points3d_;

  void exportToYaml(const std::vector<double>& errors,
                    const std::string prefix = "");

  void printIntrinsics();
};
