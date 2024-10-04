#pragma once

#include <wand_calibration/Utils.h>

#include <Eigen/Eigen>
#include <vector>

class Initialization {
 public:
  Initialization(
      const std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>>&
          data_points,
      std::vector<std::tuple<bool, Eigen::Vector3d>>& points3d,
      const bool self_calibration, const bool extrinsics_gt);

  /**
   * @brief Sets the camera matrices
   *
   * @param[in] K Camera matrices
   */
  void setK(const std::vector<Eigen::Matrix3d>& K);

  /**
   * @brief Sets the lens distortion
   *
   * @param[in] d Distortion coefficients
   */
  void setDistortion(const std::vector<std::array<double, 4>>& d);

  /**
   * @brief Calculates the fundamental matrices for all the cameras
   *        with respect to the first camera.
   */
  void calculateFundamental();

  /**
   * @brief Calculates the essential matrix and initializes the extrinsics
   */
  void calculateEssentialAndPose();

  /**
   * @brief Performs a golden section search using the distance between the
   *        wand markers to refine the scaling of the translation.
   */
  void refineTranslation();

  /**
   * @brief Main method of the Initialization.
   *        * Calculates the fundamental matrices
   *        * Runs the self-calibration
   *        * Initializes the extrinsics
   *        * Refines the translation
   *
   * @param[in] camera_dimensions Dimensions (width and height) of the cameras
   */
  std::tuple<std::vector<Eigen::Matrix3d>, std::vector<CameraPose>>
  runInitialization(const std::vector<std::array<int, 2>>& camera_dimensions);

 private:
  /// Data points [cam_number][image_number][{x,y}][i] (for every camera and
  /// image, a x-, y- vector)
  const std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>>&
      data_points_;

  std::vector<std::tuple<bool, Eigen::Vector3d>>& points3d_;

  std::vector<Eigen::Matrix3d> fundamentalMatrices_;

  std::vector<Eigen::Matrix3d> essentialMatrices_;

  /// Camera Matrix (for every camera)
  std::vector<Eigen::Matrix3d> K_;

  // Distortion coefficients (for every camera)
  std::vector<std::array<double, 4>> distortion_;

  /// Extrinsic SE3 (for every camera) using Sophus
  std::vector<CameraPose> extrinsics_;

  /// Flag indicating if we use self calibration to determine the intrinsics
  const bool self_calibration_;

  /// Flag indicating that we use the ground truth extrinsics (Used with the
  /// synthetic data generation script)
  const bool extrinsics_gt_;
};
