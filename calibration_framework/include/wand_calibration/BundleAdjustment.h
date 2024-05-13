#pragma once

#include <wand_calibration/Utils.h>

#include <cstdio>
#include <sophus/se3.hpp>
#include <vector>

struct OptimizationOptions {
  OptimizationOptions()
  : nr_observations{ 0 }, 
    fix_distortion{ false },
    fix_cammatrix{ false },
    fix_extrinsics{ false },
    extrinsics_gt{ false },
    ignore_wand{ false },
    wc_distance_scaling { 1.0 },
    wc_linearity_scaling { 1.0 } {}

  OptimizationOptions(const std::size_t _nr_observations,
                      const bool _fix_distortion, const bool _fix_cammatrix,
                      const bool _fix_extrinsics, const bool _extrinsics_gt,
                      const bool _ignore_wand, const double _wc_distance_scaling, const double _wc_linearity_scaling)
  : nr_observations{ _nr_observations }, 
    fix_distortion{ _fix_distortion },
    fix_cammatrix{ _fix_cammatrix },
    fix_extrinsics{ _fix_extrinsics },
    extrinsics_gt{ _extrinsics_gt },
    ignore_wand{ _ignore_wand },
    wc_distance_scaling { _wc_distance_scaling },
    wc_linearity_scaling { _wc_linearity_scaling } {}

  OptimizationOptions(const OptimizationOptions&) = default;

  std::size_t nr_observations;
  bool fix_distortion;
  bool fix_cammatrix;
  bool fix_extrinsics;
  bool extrinsics_gt;
  bool ignore_wand;
  double wc_distance_scaling;
  double wc_linearity_scaling;
};

/**
 * @brief Class interfacing Ceres for the bundle adjustement
 */
class BundleAdjustment {
 public:
  BundleAdjustment();

  BundleAdjustment(OptimizationOptions options);

  /**
   * @brief Bundle adjustment inspired by
   *        https://www.christian-diller.de/projects/bundle-a-ceres/
   *        https://github.com/chrdiller/BundleACeres
   */
  void optimize(
      std::vector<std::array<double, 4>>& distortions,
      std::vector<Eigen::Matrix3d>& K, std::vector<CameraPose>& extrinsics,
      std::vector<std::tuple<bool, Eigen::Vector3d>>& points3d,
      std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>>& data_points);

 private:
  OptimizationOptions options_;
};
