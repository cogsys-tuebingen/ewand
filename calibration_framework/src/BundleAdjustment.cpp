#include <ceres/rotation.h>
#include <ceres/ceres.h>
#include <omp.h>
#include <wand_calibration/BundleAdjustment.h>

#include <fstream>
#include <sophus/ceres_local_parameterization.hpp>

/**
 * @brief Reprojection error implementation used by Ceres
 */
class ReprojectionError {
 public:
  ReprojectionError(double _observed_x, double _observed_y)
      : observed_x{_observed_x}, observed_y{_observed_y} {}

  template <typename T>
  bool operator()(const T* const camera, const T* const K,
                  const T* const distortion, const T* const point,
                  T* residuals) const {
    const Eigen::Map<const Sophus::SE3<T>> se3(camera);

    const Eigen::Map<const Eigen::Matrix<T, 3, 3>> K_eigen(K);
    std::array<T, 4> dist{distortion[0], distortion[1], distortion[2],
                          distortion[3]};

    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> vec3(point);
    Eigen::Matrix<T, 4, 1> vec;
    vec << vec3, Eigen::Matrix<T, 1, 1>::Ones();

    // Converting from world coordinates to camera coordinates
    Eigen::Matrix<T, 4, 1> p = se3.matrix() * vec;

    /*
    T predicted_x = p[0] / p[2] * K_eigen(0, 0) + K_eigen(0, 2);
    T predicted_y = p[1] / p[2] * K_eigen(1, 1) + K_eigen(1, 2);
    */

    // Converting to normalized camera coordiantes
    T undistorted_x = p[0] / p[2];
    T undistorted_y = p[1] / p[2];

    // std::cout << "undistorted_x: " << undistorted_x << std::endl;
    // std::cout << "undistorted_y: " << undistorted_y << std::endl;

    // r^2
    T r_2 = undistorted_x * undistorted_x + undistorted_y * undistorted_y;

    // std::cout << "undistorted_x: " << undistorted_x << std::endl;
    // std::cout << "undistorted_y: " << undistorted_y << std::endl;

    // Applying distortion
    T predicted_x =
        // Radial distortion
        undistorted_x * (T(1) + dist.at(0) * r_2 + dist.at(1) * r_2 * r_2)
        // Tangential distortion
        + (T(2) * dist.at(2) * undistorted_x * undistorted_y +
           dist.at(3) * (r_2 + T(2) * undistorted_x * undistorted_x));

    T predicted_y =
        // Radial distortion
        undistorted_y * (T(1) + dist.at(0) * r_2 + dist.at(1) * r_2 * r_2)
        // Tangential distortion
        + (dist.at(2) * (r_2 + T(2) * undistorted_y * undistorted_y) +
           T(2) * dist.at(3) * undistorted_x * undistorted_y);

    // Converting back to pixel coordinates
    predicted_x = predicted_x * K_eigen(0, 0) + K_eigen(0, 2);
    predicted_y = predicted_y * K_eigen(1, 1) + K_eigen(1, 2);

    // std::cout << "predicted_x: " << predicted_x << std::endl;
    // std::cout << "predicted_y: " << predicted_y << std::endl;
    // std::cout << "observed_x: " << observed_x << std::endl;
    // std::cout << "observed_y: " << observed_y << std::endl;

    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    // std::cout << "repro residuals[0]: " << residuals[0] << std::endl;
    // std::cout << "repro residuals[1]: " << residuals[1] << std::endl;

    return true;
  }

 private:
  double observed_x, observed_y;
};

/**
 * @brief Wand constraints used by Ceres
 * @note Scaling is used since the reprojection error is in [pixels] and the
 * wand constraint in [meters] to bring the error terms into the same range.
 */
class WandConstraints {
 public:
  WandConstraints(double distance_scaling, double linearity_scaling)
  : distance_scaling_{distance_scaling},
    linearity_scaling_{linearity_scaling} {}
  template <typename T>
  bool operator()(const T* const point_1, const T* const point_2,
                  const T* const point_3, T* residuals) const {
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p1(point_1);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p2(point_2);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p3(point_3);

    T distance_marker_1_2 = (p2 - p1).norm();
    T distance_marker_2_3 = (p3 - p2).norm();

    residuals[0] = distance_marker_1_2 - T(marker_1_2_);
    residuals[0] *= T(distance_scaling_);

    residuals[1] = distance_marker_2_3 - T(marker_2_3_);
    residuals[1] *= T(distance_scaling_);

    // Line constraint (Collinearity of Three Points
    // https://www.math-only-math.com/collinearity-of-three-points.html)
    // residuals[2] = x1 * (y2 - y3) + x2 * (y3 -y1) + x3 * (y1 - y2);
    auto line_constraint = p1(0) * (p2(1) - p3(1)) + p2(0) * (p3(1) - p1(1)) +
                           p3(0) * (p1(1) - p2(1));

    if (line_constraint < T(0)) {
      line_constraint *= T(-1);
    }
    residuals[2] = line_constraint;
    residuals[2] *= T(linearity_scaling_);

    // std::cout << "wc residuals[0]: " << residuals[0] << std::endl;
    // std::cout << "wc residuals[1]: " << residuals[1] << std::endl;
    // std::cout << "wc residuals[2]: " << residuals[2] << std::endl;

    return true;
  }

 private:
  double marker_1_2_ = 0.13;  // In meters
  double marker_2_3_ = 0.26;  // In meters
  double distance_scaling_;
  double linearity_scaling_;
};

BundleAdjustment::BundleAdjustment() {
  std::size_t nr_observations = 0;
  auto fix_distortion = false;
  auto fix_cammatrix = false;
  auto fix_extrinsics = false;
  auto extrinsics_gt = false;
  auto ignore_wand = false;
  double wc_distance_scaling = 1.0;
  double wc_linearity_scaling = 1.0;
  options_ = OptimizationOptions(nr_observations, fix_distortion, fix_cammatrix,
                                 fix_extrinsics, extrinsics_gt, ignore_wand,
                                 wc_distance_scaling, wc_linearity_scaling);
}

BundleAdjustment::BundleAdjustment(OptimizationOptions options)
    : options_{options} {}

void BundleAdjustment::optimize(
    std::vector<std::array<double, 4>>& distortions,
    std::vector<Eigen::Matrix3d>& K, std::vector<CameraPose>& extrinsics,
    std::vector<std::tuple<bool, Eigen::Vector3d>>& points3d,
    std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>>& data_points) {
  ceres::Problem problem;
  ceres::LocalParameterization* se3_parameterization =
      new Sophus::LocalParameterization<Sophus::SE3>;

  // Add camera pose (extrinsic) to optimization (we do not include the
  // reference camera)
  for (std::size_t cam_i = 0; cam_i < extrinsics.size(); ++cam_i) {
    problem.AddParameterBlock(extrinsics.at(cam_i).pose.data(),
                              Sophus::SE3d::num_parameters,
                              se3_parameterization);
  }
  // Set extrinsics of reference camera constant
  problem.SetParameterBlockConstant(extrinsics.at(0).pose.data());

  // @todo remove the following since it is only for debugging BA
  if (options_.fix_extrinsics) {
    for (std::size_t i = 0; i < extrinsics.size(); ++i) {
      problem.SetParameterBlockConstant(extrinsics.at(i).pose.data());
    }
  }

  // Add camera matrix as parameter
  for (std::size_t cam_i = 0; cam_i < K.size(); ++cam_i) {
    problem.AddParameterBlock(K.at(cam_i).data(), 9);
  }
  if (options_.fix_cammatrix) {
    for (std::size_t cam_i = 0; cam_i < K.size(); ++cam_i) {
      problem.SetParameterBlockConstant(K.at(cam_i).data());
    }
  }

  // Add distortion as parameter
  for (std::size_t cam_i = 0; cam_i < distortions.size(); ++cam_i) {
    problem.AddParameterBlock(distortions.at(cam_i).data(), 4);
  }
  if (options_.fix_distortion) {
    for (std::size_t cam_i = 0; cam_i < distortions.size(); ++cam_i) {
      problem.SetParameterBlockConstant(distortions.at(cam_i).data());
    }
  }

  auto limit = points3d.size();
  if (options_.nr_observations > 0) {
    limit = options_.nr_observations * 3;
  }
  std::cout << "Number of observations used for optimization " << limit
            << std::endl;

  std::vector<std::size_t> cam_observations;
  for (std::size_t i = 0; i < extrinsics.size(); ++i) {
    cam_observations.emplace_back(0);
  }

  // ceres::LossFunction *huber_repr = new ceres::HuberLoss(5);
  // ceres::LossFunction *huber_wand = new ceres::HuberLoss(1000);
  ceres::LossFunction *cauchy = new ceres::CauchyLoss(1);

  std::vector<ceres::ResidualBlockId> repr_residual_block_ids;
  std::vector<ceres::ResidualBlockId> wand_residual_block_ids;

  for (std::size_t point_i = 0; point_i < limit; ++point_i) {
    if (std::get<0>(points3d.at(point_i))) {
      // Add 3D point coordinates to optimization
      problem.AddParameterBlock(std::get<1>(points3d.at(point_i)).data(), 3);

      for (std::size_t cam_i = 0; cam_i < extrinsics.size(); ++cam_i) {
        if (std::get<0>(data_points.at(cam_i).at(point_i))) {
          cam_observations.at(cam_i) += 1;
          // Add a 2D observation
          ReprojectionError* constraint = new ReprojectionError(
              std::get<1>(data_points.at(cam_i).at(point_i))(0),
              std::get<1>(data_points.at(cam_i).at(point_i))(1));
          auto cost_func_auto =
              new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7, 9, 4, 3>(
                  constraint);

          auto id = problem.AddResidualBlock(cost_func_auto, cauchy /* qauchy loss */,
                                   extrinsics.at(cam_i).pose.data(),
                                   K.at(cam_i).data(),
                                   distortions.at(cam_i).data(),
                                   std::get<1>(points3d.at(point_i)).data());
          repr_residual_block_ids.emplace_back(id);

          // Add wand constraints (distance and Collinearity) using all the
          // points of the wand
          if (!options_.ignore_wand) {
            if (point_i > 1 && (point_i + 1) % 3 == 0 &&
                std::get<0>(points3d.at(point_i - 2)) &&
                std::get<0>(points3d.at(point_i - 1)) &&
                std::get<0>(points3d.at(point_i))) {
              WandConstraints* constraint =
                  new WandConstraints(options_.wc_distance_scaling, options_.wc_linearity_scaling);
              auto cost_func_auto =
                  new ceres::AutoDiffCostFunction<WandConstraints, 3, 3, 3, 3>(
                      constraint);

              if ((std::get<1>(points3d.at(point_i - 2)) - std::get<1>(points3d.at(point_i - 1))).norm() < 1e-3) {
                std::cout << "Same point: point_i: " << point_i << std::endl;
              }
              
              // @todo: Maybe  squared loss is not good
              auto id = problem.AddResidualBlock(
                  cost_func_auto, cauchy /* cauchy loss */,
                  std::get<1>(points3d.at(point_i - 2)).data(),
                  std::get<1>(points3d.at(point_i - 1)).data(),
                  std::get<1>(points3d.at(point_i)).data());
              wand_residual_block_ids.emplace_back(id);
            }
          }
        }
      }
    }
  }

  std::cout << "Nr. of observations:" << std::endl;
  for (std::size_t i = 0; i < cam_observations.size(); ++i) {
    std::cout << "- Cam " << i << " " << cam_observations.at(i) << " observations" << std::endl;
  }

  std::cout << "Extrinsics before optimization:" << std::endl;
  for (const auto& extrinsic : extrinsics) {
    std::cout << "Camera: " << extrinsic.camera_index << "\n"
              << extrinsic.pose.matrix() << std::endl;
  }

  // Write 3D points to csv
  std::ofstream csv_file;
  csv_file.open("3d_points_initialized.csv");
  for (const auto& bool_point : points3d) {
    const auto& point = std::get<1>(bool_point);
    csv_file << std::to_string(point(0)) + ";" + std::to_string(point(1)) +
                    ";" + std::to_string(point(2)) + "\n";
  }
  csv_file.close();

  ceres::Solver::Options options;
  // options.linear_solver_type = ceres::DENSE_SCHUR;
  // options.linear_solver_type = ceres::DENSE_QR;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  // options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  // options.logging_type = ceres::SILENT;
  options.max_num_iterations = 5000;
  // options.max_num_iterations = 10;
  options.num_threads = omp_get_max_threads();
  // options.num_threads = 1;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  /* Calculate residual */
  ceres::Problem::EvaluateOptions evaluate_options;
  evaluate_options.num_threads = 8;
  evaluate_options.apply_loss_function = false;
  evaluate_options.residual_blocks = repr_residual_block_ids;

  std::vector<double> repr_residuals;
  problem.Evaluate(evaluate_options, nullptr, &repr_residuals, nullptr, nullptr);
  FILE* repr_psr_residual = fopen("repr_residuals.csv", "w+");
  for(std::size_t i = 0; i < repr_residuals.size(); i++) {
    fprintf(repr_psr_residual, "%7.9f \n", repr_residuals.at(i));
    fflush(repr_psr_residual);  
  }

  evaluate_options.residual_blocks = wand_residual_block_ids;

  std::vector<double> wand_residuals;
  problem.Evaluate(evaluate_options, nullptr, &wand_residuals, nullptr, nullptr);
  FILE* wand_psr_residual = fopen("wand_residuals.csv", "w+");
  for(std::size_t i = 0; i < wand_residuals.size(); i++) {
    fprintf(wand_psr_residual, "%7.9f \n", wand_residuals.at(i));
    fflush(wand_psr_residual);  
  }

  std::cout << "Extrinsics after optimization:" << std::endl;
  for (const auto& extrinsic : extrinsics) {
    std::cout << "Camera: " << extrinsic.camera_index << "\n"
              << extrinsic.pose.matrix() << std::endl;
  }

  // Write 3D points to csv
  csv_file.open("3d_points_optimized.csv");
  for (const auto& bool_point : points3d) {
    const auto& point = std::get<1>(bool_point);
    csv_file << std::to_string(point(0)) + ";" + std::to_string(point(1)) +
                    ";" + std::to_string(point(2)) + "\n";
  }
  csv_file.close();
}
