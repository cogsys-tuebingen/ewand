#include <rapidcsv.h>
#include <wand_calibration/Utils.h>
#include <wand_calibration/WandCalibration.h>

#include <array>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.hpp>
#include <string>
#include <tuple>
#include <vector>

WandCalibration::WandCalibration()
    : self_calibration_{false},
      extrinsics_gt_{false},
      initialization_{data_points_, points3d_, false, false} {}

WandCalibration::WandCalibration(const OptimizationOptions options,
                                 const bool self_calibration)
    : self_calibration_{self_calibration},
      extrinsics_gt_{options.extrinsics_gt},
      initialization_{data_points_, points3d_, self_calibration,
                      options.extrinsics_gt},
      bundle_adjustment_{options} {}

void WandCalibration::addCameraData(const std::string csv_file,
                                    const std::array<int, 2> camera_dimensions,
                                    const Eigen::Matrix3d K,
                                    const std::array<double, 4> distortions) {
  camera_dimensions_.emplace_back(camera_dimensions);
  distortions_.emplace_back(distortions);
  if (K != Eigen::Matrix3d::Zero()) {
    K_.emplace_back(K);
  }

  // Load csv
  std::cout << "file_name: " << csv_file << std::endl;
  std::vector<std::tuple<bool, Eigen::Vector2d>> points;
  rapidcsv::Document doc(csv_file, rapidcsv::LabelParams(-1, -1),
                         rapidcsv::SeparatorParams(';'));

  // Go thorugh all 2d observations in csv file
  for (size_t i = 0; i < doc.GetRowCount(); i++) {
    std::vector<double> line = doc.GetRow<double>(i);
    const std::size_t offset = 2;
    // Ignore first entry (timestamp)
    for (size_t idx = 1; idx < line.size(); idx += offset) {
      std::tuple<bool, Eigen::Vector2d> point;
      if (line[idx] >= 0 && line[idx + 1] >= 0) {
        point = {true, {line[idx], line[idx + 1]}};
      } else {
        point = {false, {line[idx], line[idx + 1]}};
      }
      points.emplace_back(point);
    }
  }

  // save data points
  data_points_.emplace_back(points);

  initialization_.setK(K_);
  initialization_.setDistortion(distortions_);
}

void WandCalibration::addCameraData(const std::string csv_file,
                                    const std::string config_file) {
  std::cout << "file name: " << config_file << std::endl;
  cv::FileStorage fs;
  fs.open(config_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Failed to open " << config_file << std::endl;
    return;
  }

  int width;
  fs["image_width"] >> width;
  int height;
  fs["image_height"] >> height;
  std::array<int, 2> camera_dimensions{width, height};

  std::array<double, 4> distortions;
  Eigen::Matrix3d eigen_K;
  if (self_calibration_) {
    distortions = {0.0, 0.0, 0.0, 0.0};
    eigen_K = Eigen::Matrix3d::Zero();
  } else {
    cv::Mat K;
    cv::Mat dist_coeffs;
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> dist_coeffs;
    distortions = {dist_coeffs.at<double>(0), dist_coeffs.at<double>(1),
                   dist_coeffs.at<double>(2), dist_coeffs.at<double>(3)};

    cv::cv2eigen(K, eigen_K);
  }

  addCameraData(csv_file, camera_dimensions, eigen_K, distortions);
}

std::vector<double> WandCalibration::getReprojectionError() {
  // Reprojection for each camera
  std::vector<double> errors(K_.size(), 0);
  int n_valid = 0;

  for (std::size_t i = 0; i < data_points_.at(1).size(); i++) {
    if (std::get<0>(points3d_.at(i))) {
      n_valid += 1;
      Eigen::Vector4d point3d;
      point3d.head(3) = std::get<1>(points3d_.at(i));
      point3d(3) = 1.0;
      for (std::size_t cam = 0; cam < K_.size(); cam++) {
        Eigen::Matrix<double, 3, 4> P =
            utils::getProjectionMat(K_, extrinsics_, cam);
        Eigen::Vector3d point2d_undist = P * point3d;
        // Eigen::Vector3d point2d_undist;
        // point2d_undist = K_.at(cam) * point3d.head(3);
        // std::cout << "point3d:\n" << point3d << std::endl;
        // std::cout << "K_:\n" << K_.at(cam) << std::endl;;
        // std::cout << "point2d_undist:\n" << point2d_undist << std::endl;
        // Normalizing point coordinates
        point2d_undist = point2d_undist / point2d_undist(2);
        // std::cout << "point2d_undist:\n" << point2d_undist << std::endl;
        Eigen::Vector2d point2d_dist;
        utils::distortPoint(point2d_undist.head(2), point2d_dist,
                            distortions_.at(cam), K_.at(cam));
        // std::cout << "point2d_dist:\n" << point2d_dist << std::endl;
        // std::cout << "observation:\n" <<
        // std::get<1>(data_points_.at(cam).at(i)) << std::endl; std::cout <<
        // std::endl;
        Eigen::Vector2d diff =
            std::get<1>(data_points_.at(cam).at(i)) - point2d_dist;
        errors.at(cam) += diff.norm();
      }
    }
  }
  for (std::size_t cam = 0; cam < K_.size(); cam++) {
    errors.at(cam) = errors.at(cam) / n_valid;
    // std::cout << errors.at(cam) << std::endl;
  }
  return errors;
}

void WandCalibration::calibrate() {
  tie(K_, extrinsics_) = initialization_.runInitialization(camera_dimensions_);
  std::cout << "Before bundle adjustment:" << std::endl;
  printIntrinsics();

  std::cout << "calculate3dPoints done" << std::endl;
  utils::calculate3dPoints(points3d_, K_, distortions_, extrinsics_,
                           data_points_, 0, 1);
  std::vector<double> errors = getReprojectionError();
  std::cout << "Reprojection error before bundle adjustment" << std::endl;
  for (std::size_t cam = 0; cam < K_.size(); cam++) {
    std::cout << errors.at(cam) << std::endl;
  }

  bundle_adjustment_.optimize(distortions_, K_, extrinsics_, points3d_,
                              data_points_);
  utils::calculate3dPoints(points3d_, K_, distortions_, extrinsics_,
                           data_points_, 0, 1);

  std::cout << "After bundle adjustment:" << std::endl;
  printIntrinsics();

  errors = getReprojectionError();
  std::cout << "Reprojection after bundle adjustment" << std::endl;
  for (std::size_t cam = 0; cam < K_.size(); cam++) {
    std::cout << errors.at(cam) << std::endl;
  }

  exportToYaml(errors, "after_ba_");
}

void WandCalibration::exportToYaml(const std::vector<double>& errors,
                                   const std::string prefix) {
  for (std::size_t i = 0; i < K_.size(); ++i) {
    cv::FileStorage fs(prefix + "cam_" + std::to_string(i) + ".yaml",
                       cv::FileStorage::WRITE);
    fs << "camera_number" << static_cast<int>(i);

    cv::Mat cv_K;
    cv::eigen2cv(K_.at(i), cv_K);
    fs << "camera_matrix" << cv_K;
    cv::Mat distortion = cv::Mat(1, 4, CV_64FC1, distortions_.at(i).data());
    fs << "distortion_coefficients" << distortion;
    cv::Mat extrinsics;
    cv::eigen2cv(extrinsics_.at(i).pose.matrix(), extrinsics);
    fs << "extrinsics" << extrinsics;
    fs << "reprojection_error" << errors.at(i);

    fs.release();
  }
}

void WandCalibration::printIntrinsics() {
  std::cout << "Camera matrices:" << std::endl;
  for (std::size_t i = 0; i < K_.size(); ++i) {
    std::cout << "Camera " << i << "\n" << K_.at(i) << std::endl;
    std::cout << "Distortion: " << distortions_.at(i).at(0) << ", "
              << distortions_.at(i).at(1) << ", " << distortions_.at(i).at(2)
              << ", " << distortions_.at(i).at(3) << std::endl;
  }
}

const Eigen::Matrix3d WandCalibration::getK(std::size_t camera_index) const {
  return K_.at(camera_index);
}

const std::vector<Eigen::Matrix3d> WandCalibration::getK() const { return K_; }

const std::array<double, 4> WandCalibration::getDistortion(
    size_t camera_index) const {
  return distortions_.at(camera_index);
}

const std::vector<std::array<double, 4>> WandCalibration::getDistortions()
    const {
  return distortions_;
}

const std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>>
WandCalibration::getDataPoints() const {
  return data_points_;
}
