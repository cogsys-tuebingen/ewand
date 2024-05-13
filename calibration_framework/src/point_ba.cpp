#include <rapidcsv.h>
#include <wand_calibration/BundleAdjustment.h>
#include <wand_calibration/Utils.h>

#include <array>
#include <boost/program_options.hpp>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv4/opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <string>
#include <tuple>
#include <vector>

#include "wand_calibration/Utils.h"

std::vector<double> getReprojectionError(
    std::vector<std::tuple<bool, Eigen::Vector3d>>& points3d,
    const std::vector<Eigen::Matrix3d>& Ks,
    const std::vector<std::array<double, 4>>& ds,
    const std::vector<CameraPose>& extrinsics,
    const std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>>&
        data_points) {
  // Reprojection for each camera
  std::vector<double> errors(Ks.size(), 0);
  int n_valid = 0;

  std::vector<std::size_t> nr_data_points;
  for (std::size_t cam_i = 0; cam_i < Ks.size(); ++cam_i) {
    nr_data_points.emplace_back(data_points.at(cam_i).size());
  }
  std::size_t min_nr_data_points =
      *std::min_element(nr_data_points.begin(), nr_data_points.end());

  for (std::size_t i = 0; i < min_nr_data_points; i++) {
    if (std::get<0>(points3d.at(i))) {
      n_valid += 1;
      Eigen::Vector4d point3d;
      point3d.head(3) = std::get<1>(points3d.at(i));
      point3d(3) = 1.0;
      for (std::size_t cam = 0; cam < Ks.size(); cam++) {
        Eigen::Matrix<double, 3, 4> P =
            utils::getProjectionMat(Ks, extrinsics, cam);
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
        utils::distortPoint(point2d_undist.head(2), point2d_dist, ds.at(cam),
                            Ks.at(cam));
        // if (cam == 0) {
        //   std::cout << i << ";" << point2d_dist[0] << ";" << point2d_dist[1]
        //             << std::endl;
        // }
        // std::cout << "point2d_dist:\n" << point2d_dist << std::endl;
        // std::cout << "observation:\n" <<
        // std::get<1>(data_points_.at(cam).at(i)) << std::endl; std::cout <<
        // std::endl;
        Eigen::Vector2d diff =
            std::get<1>(data_points.at(cam).at(i)) - point2d_dist;
        // std::cout << diff.norm() << std::endl;
        errors.at(cam) += diff.norm();
      }
    }
  }
  for (std::size_t cam = 0; cam < Ks.size(); cam++) {
    errors.at(cam) = errors.at(cam) / n_valid;
    // std::cout << errors.at(cam) << std::endl;
  }
  return errors;
}

void read_calib(const std::string config_file, Eigen::Matrix3d& K,
                std::array<double, 4>& d, CameraPose& extrinsics) {
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
  cv::Mat cv_K;
  cv::Mat cv_extrinsics;
  cv::Mat cv_d;
  fs["camera_matrix"] >> cv_K;
  fs["distortion_coefficients"] >> cv_d;
  fs["extrinsics"] >> cv_extrinsics;
  extrinsics.camera_index = 0;
  Eigen::Vector3d t;
  Eigen::Matrix3d R;
  cv::cv2eigen(cv_extrinsics.colRange(3, 4).rowRange(0, 3), t);
  cv::cv2eigen(cv_extrinsics.colRange(0, 3).rowRange(0, 3), R);
  R = utils::orthogonalize_rotation(R);
  extrinsics.pose = Sophus::SE3d(R, t);
  d = {cv_d.at<double>(0), cv_d.at<double>(1), cv_d.at<double>(2),
       cv_d.at<double>(3)};
  cv::cv2eigen(cv_K, K);
}

std::vector<std::tuple<bool, Eigen::Vector2d>> read_points(
    const std::string csv_file) {
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
    for (size_t idx = 0; idx < line.size(); idx += offset) {
      std::tuple<bool, Eigen::Vector2d> point;
      if (line[idx] >= 0 && line[idx + 1] >= 0) {
        point = {true, {line[idx], line[idx + 1]}};
      } else {
        point = {false, {line[idx], line[idx + 1]}};
      }
      std::cout << std::get<1>(point) << std::endl << std::endl;
      ;
      points.emplace_back(point);
    }
  }
  std::cout << points.size() << std::endl;
  return points;
}

int main(int argc, char** argv) {
  boost::program_options::options_description od{"Options"};
  od.add_options()("help,h", "Help screen")(
      "cam1_config",
      boost::program_options::value<std::string>()->default_value(""),
      "Camera config of first camera")(
      "cam1_data",
      boost::program_options::value<std::string>()->default_value(""),
      "Observations of first camera")(
      "cam2_config",
      boost::program_options::value<std::string>()->default_value(""),
      "Camera config of second camera")(
      "cam2_data",
      boost::program_options::value<std::string>()->default_value(""),
      "Observations of second camera")(
      "cam3_config",
      boost::program_options::value<std::string>()->default_value(""),
      "Camera config of third camera")(
      "cam3_data",
      boost::program_options::value<std::string>()->default_value(""),
      "Observations of third camera")(
      "cam4_config",
      boost::program_options::value<std::string>()->default_value(""),
      "Camera config of fourth camera")(
      "cam4_data",
      boost::program_options::value<std::string>()->default_value(""),
      "Observations of fourth camera")(
      "cam5_config",
      boost::program_options::value<std::string>()->default_value(""),
      "Camera config of fifth camera")(
      "cam5_data",
      boost::program_options::value<std::string>()->default_value(""),
      "Observations of fifth camera")(
      "cam6_config",
      boost::program_options::value<std::string>()->default_value(""),
      "Camera config of sixth camera")(
      "cam6_data",
      boost::program_options::value<std::string>()->default_value(""),
      "Observations of sixth camera")(
      "nr_observations", boost::program_options::value<int>()->default_value(0),
      "Maximum number of wand observation considered for the optimization (0 "
      "represent no limit)");

  boost::program_options::variables_map vm;
  boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, od), vm);
  boost::program_options::notify(vm);

  if (vm.count("help")) {
    std::cout << od << "\n";
    return false;
  }

  auto nr_observations = vm["nr_observations"].as<int>();
  if (nr_observations > 0) {
    std::cout << "We will consider " << nr_observations << " wand observations."
              << std::endl;
  }
  bool fix_distortion = true;
  bool fix_cammatrix = true;
  bool fix_extrinsics = true;
  bool extrinsics_gt = false;
  bool ignore_wand = true;
  bool self_calibration = false;
  double wc_distance_scaling = 1;
  double wc_linearity_scaling = 1;
  OptimizationOptions options(nr_observations, fix_distortion, fix_cammatrix,
                              fix_extrinsics, extrinsics_gt, ignore_wand,
                              wc_distance_scaling, wc_linearity_scaling);

  std::vector<Eigen::Matrix3d> Ks;
  std::vector<std::array<double, 4>> ds;
  std::vector<CameraPose> extrinsics;
  std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>> data_points_;

  int offset = 0;

  auto cam1_config = vm["cam1_config"].as<std::string>();
  auto cam1_data = vm["cam1_data"].as<std::string>();
  if (!cam1_config.empty() && !cam1_data.empty()) {
    Eigen::Matrix3d K;
    std::array<double, 4> d;
    CameraPose extrinsic;
    read_calib(cam1_config, K, d, extrinsic);
    Ks.push_back(K);
    ds.push_back(d);
    extrinsics.push_back(extrinsic);

    // std::vector<std::tuple<bool, Eigen::Vector2d>> tmp_vec =
    //     read_points(cam1_data);
    // data_points_.push_back({tmp_vec.begin() + offset, tmp_vec.end()});
    data_points_.push_back(read_points(cam1_data));
  }

  auto cam2_config = vm["cam2_config"].as<std::string>();
  auto cam2_data = vm["cam2_data"].as<std::string>();
  if (!cam2_config.empty() && !cam2_data.empty()) {
    Eigen::Matrix3d K;
    std::array<double, 4> d;
    CameraPose extrinsic;
    read_calib(cam2_config, K, d, extrinsic);
    Ks.push_back(K);
    ds.push_back(d);
    extrinsics.push_back(extrinsic);
    // std::vector<std::tuple<bool, Eigen::Vector2d>> tmp_vec =
    //     read_points(cam2_data);
    // data_points_.push_back({tmp_vec.begin() + offset, tmp_vec.end()});
    data_points_.push_back(read_points(cam2_data));
  }

  auto cam3_config = vm["cam3_config"].as<std::string>();
  auto cam3_data = vm["cam3_data"].as<std::string>();
  if (!cam3_config.empty() && !cam3_data.empty()) {
    Eigen::Matrix3d K;
    std::array<double, 4> d;
    CameraPose extrinsic;
    read_calib(cam3_config, K, d, extrinsic);
    Ks.push_back(K);
    ds.push_back(d);
    extrinsics.push_back(extrinsic);
    // std::vector<std::tuple<bool, Eigen::Vector2d>> tmp_vec =
    //     read_points(cam3_data);
    // data_points_.push_back({tmp_vec.begin() + offset, tmp_vec.end()});
    data_points_.push_back(read_points(cam3_data));
  }

  auto cam4_config = vm["cam4_config"].as<std::string>();
  auto cam4_data = vm["cam4_data"].as<std::string>();
  if (!cam4_config.empty() && !cam4_data.empty()) {
    Eigen::Matrix3d K;
    std::array<double, 4> d;
    CameraPose extrinsic;
    read_calib(cam4_config, K, d, extrinsic);
    Ks.push_back(K);
    ds.push_back(d);
    extrinsics.push_back(extrinsic);
    // std::vector<std::tuple<bool, Eigen::Vector2d>> tmp_vec =
    //     read_points(cam4_data);
    // data_points_.push_back({tmp_vec.begin() + offset, tmp_vec.end()});
    data_points_.push_back(read_points(cam4_data));
  }

  auto cam5_config = vm["cam5_config"].as<std::string>();
  auto cam5_data = vm["cam5_data"].as<std::string>();
  if (!cam5_config.empty() && !cam5_data.empty()) {
    Eigen::Matrix3d K;
    std::array<double, 4> d;
    CameraPose extrinsic;
    read_calib(cam5_config, K, d, extrinsic);
    Ks.push_back(K);
    ds.push_back(d);
    extrinsics.push_back(extrinsic);
    // std::vector<std::tuple<bool, Eigen::Vector2d>> tmp_vec =
    //     read_points(cam4_data);
    // data_points_.push_back({tmp_vec.begin() + offset, tmp_vec.end()});
    data_points_.push_back(read_points(cam5_data));
  }

  auto cam6_config = vm["cam6_config"].as<std::string>();
  auto cam6_data = vm["cam6_data"].as<std::string>();
  if (!cam6_config.empty() && !cam6_data.empty()) {
    Eigen::Matrix3d K;
    std::array<double, 4> d;
    CameraPose extrinsic;
    read_calib(cam6_config, K, d, extrinsic);
    Ks.push_back(K);
    ds.push_back(d);
    extrinsics.push_back(extrinsic);
    // std::vector<std::tuple<bool, Eigen::Vector2d>> tmp_vec =
    //     read_points(cam4_data);
    // data_points_.push_back({tmp_vec.begin() + offset, tmp_vec.end()});
    data_points_.push_back(read_points(cam6_data));
  }

  std::vector<std::tuple<bool, Eigen::Vector3d>> points3d;
  utils::calculate3dPoints(points3d, Ks, ds, extrinsics, data_points_, 0, 1);
  // std::cout << std::get<1>(points3d.at(0)) << std::endl;
  // std::cerr << "test" << std::endl;
  std::cout << "calculate3dPoints done" << std::endl;
  std::vector<double> errors =
      getReprojectionError(points3d, Ks, ds, extrinsics, data_points_);
  std::cout << "Reprojection error before bundle adjustment" << std::endl;
  for (std::size_t cam = 0; cam < Ks.size(); cam++) {
    std::cout << errors.at(cam) << std::endl;
  }
  BundleAdjustment ba(options);
  ba.optimize(ds, Ks, extrinsics, points3d, data_points_);
  // utils::calculate3dPoints(points3d_, K_, distortions_, extrinsics_,
  //                          data_points_, 0, 1);

  errors = getReprojectionError(points3d, Ks, ds, extrinsics, data_points_);
  std::cout << "Reprojection error after bundle adjustment" << std::endl;
  for (std::size_t cam = 0; cam < Ks.size(); cam++) {
    std::cout << errors.at(cam) << std::endl;
  }

  // exportToYaml(errors, "after_ba_");
}
