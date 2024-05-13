#include <wand_calibration/Utils.h>

#include <numeric>
#include <opencv2/core/eigen.hpp>

namespace utils {
void filterPoints(const std::vector<std::tuple<bool, Eigen::Vector3d>>& raw_pts,
                  std::vector<Eigen::Vector3d>& f_pts) {
  for (size_t point_idx = 0; point_idx < raw_pts.size(); point_idx++) {
    // Only add if there is a valid observation
    if (std::get<0>(raw_pts.at(point_idx))) {
      f_pts.emplace_back(Eigen::Vector3d(std::get<1>(raw_pts.at(point_idx))));
    }
  }
}

void filterPoints(const std::vector<std::tuple<bool, Eigen::Vector3d>>& raw_pts,
                  std::vector<Eigen::Vector3d>& f_pts,
                  std::vector<int>& inliers) {
  for (size_t point_idx = 0; point_idx < raw_pts.size(); point_idx++) {
    // Only add if there is a valid observation
    if (point_idx % 3 == 2 && std::get<0>(raw_pts.at(point_idx - 2)) &&
        std::get<0>(raw_pts.at(point_idx - 1)) &&
        std::get<0>(raw_pts.at(point_idx))) {
      // double d = (std::get<1>(points3d_.at(point_idx - 2)) -
      //             std::get<1>(points3d_.at(point_idx - 1)))
      //                .norm();
      // std::cout << d << std::endl;
      // double d = (std::get<1>(points3d_.at(point_idx - 1)) -
      //             std::get<1>(points3d_.at(point_idx)))
      //                .norm();
      // std::cout << d << std::endl;
      inliers.push_back(point_idx);
      f_pts.emplace_back(Eigen::Vector3d(std::get<1>(raw_pts.at(point_idx))));
    }
  }
}

void filterPoints(
    const std::vector<std::tuple<bool, Eigen::Vector2d>>& raw_pts1,
    const std::vector<std::tuple<bool, Eigen::Vector2d>>& raw_pts2,
    std::vector<cv::Point2d>& f_pts1, std::vector<cv::Point2d>& f_pts2) {
  // TODO: iteration should be min number of points
  for (size_t point_idx = 0; point_idx < raw_pts2.size(); point_idx++) {
    // Only add if there is a valid observation
    if ((std::get<0>(raw_pts1.at(point_idx))) &&
        (std::get<0>(raw_pts2.at(point_idx)))) {
      // extract Point and convert from eigen to OpenCV
      cv::Point2d p1(std::get<1>(raw_pts1.at(point_idx))(0),
                     std::get<1>(raw_pts1.at(point_idx))(1));
      cv::Point2d p2(std::get<1>(raw_pts2.at(point_idx))(0),
                     std::get<1>(raw_pts2.at(point_idx))(1));
      f_pts1.emplace_back(p1);
      f_pts2.emplace_back(p2);
    }
  }
}
Eigen::Matrix3d orthogonalize_rotation(const Eigen::Matrix3d& R) {
  // Perform Singular Value Decomposition on R
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  // Construct an orthogonal rotation matrix from the SVD result
  Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
  if (U.determinant() * V.determinant() < 0) {
    S(2, 2) = -1;
  }
  Eigen::Matrix3d R_ortho = U * S * V.transpose();

  return R_ortho;
}
void undistortPoint(const Eigen::Vector2d& dist_point,
                    Eigen::Vector2d& undist_point,
                    const std::array<double, 4>& d, const Eigen::Matrix3d& K) {
  std::vector<cv::Point2d> cv_dist_point = {{dist_point(0), dist_point(1)}};
  std::vector<cv::Point2d> cv_undist_point = {{dist_point(0), dist_point(1)}};
  cv::Mat cv_K;
  cv::eigen2cv(K, cv_K);
  cv::undistortPoints(cv_dist_point, cv_undist_point, cv_K, d);
  undist_point(0) = K(0, 0) * cv_undist_point.at(0).x + K(0, 2);
  undist_point(1) = K(1, 1) * cv_undist_point.at(0).y + K(1, 2);
}

void distortPoint(const Eigen::Vector2d& undist_point,
                  Eigen::Vector2d& dist_point, const std::array<double, 4>& d,
                  const Eigen::Matrix3d& K) {
  // Converting (u, v) to (x, y)
  double x = (undist_point(0) - K(0, 2)) / K(0, 0);
  double y = (undist_point(1) - K(1, 2)) / K(1, 1);
  double r_2 = x * x + y * y;  // r squared

  // Applying distortion
  x = x * (1 + d[0] * r_2 + d[1] * r_2 * r_2) + 2 * d[2] * x * y +
      d[3] * (r_2 + 2 * x * x);
  y = y * (1 + d[0] * r_2 + d[1] * r_2 * r_2) + d[2] * (r_2 + 2 * y * y) +
      2 * d[3] * x * y;

  // Converting back to pixel coordinates
  dist_point(0) = x * K(0, 0) + K(0, 2);
  dist_point(1) = y * K(1, 1) + K(1, 2);
}

// Takes as input undistorted Points
void calculate3dPoint(Eigen::Vector3d& triang_pt, const Eigen::Vector2d& pt1,
                      const Eigen::Vector2d& pt2,
                      const Eigen::Matrix<double, 3, 4>& P1,
                      const Eigen::Matrix<double, 3, 4>& P2) {
  // Convert variables to values accepted by opencv
  cv::Mat point4D(4, 1, CV_64FC1);
  cv::Mat point3D(3, 1, CV_64FC1);
  cv::Mat projPoints1(2, 1, CV_64FC1);
  projPoints1.at<double>(0, 0) = pt1(0);
  projPoints1.at<double>(1, 0) = pt1(1);

  cv::Mat projPoints2(2, 1, CV_64FC1);
  projPoints2.at<double>(0, 0) = pt2(0);
  projPoints2.at<double>(1, 0) = pt2(1);
  // std::cerr << projMat1 << std::endl;
  // std::cerr << projMat2 << std::endl;

  cv::Mat cv_P1;
  cv::Mat cv_P2;
  cv::eigen2cv(P1, cv_P1);
  cv::eigen2cv(P2, cv_P2);
  cv::triangulatePoints(cv_P1, cv_P2, projPoints1, projPoints2, point4D);
  // std::cout << point4D << std::endl;
  // std::cerr << projPoints1 << std::endl;
  // std::cerr << projPoints2 << std::endl << std::endl;

  // Converting
  point3D = point4D.rowRange(0, 3).clone() / point4D.at<double>(3, 0);
  triang_pt(0) = point3D.at<double>(0, 0);
  triang_pt(1) = point3D.at<double>(1, 0);
  triang_pt(2) = point3D.at<double>(2, 0);
}

bool valid3dPose(Eigen::Vector3d& pose) {
  if (pose(2) < 0) {
    return false;
  } else {
    return true;
  }
}

double getWandDistanceError(std::vector<Eigen::Vector3d>& pts) {
  // Distance between each ball
  double d1_ref = 0.13;
  double d2_ref = 0.26;
  // std::cout << (pts.size() % 3 == 0) << std::endl;
  int n_wand = pts.size() / 3;  // Number of wands observed
  double error = 0;
  std::vector<double> d1;
  std::vector<double> d2;
  for (size_t i = 0; i < n_wand; i++) {
    Eigen::Vector3d delta = pts.at(3 * i) - pts.at(3 * i + 1);
    d1.push_back(delta.norm());
    delta = pts.at(3 * i + 1) - pts.at(3 * i + 2);
    d2.push_back(delta.norm());
    // error += (d1 - d1_ref) * (d1 - d1_ref) + (d2 - d2_ref) * (d2 - d2_ref);
    // std::cout << d1.back() << std::endl;
    // std::cout << d2.back() << std::endl;
  }
  double mean_d1 = std::accumulate(d1.begin(), d1.end(), 0.) / d1.size();
  // std::cout << "mean d1: " << mean_d1 << std::endl;
  double mean_d2 = std::accumulate(d2.begin(), d2.end(), 0.) / d2.size();
  // std::cout << "mean d2: " << mean_d2 << std::endl;
  return d1_ref - mean_d1 + d2_ref - mean_d2;
}

Eigen::Matrix<double, 3, 4> getProjectionMat(
    const std::vector<Eigen::Matrix3d>& K,
    const std::vector<CameraPose>& extrinsics, const int cam_i) {
  Eigen::Matrix<double, 3, 4> projection_matrix;
  projection_matrix = K.at(cam_i) * extrinsics.at(cam_i).pose.matrix3x4();
  // std::cout << projection_matrix << std::endl;
  return projection_matrix;
}

// Takes as input distorted points
void calculate3dPoints(
    std::vector<std::tuple<bool, Eigen::Vector3d>>& points3d,
    const std::vector<Eigen::Matrix3d>& K,
    const std::vector<std::array<double, 4>>& d,
    const std::vector<CameraPose>& extrinsics,
    const std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>>&
        data_points,
    int cam1, int cam2) {
  points3d.clear();
  // Reference values for test
  // Eigen::Matrix<double, 3, 4> P1;
  // P1 << 1255.382, 0, 652.6714, 0, 0, 1253.625, 506.8444, 0, 0, 0, 1, 0;
  // Eigen::Matrix<double, 3, 4> P2;
  // P2 << 1069.089, 828.6367, -410.1622, 4306178, -240.7342, 1093.284,
  // 775.4156,
  //     -638541.9, 0.728247, 0.1611319, 0.6661026, 1574.116;

  // Use cam1 and cam2 to triangulate points
  Eigen::Matrix<double, 3, 4> P1 = getProjectionMat(K, extrinsics, cam1);
  Eigen::Matrix<double, 3, 4> P2 = getProjectionMat(K, extrinsics, cam2);

  std::vector<std::size_t> nr_data_points;
  for (std::size_t cam_i = 0; cam_i < K.size(); ++cam_i) {
    nr_data_points.emplace_back(data_points.at(cam_i).size());
  }
  std::size_t min_nr_data_points =
      *std::min_element(nr_data_points.begin(), nr_data_points.end());
  // std::vector<std::vector<std::tuple<bool, Eigen::Vector2i>>> data_points_;
  for (std::size_t point_i = 0; point_i < min_nr_data_points; ++point_i) {
    bool always_visible = true;
    // Check if the wand is visible from all cameras
    for (std::size_t cam_i = 0; cam_i < K.size(); ++cam_i) {
      if (!std::get<0>(data_points.at(cam_i).at(point_i))) {
        always_visible = false;
      }
    }

    if (always_visible) {
      Eigen::Vector3d point_3d;
      // Undistort the observed points
      Eigen::Vector2d cam1_undist_pt, cam2_undist_pt;
      utils::undistortPoint(std::get<1>(data_points.at(cam1).at(point_i)),
                            cam1_undist_pt, d.at(cam1), K.at(cam1));
      utils::undistortPoint(std::get<1>(data_points.at(cam2).at(point_i)),
                            cam2_undist_pt, d.at(cam2), K.at(cam2));
      utils::calculate3dPoint(point_3d, cam1_undist_pt, cam2_undist_pt, P1, P2);

      // std::cout << point_3d << std::endl << std::endl;
      points3d.emplace_back(std::tuple<bool, Eigen::Vector3d>(true, point_3d));
    } else {
      Eigen::Vector3d point_3d{-1, -1, -1};
      points3d.emplace_back(std::tuple<bool, Eigen::Vector3d>(false, point_3d));
    }
  }
}
}  // namespace utils
