#include <wand_calibration/SelfCalibration.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace self_calibration {
Eigen::Matrix3d normalizeNorm(Eigen::Matrix3d A) { return A / A.norm(); }

Eigen::Matrix3d stabilize(Eigen::Matrix3d x, double tol) {
  Eigen::Matrix3d xs;
  for (std::size_t i = 0; i < x.rows(); i++) {
    for (std::size_t j = 0; j < x.cols(); j++) {
      if (std::abs(x(i, j)) > tol) {
        xs(i, j) = x(i, j);
      } else {
        xs(i, j) = 0;
      }
    }
  }
  return xs;
}

std::tuple<Eigen::Matrix3d, Eigen::Matrix3d> selfCalibrate(Eigen::Matrix3d F,
                                                           int w, int h) {
  // Compute the semi-calibrated fundamental matrix
  Eigen::Matrix3d K;
  K << 2.0 * (w + h), 0.0, w / 2.0, 0.0, 2.0 * (w + h), h / 2.0, 0.0, 0.0, 1.0;
  Eigen::Matrix3d G = normalizeNorm(K.transpose() * F * K);

  // Self-calibration using the Kruppa equations (Sturm's method)
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  svd.computeU();
  svd.computeV();

  Eigen::Vector3d s = svd.singularValues();
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d Vh = svd.matrixV();

  Eigen::Vector3d fp;
  fp << std::pow(s(2), 2) * std::pow(U(2, 0), 2) * std::pow(Vh(0, 2), 2) -
            std::pow(s(1), 2) * std::pow(U(2, 1), 2) * std::pow(Vh(1, 2), 2),
      std::pow(s(0), 2) * (std::pow(U(2, 0), 2) + std::pow(Vh(0, 2), 2) -
                           2 * std::pow(U(2, 0), 2) * std::pow(Vh(2, 0), 2)) -
          std::pow(s(1), 2) *
              (std::pow(U(2, 1), 2),
               +std::pow(Vh(1, 2), 2) -
                   2 * std::pow(U(2, 1), 2) * std::pow(Vh(1, 2), 2)),
      std::pow(s(0), 2) * (1 - std::pow(U(2, 0), 2)) *
              (1 - std::pow(Vh(0, 2), 2)) -
          std::pow(s(1), 2) * (2 - std::pow(U(2, 1), 2)) *
              (1 - std::pow(Vh(1, 2), 2));

  cv::Mat cv_fp;
  cv::eigen2cv(fp, cv_fp);
  cv::Mat rs;
  cv::solvePoly(cv_fp, rs);
  std::vector<double> rs_2;
  for (std::size_t i = 0; i < rs.rows; i++) {
    if (std::abs(rs.at<double>(i, 1)) < 1e-6 && rs.at<double>(i, 0) > 0) {
      rs_2.emplace_back(rs.at<double>(i, 0));
    }
  }

  double f = 2 * (w + h);
  for (size_t i = 0; i < rs_2.size(); i++) {
    if (rs_2.at(i) > 1e-6) {
      f = 2 * (w + h) * std::sqrt(rs_2[0]);
    }
  }

  K << f, 0, w / 2.0, 0, f, h / 2.0, 0, 0, 1;
  Eigen::Matrix3d E;
  E = K.transpose() * F * K;

  return {stabilize(normalizeNorm(E)), K};
}
}  // namespace self_calibration
