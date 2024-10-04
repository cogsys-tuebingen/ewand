#include <wand_calibration/Initialization.h>
#include <wand_calibration/SelfCalibration.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

Initialization::Initialization(
    const std::vector<std::vector<std::tuple<bool, Eigen::Vector2d>>>&
        data_points,
    std::vector<std::tuple<bool, Eigen::Vector3d>>& points3d,
    const bool self_calibration, const bool extrinsics_gt)
    : data_points_{data_points},
      points3d_{points3d},
      self_calibration_{self_calibration},
      extrinsics_gt_{extrinsics_gt} {}

void Initialization::calculateFundamental() {
  // Parameter for the RANSAC 7 points algo
  const double confidence = 0.99;
  const double ransac_reproj_threshold = 2;

  for (std::size_t cam_i = 1; cam_i < data_points_.size(); cam_i++) {
    // Initialize points
    std::vector<cv::Point2d> pts1;
    std::vector<cv::Point2d> pts2;
    utils::filterPoints(data_points_.at(0), data_points_.at(cam_i), pts1, pts2);

    // Calculate fundamental matrix
    // std::cout << "pts1: " << pts1 << std::endl;
    // std::cout << "pts2: " << pts2 << std::endl;
    cv::Mat fundamenta_matrix = cv::findFundamentalMat(
        pts1, pts2, cv::FM_RANSAC, ransac_reproj_threshold, confidence);

    Eigen::Matrix3d fundamenta_matrix_eig;
    // std::cout << "fundamenta_matrix: " << fundamenta_matrix << std::endl;
    cv::cv2eigen(fundamenta_matrix, fundamenta_matrix_eig);
    fundamentalMatrices_.emplace_back(fundamenta_matrix_eig);
  }
}

void Initialization::setK(const std::vector<Eigen::Matrix3d>& K) { K_ = K; }

void Initialization::setDistortion(
    const std::vector<std::array<double, 4>>& d) {
  distortion_ = d;
}

void Initialization::calculateEssentialAndPose() {
  CameraPose camera_pose;
  camera_pose.camera_index = 0;
  camera_pose.pose =
      Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  extrinsics_.emplace_back(camera_pose);

  for (std::size_t cam_i = 1; cam_i < data_points_.size(); cam_i++) {
    // Initialize points
    std::vector<cv::Point2d> pts1;
    std::vector<cv::Point2d> pts2;
    utils::filterPoints(data_points_.at(0), data_points_.at(cam_i), pts1, pts2);
    cv::Mat cv_K_0;
    cv::eigen2cv(K_.at(0), cv_K_0);
    cv::Mat cv_K_1;
    cv::eigen2cv(K_.at(cam_i), cv_K_1);
    // double prob = 0.99;
    // double threshold = 2;
    // cv::Mat mask;
    // cv::Mat cv_E = cv::findEssentialMat(pts1, pts2, cv_K_0, distortion_.at(0), cv_K_1, distortion_.at(cam_i));
    // cv::Mat cv_E = cv::findEssentialMat(pts1, pts2, cv_K, cv::RANSAC, prob,
    //                                     threshold, mask);
    // std::cout << mask << std::endl;

    cv::Mat cv_E;
    cv::Mat cv_R;
    cv::Mat cv_t;
    double prob = 0.99;
    double threshold = 2;
    cv::Mat mask;
    auto inliers = cv::recoverPose(pts1, pts2, cv_K_0, distortion_.at(0), cv_K_1, distortion_.at(cam_i), cv_E, cv_R, cv_t, cv::RANSAC, prob, threshold, mask);
    Eigen::Matrix3d E;
    cv::cv2eigen(cv_E, E);

    // Add the calculated essential matrix
    essentialMatrices_.emplace_back(E);


    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    cv::cv2eigen(cv_t, t);
    cv::cv2eigen(cv_R, R);

    // Construct camera pose for extrinsics
    CameraPose camera_pose;
    camera_pose.camera_index = cam_i;
    camera_pose.pose = Sophus::SE3d(R, t);
    extrinsics_.emplace_back(camera_pose);

    // std::cout << "fundamenta_matrix:\n" << fundamenta_matrix << std::endl;
    // std::cout << "fundamenta_matrix_eig:\n" << fundamenta_matrix_eig <<
    // std::endl; std::cout << "R:\n" << R << std::endl; std::cout << "t:\n" <<
    // t << std::endl;
  }

  // Set extrinsics to known values for debugging
  if (extrinsics_gt_) {
    extrinsics_.clear();
    std::cout << "Static extrinsics:" << std::endl;
    Eigen::Matrix3d R;
    R << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Vector3d t;
    t << 0.0, 0.0, 0.0;
    Eigen::Matrix4d M;
    M << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0;

    Sophus::SE3d test(R, t);
    // test.normalize();
    camera_pose.camera_index = 0;
    camera_pose.pose = Sophus::SE3d(R, t);
    extrinsics_.emplace_back(camera_pose);

    std::cout << "Extrinisics 0:\n"
              << extrinsics_.at(0).pose.matrix() << std::endl;

    R << 0.4766171, 0.5751653, -0.6648467, -0.4909552, 0.8014944, 0.3414232,
        0.7292456, 0.1636818, 0.6643863;

    Eigen::AngleAxisd aa(R);    // RotationMatrix to AxisAngle
    R = aa.toRotationMatrix();  // AxisAngle      to RotationMatrix
    t << 2.621904, -1.151462, 1.580514;
    camera_pose.camera_index = 1;
    camera_pose.pose = Sophus::SE3d(R, t);
    camera_pose.pose.normalize();
    extrinsics_.emplace_back(camera_pose);
    // extrinsics_.at(1).pose.translation() = t;
    // extrinsics_.at(1).pose.setRotationMatrix(R);
    std::cout << "Extrinisics 1:\n"
              << extrinsics_.at(1).pose.matrix() << std::endl;

    R << -0.9625506, 0.1055529, -0.2497095, -0.2079396, 0.303545, 0.9298503,
        0.1739465, 0.9469525, -0.2702288;
    aa = Eigen::AngleAxisd(R);
    R = aa.toRotationMatrix();
    t << 0.7531008, -3.782122, 4.505475;
    camera_pose.camera_index = 2;
    camera_pose.pose = Sophus::SE3d(R, t);
    extrinsics_.emplace_back(camera_pose);
    // extrinsics_.at(2).pose.matrix() = M2;
    std::cout << "Extrinisics 2:\n"
              << extrinsics_.at(2).pose.matrix() << std::endl;

    R << -0.389769, -0.4141864, 0.8225143, 0.5937274, 0.5697249, 0.568244,
        -0.7039658, 0.7098331, 0.02385277;
    aa = Eigen::AngleAxisd(R);
    R = aa.toRotationMatrix();
    t << -3.331472, -2.042174, 3.035646;
    camera_pose.camera_index = 3;
    camera_pose.pose = Sophus::SE3d(R, t);
    extrinsics_.emplace_back(camera_pose);
    // extrinsics_.at(3).pose.matrix() = M3;
    std::cout << "Extrinisics 3:\n"
              << extrinsics_.at(3).pose.matrix() << std::endl;

    R << -0.59194903, 0.80325243, -0.06619576, -0.80540455, -0.59262192,
        0.01107998, -0.03032904, 0.05987315, 0.99774514;
    aa = Eigen::AngleAxisd(R);
    R = aa.toRotationMatrix();
    t << 0.16203367, -0.04697936, -0.07383103;
    camera_pose.camera_index = 4;
    camera_pose.pose = Sophus::SE3d(R, t);
    extrinsics_.emplace_back(camera_pose);
    // extrinsics_.at(4).pose.matrix() = M4;
    std::cout << "Extrinisics 4:\n"
              << extrinsics_.at(4).pose.matrix() << std::endl;

    R << 0.12057087, -0.93627593, 0.3299243, 0.68069338, -0.16393746,
        -0.71398952, 0.72257815, 0.31066363, 0.61755075;
    aa = Eigen::AngleAxisd(R);
    R = aa.toRotationMatrix();
    t << -1.01487019, 2.76750612, 1.61925258;
    camera_pose.camera_index = 5;
    camera_pose.pose = Sophus::SE3d(R, t);
    extrinsics_.emplace_back(camera_pose);
    // extrinsics_.at(5).pose.matrix() = M5;
    std::cout << "Extrinisics 5:\n"
              << extrinsics_.at(5).pose.matrix() << std::endl;
  }
}

void Initialization::refineTranslation() {
  // Perform golden search to find the scaling factor of the unit translation
  // vector
  for (std::size_t cam_i = 1; cam_i < K_.size(); cam_i++) {
    std::cout << "cam: " << cam_i << std::endl;
    Eigen::Vector3d init_t = extrinsics_.at(cam_i).pose.translation();
    double a = 0;
    double b = 10;
    double r = (3 - sqrt(5)) / 2;
    double delta_c, delta_d;  // Errors for c and d proposed scaling factors
    delta_c = 1;              // Initialization of the error
    Eigen::Vector3d t_c;
    Eigen::Vector3d t_d;
    while (abs(delta_c) > 0.010) {
      double c = a + r * (b - a);
      t_c = c * init_t;
      double d = b - r * (b - a);
      t_d = d * init_t;
      // std::cout << c << std::endl;
      // std::cout << d << std::endl;

      // Evaluating for c
      extrinsics_.at(cam_i).pose.translation() = t_c;
      utils::calculate3dPoints(points3d_, K_, distortion_, extrinsics_,
                               data_points_, 0, cam_i);
      std::vector<Eigen::Vector3d> pts;
      utils::filterPoints(points3d_, pts);
      // Calculate distance between balls
      delta_c = utils::getWandDistanceError(pts);

      // Evaluating for d
      extrinsics_.at(cam_i).pose.translation() = t_d;
      utils::calculate3dPoints(points3d_, K_, distortion_, extrinsics_,
                               data_points_, 0, cam_i);
      utils::filterPoints(points3d_, pts);
      // Calculate distance between balls
      delta_d = utils::getWandDistanceError(pts);

      if (abs(delta_c) < abs(delta_d)) {
        b = d;
      } else {
        a = c;
      }
    }
    std::cout << "delta: " << delta_c << std::endl;

    // Update extrinsic translation until valid values
    extrinsics_.at(cam_i).pose.translation() = t_c;
  }
  // Get usable points
}

std::tuple<std::vector<Eigen::Matrix3d>, std::vector<CameraPose>>
Initialization::runInitialization(
    const std::vector<std::array<int, 2>>& camera_dimensions) {
  // Find Fundamental Matrix for every camera pair
  calculateFundamental();

  if (self_calibration_) {
    for (std::size_t i = 0; i < fundamentalMatrices_.size(); ++i) {
      auto [E, K] = self_calibration::selfCalibrate(
          fundamentalMatrices_.at(i), camera_dimensions.at(i).at(0),
          camera_dimensions.at(i).at(1));
      std::cout << "camera " << i << ":" << std::endl;
      std::cout << " E:\n" << E << std::endl;
      std::cout << " K:\n" << K << std::endl;
      essentialMatrices_.emplace_back(E);

      if (i == 0) {
        K_.emplace_back(K);
      }

      K_.emplace_back(K);
    }
  }
  calculateEssentialAndPose();

  std::cout << "Extrinsics before refinement:" << std::endl;
  for (const auto& extrinsic : extrinsics_) {
    std::cout << "Camera: " << extrinsic.camera_index << "\n"
              << extrinsic.pose.matrix() << std::endl;
  }
  if (!extrinsics_gt_) {
    refineTranslation();
  }
  std::cout << "Extrinsics after refinement:" << std::endl;
  for (const auto& extrinsic : extrinsics_) {
    std::cout << "Camera: " << extrinsic.camera_index << "\n"
              << extrinsic.pose.matrix() << std::endl;
  }

  return {K_, extrinsics_};
}
