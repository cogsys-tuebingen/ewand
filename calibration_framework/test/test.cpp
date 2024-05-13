#include <wand_calibration/SelfCalibration.h>
#include <wand_calibration/WandCalibration.h>

#include <Eigen/Eigen>
#include <tuple>

#include "Eigen/src/Core/Matrix.h"
#include "gtest/gtest.h"

namespace {

// Tests loading the intrinsics.
TEST(CalibrationTest, LoadIntrinsicsParams) {
  WandCalibration wand_calibration;
  Eigen::Matrix3d K_cam_1;
  K_cam_1 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  std::array<int, 2> camera_dimensions{1290, 1024};
  std::array<double, 4> distortion_cam_1 = {1, 2, 3, 4};
  wand_calibration.addCameraData("./test/frame_0.csv", camera_dimensions, K_cam_1,
                                 distortion_cam_1);

  // Test for loading yaml
  auto intrinsics = wand_calibration.getK(0);
  auto distortion = wand_calibration.getDistortion(0);

  EXPECT_DOUBLE_EQ(intrinsics(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(intrinsics(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(intrinsics(0, 2), 3.0);
  EXPECT_DOUBLE_EQ(intrinsics(1, 0), 4.0);
  EXPECT_DOUBLE_EQ(intrinsics(1, 1), 5.0);
  EXPECT_DOUBLE_EQ(intrinsics(1, 2), 6.0);
  EXPECT_DOUBLE_EQ(intrinsics(2, 0), 7.0);
  EXPECT_DOUBLE_EQ(intrinsics(2, 1), 8.0);
  EXPECT_DOUBLE_EQ(intrinsics(2, 2), 9.0);

  EXPECT_DOUBLE_EQ(distortion.at(0), 1.0);
  EXPECT_DOUBLE_EQ(distortion.at(1), 2.0);
  EXPECT_DOUBLE_EQ(distortion.at(2), 3.0);
  EXPECT_DOUBLE_EQ(distortion.at(3), 4.0);
}

TEST(CalibrationTest, LoadIntrinsicsFile) {
  WandCalibration wand_calibration;
  wand_calibration.addCameraData("./test/frame_0.csv", "./test/frame_0.yaml");

  // Test for loading yam
  auto intrinsics = wand_calibration.getK(0);
  auto distortion = wand_calibration.getDistortion(0);

  EXPECT_NEAR(intrinsics(0, 0), 1255.383, 1e-3);
  EXPECT_DOUBLE_EQ(intrinsics(0, 1), 0.0);
  EXPECT_NEAR(intrinsics(0, 2), 652.671, 1e-3);
  EXPECT_DOUBLE_EQ(intrinsics(1, 0), 0.0);
  EXPECT_NEAR(intrinsics(1, 1), 1253.625, 1e-3);
  EXPECT_NEAR(intrinsics(1, 2), 506.844, 1e-3);
  EXPECT_DOUBLE_EQ(intrinsics(2, 0), 0.0);
  EXPECT_DOUBLE_EQ(intrinsics(2, 1), 0.0);
  EXPECT_DOUBLE_EQ(intrinsics(2, 2), 1.0);

  EXPECT_NEAR(distortion.at(0), -0.223, 1e-3);
  EXPECT_NEAR(distortion.at(1), 0.243, 1e-3);
  EXPECT_NEAR(distortion.at(2), -0.000, 1e-3);
  EXPECT_NEAR(distortion.at(3), 0.001, 1e-3);
}

TEST(CalibrationTest, LoadData) {
  WandCalibration wand_calibration;
  wand_calibration.addCameraData("./test/frame_0.csv", "./test/frame_0.yaml");

  // Test for loading csv-files
  auto data =
      wand_calibration.getDataPoints().at(0);    // get data points first cam
  EXPECT_EQ(data.size(), 1875);                  // 71 images inside frame0.csv
  EXPECT_EQ(std::get<0>(data.at(0)), true);      // 2 dimensions of each point
  EXPECT_EQ(std::get<1>(data.at(0)).size(), 2);  // 2 dimensions of each point
}

TEST(CalibrationTest, Calibrate) {
  WandCalibration wand_calibration;
  wand_calibration.addCameraData("/data/frame_0.csv", "/data/frame_0.yaml");
  wand_calibration.addCameraData("/data/frame_1.csv", "/data/frame_1.yaml");
  wand_calibration.addCameraData("/data/frame_2.csv", "/data/frame_2.yaml");
  wand_calibration.addCameraData("/data/frame_3.csv", "/data/frame_3.yaml");
  wand_calibration.addCameraData("/data/event_0.csv", "/data/event_0.yaml");
  wand_calibration.addCameraData("/data/event_1.csv", "/data/event_1.yaml");

  // Test calibration
  wand_calibration.calibrate();
}

TEST(CalibrationTest, SelfCalibration) {
  WandCalibration wand_calibration;

  Eigen::Matrix<double, 3, 3> F;
  F << -1.023991058307205e-07, -1.462722764481383e-06, -0.000768455221244857,
      -1.210870253923175e-06, 4.298378067372675e-07, -0.002794759505384437,
      -0.0001817701484103651, 0.004063659791634922, 1.0;
  self_calibration::selfCalibrate(F, 1280, 720);
}

}  // namespace
