#include <wand_calibration/WandCalibration.h>

#include <boost/program_options.hpp>

int main(int argc, char **argv) {
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
      "represent no limit)")("self_calibration",
                             boost::program_options::bool_switch(),
                             "Perform self-calibration")(
      "fix_distortion", boost::program_options::bool_switch(),
      "Keep distortions fixed")("fix_cammatrix",
                                boost::program_options::bool_switch(),
                                "Keep camera matrix fixed")(
      "fix_extrinsics", boost::program_options::bool_switch(),
      "Keep extrinsics fixed")("extrinsics_gt",
                               boost::program_options::bool_switch(),
                               "Use ground truth extrinsics")(
      "ignore_wand", boost::program_options::bool_switch(),
      "Ignore wand constraint")(
      "wc_distance_scaling", boost::program_options::value<double>()->default_value(1.0),
      "Scaling factor for the wand distance constraint in the bundle adjustement.")(
      "wc_linearity_scaling", boost::program_options::value<double>()->default_value(1.0),
      "Scaling factor for the wand linearity constraint in the bundle adjustement.");

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
  auto self_calibration = vm["self_calibration"].as<bool>();
  if (self_calibration) {
    std::cout << "We will perform self-calibration" << std::endl;
  } else {
    std::cout << "Self-calibration is not performed" << std::endl;
  }
  auto fix_distortion = vm["fix_distortion"].as<bool>();
  if (fix_distortion) {
    std::cout << "The distortions will not be optimized" << std::endl;
  } else {
    std::cout << "The distortions will be optimized" << std::endl;
  }
  auto fix_cammatrix = vm["fix_cammatrix"].as<bool>();
  if (fix_cammatrix) {
    std::cout << "The camera matrices will not be optimized" << std::endl;
  } else {
    std::cout << "The camera matrices will be optimized" << std::endl;
  }
  auto fix_extrinsics = vm["fix_extrinsics"].as<bool>();
  if (fix_extrinsics) {
    std::cout << "The extrinsics will not be optimized" << std::endl;
  } else {
    std::cout << "The extrinsics will be optimized" << std::endl;
  }
  auto extrinsics_gt = vm["extrinsics_gt"].as<bool>();
  if (extrinsics_gt) {
    std::cout << "The ground truth extrinsics will be used" << std::endl;
  } else {
    std::cout << "The ground truth extrinsics will not be used" << std::endl;
  }
  auto ignore_wand = vm["ignore_wand"].as<bool>();
  if (ignore_wand) {
    std::cout << "The wand is not used as a constraint in the BA" << std::endl;
  } else {
    std::cout << "The wand is used as a constraint in the BA" << std::endl;
  }
  auto wc_distance_scaling = vm["wc_distance_scaling"].as<double>();
  if (wc_distance_scaling) {
    std::cout << "Wand constraint distance scaling factor " << wc_distance_scaling << std::endl;
  }
  auto wc_linearity_scaling = vm["wc_linearity_scaling"].as<double>();
  if (wc_linearity_scaling) {
    std::cout << "Wand constraint linearity scaling factor " << wc_linearity_scaling << std::endl;
  }
  OptimizationOptions options(nr_observations, fix_distortion, fix_cammatrix,
                              fix_extrinsics, extrinsics_gt, ignore_wand,
                              wc_distance_scaling, wc_linearity_scaling);
  WandCalibration wand_calibration(options, self_calibration);

  auto cam1_config = vm["cam1_config"].as<std::string>();
  auto cam1_data = vm["cam1_data"].as<std::string>();
  if (!cam1_config.empty() && !cam1_data.empty()) {
    wand_calibration.addCameraData(cam1_data, cam1_config);
  }

  auto cam2_config = vm["cam2_config"].as<std::string>();
  auto cam2_data = vm["cam2_data"].as<std::string>();
  if (!cam2_config.empty() && !cam2_data.empty()) {
    wand_calibration.addCameraData(cam2_data, cam2_config);
  }

  auto cam3_config = vm["cam3_config"].as<std::string>();
  auto cam3_data = vm["cam3_data"].as<std::string>();
  if (!cam3_config.empty() && !cam3_data.empty()) {
    wand_calibration.addCameraData(cam3_data, cam3_config);
  }

  auto cam4_config = vm["cam4_config"].as<std::string>();
  auto cam4_data = vm["cam4_data"].as<std::string>();
  if (!cam4_config.empty() && !cam4_data.empty()) {
    wand_calibration.addCameraData(cam4_data, cam4_config);
  }

  auto cam5_config = vm["cam5_config"].as<std::string>();
  auto cam5_data = vm["cam5_data"].as<std::string>();
  if (!cam5_config.empty() && !cam5_data.empty()) {
    wand_calibration.addCameraData(cam5_data, cam5_config);
  }

  auto cam6_config = vm["cam6_config"].as<std::string>();
  auto cam6_data = vm["cam6_data"].as<std::string>();
  if (!cam6_config.empty() && !cam6_data.empty()) {
    wand_calibration.addCameraData(cam6_data, cam6_config);
  }
  wand_calibration.calibrate();
}
