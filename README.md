# eWand

This paper contains the code for the paper [eWand: A calibration framework for wide baseline frame-based and event-based camera systems](https://cogsys-tuebingen.github.io/ewand/).

#### Citing

If you use this code in an academic context, please cite the following publication:

Paper: [eWand: A calibration framework for wide baseline frame-based and event-based camera systems](https://arxiv.org/pdf/2309.12685.pdf)

Video: [YouTube](https://youtu.be/Yd_Bsltdfi4)

Project: [See here](https://cogsys-tuebingen.github.io/ewand/)

```
@inproceedings{Gossard2024icra,
  url = {https://arxiv.org/pdf/2309.12685.pdf},
  title={eWand: A calibration framework for wide baseline frame-based and event-based camera systems},
  author={Gossard, Thomas and Ziegler, Andreas and Kolmar, Levin and Tebbe, Jonas and Zell, Andreas},
  year={2024},
  booktitle = {2024 {International} {Conference} on {Robotics} and {Automation} ({ICRA})},
  publisher = {IEEE},
}
```

## Using the code

The code of this project is split into several packages:

* [Marker extrection from frame-based cameras](https://github.com/cogsys-tuebingen/ewand?tab=readme-ov-file#marker-extrection-from-frame-based-cameras)
* [Marker extrection from event-based cameras](https://github.com/cogsys-tuebingen/eb_marker_extraction)
* [A calibration framework consuming the extracted marker positions and optimizing the cameras' intrinsics and extrinsics](https://github.com/cogsys-tuebingen/ewand?tab=readme-ov-file#calibration-framework)

### Marker extrection from frame-based cameras


### Calibration framework

#### Software requirements

This code has been tested on Ubuntu 20.04.

Dependencies:
- OpenCV
- Ceres

#### How to build

1. Clone this repositories:
```
git clone https://github.com/cogsys-tuebingen/ewand
```


4. Build:
```
cd calibration_framework
mkdir build
cd build
cmake ..
make
```

5. Setup the intrinsic camera parameters

For every camera you want to calibrate, create a `.yaml` file with the following structure. You can get the intrinsic parameters from the intrinsic calibration tool of your choice, e.g. kalibr, OpenCV.

```
%YAML:1.0
camera_name: cam0
image_width: 1280
image_height: 1024
camera_matrix: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d
  data: [1277.0348869354, 0, 637.71636909691, 0, 1276.78458195841, 488.458326168923, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients: !!opencv-matrix
  rows: 1
  cols: 5
  dt: d
  data: [-0.220061443243291, 0.261537432483716, -0.0025783399940279, 0.00000095452629096082, 0]
```

6. Run:

```
cd buid
./src/wand_calibration_main \
  --cam1_config ../intrinsics/camera_0.yaml \
  --cam1_data ../data/camera_0.csv \
  --cam2_config ../intrinsics/camera_1.yaml \
  --cam2_data ../data/camera_1.csv \
  --cam3_config ../intrinsics/camera_2.yaml \
  --cam3_data ../data/camera_2.csv \
  --cam4_config ../intrinsics/camera_3.yaml \
  --cam4_data ../data/camera_3.csv \
  --cam5_config ../intrinsics/camera_4.yaml \
  --cam5_data ../data/camera_4.csv \
  --cam6_config ../intrinsics/camera_5.yaml \
  --cam6_data ../data/camera_5.csv \
  --wc_distance_scaling 1000 --wc_linearity_scaling 100000
```

where the `yaml` files contain the intrinsics of the cameras and the `csv` files the extracted markers from the frame- and event-based cameras.

**Note:** Use `--help` to see all the options.

## License

This software is issued under the Apache License Version 2.0.
