"""
To extract the ball positions from the wand in frame images
"""

import csv
from logging import raiseExceptions
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import norm
from tqdm import tqdm

# Color thresholds for the color mask
HSV_LOW = (0, 100, 50)
HSV_HIGH = (40, 255, 255)


def get_wand_balls(img):
    # plt.imshow(img)
    # plt.show()
    # Convert to HSV
    hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # plt.imshow(hsv_img)
    # plt.show()

    # Color segmentation
    mask = cv2.inRange(hsv_img, HSV_LOW, HSV_HIGH)
    # plt.imshow(mask)
    # plt.show()

    # Blob detection
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
    # params.minThreshold = 10
    # params.maxThreshold = 200
    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 1000
    params.filterByCircularity = True
    params.minCircularity = 0.3
    params.filterByConvexity = True
    params.minConvexity = 0.5
    params.filterByInertia = False
    params.minInertiaRatio = 0.7
    detector = cv2.SimpleBlobDetector_create(params)

    # Identify dots
    kps = detector.detect(mask)
    # print(len(kps))
    if len(kps) != 3:
        im_with_keypoints = cv2.drawKeypoints(
            cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB),
            kps,
            np.array([]),
            (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )
        # plt.imshow(im_with_keypoints)
        # plt.show()
        debug_img = im_with_keypoints
        pts = None
    else:
        pts = np.array([key_point.pt for key_point in kps]).reshape(-1, 1, 2)

        pts, idx_max, idx_min = sort_balls(pts)
        img = cv2.circle(img, pts[0].astype(int), 12, (255, 0, 0), 3)
        img = cv2.circle(img, pts[1].astype(int), 12, (0, 255, 0), 3)
        img = cv2.circle(img, pts[2].astype(int), 12, (0, 0, 255), 3)
        img = cv2.putText(
            img,
            str(idx_max),
            (10, 500),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
        )
        img = cv2.putText(
            img,
            str(idx_min),
            (10, 550),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
        )
        debug_img = img
        # plt.imshow(img)
        # plt.show()
    return pts, debug_img


def sort_balls(balls):
    assert len(balls) == 3
    sorted_balls = np.zeros((3, 2))
    d = []
    for i in range(3):
        for j in range(i + 1, 3):
            dist = norm(balls[i] - balls[j])
            d.append(dist)
    idx_min = np.argmin(d)
    idx_max = np.argmax(d)
    if idx_max == 0:
        sorted_balls[1] = balls[2]
        if idx_min == 1:
            sorted_balls[0] = balls[0]
            sorted_balls[2] = balls[1]
        else:
            sorted_balls[0] = balls[1]
            sorted_balls[2] = balls[0]

    elif idx_max == 1:
        sorted_balls[1] = balls[1]
        if idx_min == 0:
            sorted_balls[0] = balls[0]
            sorted_balls[2] = balls[2]
        else:
            sorted_balls[0] = balls[2]
            sorted_balls[2] = balls[0]
    elif idx_max == 2:
        sorted_balls[1] = balls[0]
        if idx_min == 0:
            sorted_balls[0] = balls[1]
            sorted_balls[2] = balls[2]
        else:
            sorted_balls[0] = balls[2]
            sorted_balls[2] = balls[1]
    else:
        print("Error")
    return sorted_balls, idx_max, idx_min


def get_timestamp(path):
    name = path.split("/")[-1]
    time = name.split("-")[-1][:-4]
    return int(time)


if __name__ == "__main__":
    root_dir = Path(
        "/cshome/share/TT_robot/camera_calibration/2023_05_12/calib_0/frame_3"
    )
    img_paths = root_dir.glob("*.png")
    img_paths = list(img_paths)
    img_paths = sorted(img_paths)
    csv_path = "wand.csv"
    n = len(img_paths)
    print("tot images: {}".format(n))
    not_detect = 0
    print("writing debug images to: {}".format(str(root_dir / "debug_imgs")))
    with open(str(root_dir / csv_path), "w") as f:
        writer = csv.writer(f, delimiter=";")
        for path in tqdm(img_paths):
            time = get_timestamp(str(path))
            img = cv2.imread(str(path))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            balls, debug_img = get_wand_balls(img)
            cv2.imwrite(
                str(root_dir / "debug_imgs" / str(path).split("/")[-1]),
                cv2.cvtColor(debug_img, cv2.COLOR_RGB2BGR),
            )
            if balls is None:
                writer.writerow([time, -1, -1, -1, -1, -1, -1])
                not_detect += 1
            else:
                writer.writerow(
                    [
                        time,
                        balls[0][0],
                        balls[0][1],
                        balls[1][0],
                        balls[1][1],
                        balls[2][0],
                        balls[2][1],
                    ]
                )
    print("not detected: {}".format(not_detect / n))
