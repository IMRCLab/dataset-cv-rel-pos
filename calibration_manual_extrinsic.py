import numpy as np
import argparse
import yaml
import cv2
from pathlib import Path
from parameters import params
import os
import shutil
import math
import rowan
import utils


def compute_calibration(calibration_yaml, rvec = None, tvec = None):

    with open(calibration_yaml) as f:
                calib = yaml.safe_load(f)
    mtx = np.array(calib['camera_matrix'])
    dist = np.array(calib['dist_coeff'])
    print(dist, mtx)

    if rvec is None:
        # # forward facing (cf7)
        # tvec = [0.0, 0.0, -0.025] # m
        # rvec = [92, 0, 88]          # deg (up/down, ?, left/right)

        # # forward facing (cf8)
        # tvec = [0.0, 0.0, -0.025] # m
        # rvec = [48, 0, 84]          # deg (up/down, ?, left/right)

        # forward facing (cf6)
        tvec = [0.0, 0.0, -0.025] # m
        rvec = [5, 0, 90]          # deg (up/down, ?, left/right)

    tvec = np.array(tvec)
    rvec = utils.quat2opencv(rowan.from_euler(np.radians(rvec[0]),np.radians(rvec[1]),np.radians(rvec[2]), convention="xyz"))

    print('Euler angles (in deg.) for robot in camera frame:')
    print(np.degrees(rowan.to_euler(utils.opencv2quat(rvec), convention="xyz")))
    print('Position for robot in camera frame:')
    print(tvec)


    data = {'camera_matrix': np.asarray(mtx).tolist(), # save calibration results
            'dist_coeff': np.asarray(dist).squeeze().tolist(),
            'tvec': np.asarray(tvec).squeeze().tolist(),
            'rvec': np.asarray(rvec).squeeze().tolist()}
    with open(calibration_yaml, "w") as f:
        yaml.dump(data, f)

# offline calibration and reprojecting corners back to image with computed parameters
# python3 offline_calibration.py PATH-TO-YAML-FILE 
def main():
    parser = argparse.ArgumentParser(description='Get yaml file with object and image points')
    parser.add_argument('calibration_yaml_file') 

    args = parser.parse_args()
    compute_calibration(args.calibration_yaml_file)

if __name__ == "__main__":
    main()

