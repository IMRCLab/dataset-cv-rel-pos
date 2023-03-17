import numpy as np
import argparse
import yaml
import cv2
from pathlib import Path
from parameters import params
import matplotlib.pyplot as plt

import os
import shutil
import math
import rowan
import utils


def compute_calibration(img_obj_points_yaml, calibration_yaml, filter=int(1e6), dist_mode='no-k23', percentile=90):

    parameters = params()
    
    with open(img_obj_points_yaml, 'r') as stream:
        points = yaml.safe_load(stream)

    print("Have {} files". format(len(points)))

    filterlist = [
    ]

    for _ in range(2):

        # filter to specified number of frames
        points_filtered = dict()
        for k, file in enumerate(points):
            if file not in filterlist:
                points_filtered[file] = points[file]
            if k >= filter - 1:
                break
        points = points_filtered
        print("Using {} files".format(len(points)))

        imgpoints, objpoints, cf_poses, checkerboard_poses, files = [], [], [], [], []
        for k, file in enumerate(points):
            imgpoints.append(np.array(points[file]['imgpoints'], dtype=np.float32)) # in image frame
            objpoints.append(np.array(points[file]['objpoints'], dtype=np.float32)) # in checkerboard frame
            files.append(file)

        dist = np.zeros(5)
        if dist_mode == "all":
            flags = 0
        elif dist_mode == "no-k3":
            flags = cv2.CALIB_FIX_K3
        elif dist_mode == "no-k23":
            flags = cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
        elif dist_mode == "no-k":
            flags = cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
        elif dist_mode == "none":
            flags = cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST
        else:
            print("ERROR: unknown dist-mode", dist_mode)
            exit()
        ret, mtx, dist, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors = cv2.calibrateCameraExtended(objpoints, imgpoints, parameters.img_shape, None, dist, None, None, None, None, None, flags)

        print("Error: ", ret)
        limit = np.percentile(perViewErrors.flatten(), percentile)
        print("use limit: ", limit, " (percentile ", percentile, ")")
        filterlist = np.array(files)[perViewErrors.flatten() > limit].tolist()


    # print(stdDeviationsExtrinsics.shape)
    # exit()

    # extract distortion coefficients
    k1, k2, p1, p2, k3 = dist
    # check distortion result, see https://github.com/opencv/opencv/issues/15992
    if (k1 >= 0 and k2 >= 0 and k3 >= 0) or (k1 <= 0 and k2 <= 0 and k3 <= 0):
        print("Distortion seems fine")
    else:
        print("ERROR: bad distortion estimation!", k1, k2, k3)
        exit()

    print("Error: ", ret)


    # show imgpoints heatmap
    all_imgpoints = np.vstack(imgpoints)
    fig, axs = plt.subplots(1, 1)
    axs.hist2d(all_imgpoints[:,0], all_imgpoints[:,1], bins=(50,50))
    plt.show()

    data = {'camera_matrix': np.asarray(mtx).tolist(), # save calibration results
            'dist_coeff': np.asarray(dist).squeeze().tolist(),
            'tvec': np.zeros(3).tolist(),
            'rvec': np.zeros(3).tolist()}
    with open(calibration_yaml, "w") as f:
        yaml.dump(data, f)

def main():
    parser = argparse.ArgumentParser(description='Get yaml file with object and image points')
    parser.add_argument('yaml_file') 
    parser.add_argument('--filter', default=1e6, type=int)
    parser.add_argument('--dist', default='no-k23', choices=['none', 'no-k', 'no-k23', 'no-k3', 'all'])
    parser.add_argument('--percentile', default=90, type=int)

    args = parser.parse_args()
    yaml_file = args.yaml_file
    folder = Path(yaml_file).parent
    compute_calibration(yaml_file, folder / "calibration.yaml", args.filter, args.dist, args.percentile)

if __name__ == "__main__":
    main()

