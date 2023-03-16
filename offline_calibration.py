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


# see https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
def quaternion_average(qs, weights):
    # compute Q
    scale = np.sum(weights)
    Q = np.empty((4,len(qs)))
    for i in range(len(qs)):
        # print(qs[i])
        Q[:,i] = weights[i] / scale * qs[i]
    # print(Q)

    vals, vects = np.linalg.eig(Q @ Q.T)
    maxcol = list(vals).index(max(vals))
    largest_eigenvector = vects[:,maxcol]
    # print(largest_eigenvector)
    return largest_eigenvector


def compute_calibration(img_obj_points_yaml, calibration_yaml, mode='wavg', filter=int(1e6), dist_mode='all', percentile=90, extrinsic_only=False):

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
            cf_poses.append(np.array(points[file]['cf_pose'], dtype=np.float32))
            checkerboard_poses.append(np.array(points[file]['checkerboard_pose'], dtype=np.float32))
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

        if extrinsic_only:
            print("Calibrating extrinsic only!")
            with open(calibration_yaml) as f:
                calib = yaml.safe_load(f)
            int_mtrx = np.array(calib['camera_matrix'])
            dist = np.array(calib['dist_coeff'])
            print(dist, int_mtrx)

            flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_FOCAL_LENGTH | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_TANGENT_DIST
        else:
            int_mtrx = None

        ret, mtx, dist, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors = cv2.calibrateCameraExtended(objpoints, imgpoints, parameters.img_shape, int_mtrx, dist, None, None, None, None, None, flags)

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


    # The OpenCV documentation states about rvec/tvec:
    # " In more technical terms, the tuple of the i-th rotation and translation vector performs a change of basis from object coordinate space to camera coordinate space. Due to its duality, this tuple is equivalent to the position of the calibration pattern with respect to the camera coordinate space."
    # i.e., rvec/tvec are T_checkerboard_to_camera

    # convert resulting rvec/tvec to robot/camera frame

    # rvec/tvec are T_checkerboard_in_camera, we want T_robot_in_camera
    # we also have T_checkerboard_in_world and T_robot_in_world
    # T_camera_in_robot = T_camera_in_checkerboard * T_checkerboard_in_world * T_world_in_robot

    rvecs2 = []
    eulers2 = []
    tvecs2 = []
    for k, file in enumerate(points):
        # extract result for this frame
        q = utils.opencv2quat(rvecs[k])
        translation = tvecs[k].flatten()
        T_checkerboard_to_camera = utils.pose2transform(translation, q)
        T_camera_to_checkerboard = np.linalg.inv(T_checkerboard_to_camera)

        # compute other relevant transformations
        T_robot_to_world = utils.pose2transform(cf_poses[k][0:3], cf_poses[k][3:7])
        T_world_to_robot = np.linalg.inv(T_robot_to_world)

        T_checkerboard_to_world = utils.pose2transform(checkerboard_poses[k][0:3], checkerboard_poses[k][3:7])

        T_camera_to_robot = T_world_to_robot @ T_checkerboard_to_world @ T_camera_to_checkerboard
        T_robot_to_camera = np.linalg.inv(T_camera_to_robot)

        translation, q = utils.transform2pose(T_robot_to_camera)
        rvecs2.append(utils.quat2opencv(q))
        tvecs2.append(translation)
        euler = np.degrees(rowan.to_euler(q, convention='xyz'))
        eulers2.append(euler)

        print(file, perViewErrors[k], translation, euler)

    rvecs = None
    tvecs = None

    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(3, 3)
    axs[0,0].hist(np.array(tvecs2)[:,0])
    axs[0,1].hist(np.array(tvecs2)[:,1])
    axs[0,2].hist(np.array(tvecs2)[:,2])

    axs[1,0].hist(np.array(eulers2)[:,0])
    axs[1,1].hist(np.array(eulers2)[:,1])
    axs[1,2].hist(np.array(eulers2)[:,2])

    axs[2,0].hist(perViewErrors)

    plt.show()

    # show imgpoints heatmap
    all_imgpoints = np.vstack(imgpoints)
    fig, axs = plt.subplots(1, 1)
    axs.hist2d(all_imgpoints[:,0], all_imgpoints[:,1], bins=(50,50))
    plt.show()

    if mode == 'min':
        # pick the one with the lowest per-view error
        k = np.argmin(perViewErrors)
        for k2, file in enumerate(points):
            if k == k2:
                print("best file: ", file)
                break
        rvec = rvecs2[k]
        tvec = tvecs2[k]
    elif mode == 'wavg':
        # weighted average results (weight = 1 / perViewError)
        weights = 1 / perViewErrors.flatten()
        limit = np.percentile(perViewErrors.flatten(), 80)
        print("use limit: ", limit)
        weights[perViewErrors.flatten() > limit] = 0.0
        tvec = np.average(tvecs2, axis=0, weights=weights)
        # rvec = np.average(rvecs, axis=0, weights = weights)



        # average rotation
        # first get list of quaternions
        qs = []
        for k in range(len(rvecs2)):
            q = utils.opencv2quat(rvecs2[k])
            qs.append(q)

        q = quaternion_average(qs, weights)

        # convert back to opencv's notation
        rvec = utils.quat2opencv(q)

    # output result

    # rvec = utils.quat2opencv(rowan.from_euler(np.radians(91),np.radians(0),np.radians(90), convention="xyz"))
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
    parser.add_argument('yaml_file') 
    parser.add_argument('--mode', default='wavg', choices=['min', 'wavg'])
    parser.add_argument('--filter', default=1e6, type=int)
    parser.add_argument('--dist', default='all', choices=['none', 'no-k', 'no-k23', 'no-k3', 'all'])
    parser.add_argument('--percentile', default=90, type=int)
    parser.add_argument('--extrinsic', action='store_true')

    args = parser.parse_args()
    yaml_file = args.yaml_file
    folder = Path(yaml_file).parent
    compute_calibration(yaml_file, folder / "calibration.yaml", args.mode, args.filter, args.dist, args.percentile, args.extrinsic)

if __name__ == "__main__":
    main()

