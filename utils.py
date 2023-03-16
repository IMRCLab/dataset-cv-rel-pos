import cv2
import numpy as np
import os
import glob
import csv
import yaml
import rowan
import math


# rotation vectors are axis-angle format in "compact form", where
# theta = norm(rvec) and axis = rvec / theta
# they can be converted to a matrix using cv2. Rodrigues, see
# https://docs.opencv.org/4.7.0/d9/d0c/group__calib3d.html#ga61585db663d9da06b68e70cfbf6a1eac
def opencv2quat(rvec):
    angle = np.linalg.norm(rvec)
    if angle == 0:
        q = np.array([1,0,0,0])
    else:
        axis = rvec.flatten() / angle
        q = rowan.from_axis_angle(axis, angle)
    return q

def quat2opencv(q):
    axis, angle = rowan.to_axis_angle(q)
    norm = np.linalg.norm(axis[0])
    if norm > 0:
        rvec = np.array([axis[0] / np.linalg.norm(axis[0]) * angle[0]])
    else:
        rvec = np.zeros(3)
    return rvec

# converts pose [x,y,z,qw,qx,qy,qz] to 4x4 transformation matrix
def pose2transform(translation, q):
    t = np.zeros([4,4])
    R = rowan.to_matrix(q)
    t[:3,:3] = R
    t[0:3, 3] = translation
    t[3, 3] = 1
    return t

def transform2pose(transform):
    translation = transform[0:3, 3]
    R = transform[:3,:3]
    q = rowan.from_matrix(R)
    return translation, q

# points are 3xN
def apply_transform(transform, points):
    if points.ndim == 1:
        pts = points.reshape((3,1))
    else:
        pts = points
    # convert points to homogenous coordinates
    hpoints = np.vstack((pts, np.ones(pts.shape[1])))
    hresult = transform @ hpoints
    # convert result back to normal coordinates
    result = hresult[:3,:]/hresult[[-1],:]
    if points.ndim == 1:
        result = result.flatten()
    return result

# CF pose and image name returned as dictionary 
def cf_dict(csv_file):
    cf_list = []
    dicts = {}
    poses = np.loadtxt(csv_file,comments='#',delimiter=',',skiprows=1, usecols=(2,3,4,5,6,7,8)) # skip timestamp
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            cf_list.append(row)
    for i in range(1, len(cf_list)):
        dicts[cf_list[i][0]] = poses[i-1]
    return dicts

# CF pose in mocap system
def cf_mocap(cf_pose):
    t_cf_m = np.zeros([4,4])
    qw = cf_pose[3]
    qx = cf_pose[4]
    qy = cf_pose[5]
    qz = cf_pose[6]
    R = rowan.to_matrix([qw, qx, qy, qz])
    t_cf_m[:3,:3] = R
    t_cf_m[0, 3] = cf_pose[0]
    t_cf_m[1, 3] = cf_pose[1]
    t_cf_m[2, 3] = cf_pose[2]
    t_cf_m[3, 3] = 1
    return t_cf_m

# See (2) of https://whoenig.github.io/publications/2017_IROS_Preiss.pdf
# For an efficient multi-robot version, see https://github.com/USC-ACTLab/crazyswarm/blob/master/ros_ws/src/crazyswarm/scripts/pycrazyswarm/util.py

def check_downwash(p, q, E=np.diag([0.15, 0.15, 0.3])):
    """
    returns True, if robot at position p causes robot at position q to suffer from downwash
    """
    # first compute the ellipsoid intersection
    # E = np.diag([0.15, 0.15, 0.3])
    # E = np.diag([0.3, 0.3, 0.6])
    intersects = np.linalg.norm(np.linalg.pinv(E) @ (p - q)) < 2
    # second, check if p is above q
    p_above_q = p[2] > q[2]
    return p_above_q and intersects
