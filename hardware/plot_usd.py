import argparse

import matplotlib.pyplot as plt
import numpy as np
import rowan

import cfusdlog

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("file_usd")
    args = parser.parse_args()

    data_usd = cfusdlog.decode(args.file_usd)

    # start_time = np.inf
    # for _,v in data_usd.items():
    #     start_time = min(start_time, v['timestamp'][0])

    start_time = 0

    time_fF = (data_usd['fixedFrequency']['timestamp'] - start_time) / 1e3
    time_eP = (data_usd['estPose']['timestamp'] - start_time) / 1e3

    dt_eP = np.diff(time_eP)
    print("Mocap rate: {:.1f} Hz ({:.1f})".format(np.mean(1/dt_eP), np.std(1/dt_eP)))
    fig, ax = plt.subplots(1, 1, squeeze=False)
    ax[0,0].hist(1/dt_eP)
    plt.show()

    T = len(data_usd['fixedFrequency']['timestamp'])

    pos = np.array([
        data_usd['fixedFrequency']['stateEstimate.x'],
        data_usd['fixedFrequency']['stateEstimate.y'],
        data_usd['fixedFrequency']['stateEstimate.z']]).T

    q = np.array([
        data_usd['fixedFrequency']['stateEstimate.qw'],
        data_usd['fixedFrequency']['stateEstimate.qx'],
        data_usd['fixedFrequency']['stateEstimate.qy'],
        data_usd['fixedFrequency']['stateEstimate.qz']]).T
    rpy = rowan.to_euler(rowan.normalize(q), "xyz")

    pos_mocap = np.array([
        data_usd['estPose']['locSrv.x'],
        data_usd['estPose']['locSrv.y'],
        data_usd['estPose']['locSrv.z']]).T

    q_mocap = np.array([
        data_usd['estPose']['locSrv.qw'],
        data_usd['estPose']['locSrv.qx'],
        data_usd['estPose']['locSrv.qy'],
        data_usd['estPose']['locSrv.qz']]).T
    rpy_mocap = rowan.to_euler(rowan.normalize(q_mocap), "xyz")


    fig, ax = plt.subplots(3, 1)
    ax[0].plot(time_fF, np.degrees(rpy[:,0]), label='state estimate')
    ax[0].plot(time_eP, np.degrees(rpy_mocap[:,0]), '.', label='mocap')

    ax[0].legend()

    ax[1].plot(time_fF, np.degrees(rpy[:, 1]), label='state estimate')
    ax[1].plot(time_eP, np.degrees(rpy_mocap[:,1]), '.', label='mocap')

    ax[2].plot(time_fF, np.degrees(rpy[:, 2]), label='state estimate')
    ax[2].plot(time_eP, np.degrees(rpy_mocap[:,2]), '.', label='mocap')

    plt.show()

    fig, ax = plt.subplots(3, 1)
    ax[0].plot(time_fF, pos[:,0], label='state estimate')
    ax[0].plot(time_eP, pos_mocap[:,0], '.', label='mocap')

    ax[0].legend()

    ax[1].plot(time_fF, pos[:, 1], label='state estimate')
    ax[1].plot(time_eP, pos_mocap[:,1], '.', label='mocap')

    ax[2].plot(time_fF, pos[:, 2], label='state estimate')
    ax[2].plot(time_eP, pos_mocap[:,2], '.', label='mocap')

    plt.show()

