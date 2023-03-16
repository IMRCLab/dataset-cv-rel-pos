#!/usr/bin/env python3

import numpy as np

from crazyflie_py import *
import subprocess 
import shutil
import os
import time

# # SS
# Ids = [50, 51]
# Heights = [0.6, 0.9]
# Radius = 0.3

# SL
Ids = [1, 3]
Heights = [0.3, 1.2]
Radius = 0.3
path = "/home/akmaral/cfs"
run_file = "/home/akmaral/IMRS/cv-mrs/data_collection_aideck.py"

cf_config = {
    1: {
        'IP': "130.149.82.41",
        'waypoints': [
            [0.5,0,0.5],
            [0.5,-0.5,0.5],
            [0.5,0,0.5],
        ]
    },
    3: {
        'IP': "130.149.82.40",
        'waypoints': [
            [0.0,-0.5,0.5],
            [0.0,0.0,0.5],
            [0.0,-0.5,0.5],
        ]
    }
}

# # 3 agent case
# Ids = [50, 101, 102]
# # Heights = [0.6, 0.85, 1.1]
# # Heights = [0.5, 0.8, 1.1]
# # Heights = [0.5, 0.9, 1.1]
# Heights = [0.8, 0.5, 1.1]
# Radius = 0.35

# # 4 agent case
# Ids = [200, 201, 204, 203]
# Heights = [0.35, 0.6, 0.85, 1.1]
# Radius = 0.4

# # 5 agent case
# Ids = [204, 203, 202, 201, 200]
# Heights = [0.3, 0.5, 0.7, 0.9, 1.1]
# Radius = 0.4

# SwapTimes = [4,3,2]
# SwapTimes = [3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
# SwapTimes = [3, 3, 3]
SwapTimes = [4]

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    shutil.rmtree(path, ignore_errors=True)
    os.mkdir(path)

    cfids = list(allcfs.crazyfliesById.keys())
    IPs = [cf_config[cfid]['IP'] for cfid in cfids]
    CFs = ["cf{}".format(cfid) for cfid in cfids]
    
    folder_list = [os.path.join(path, folder) for folder in CFs]
    IPs_FOLDERs = [(ip, folder) for ip, folder in zip(IPs, folder_list)]
    
    for _, folder_name in IPs_FOLDERs:
        os.mkdir(os.path.join(path, folder_name)) # creating folders 

    cmds_list = [["python3", run_file, "-n", ip, "-path", os.path.join(path, folder_name)] for ip, folder_name in IPs_FOLDERs]
    procs_list = [subprocess.Popen(cmd) for cmd in cmds_list]

    for folder in folder_list:
        while True:
            time.sleep(1)
            if len(os.listdir(folder)) > 1:
                break



    # print(list(allcfs.crazyfliesById.keys()))
    # exit()

    # allcfs.setParam("stabilizer/controller", 5) # use Lee controller

    allcfs.takeoff(targetHeight=np.min(Heights), duration=3.0)
    timeHelper.sleep(3.5)

    # # go to initial positions
    # angles = np.linspace(0, 2*np.pi, 2 * len(Ids), endpoint=False)
    # # print(angles, angles[0:len(Ids)], angles[len(Ids):])
    # for angle, cfid, height in zip(angles[0:len(Ids)], Ids, Heights):
    #     pos = np.array([np.sin(angle) * Radius, np.cos(angle) * Radius, height])
    #     allcfs.crazyfliesById[cfid].goTo(pos, 0, 3.0)

    # go to initial positions
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        cf.goTo(pos, 0, 3.0)

    timeHelper.sleep(5)

    for cfid in Ids:
        allcfs.crazyfliesById[cfid].setParam("hlCommander.yawrate", 5.0)

    # allcfs.setParam("usd/logging", 1)

    for i in range(3):
        for cfid in Ids:
            pos = np.array(cf_config[cfid]['waypoints'][i])
            print(cfid, pos)
            allcfs.crazyfliesById[cfid].goTo(pos, 0, 3.0)
        timeHelper.sleep(5)

    # for swapTime in SwapTimes:
    #     # swap 1
    #     for angle, cfid, height in zip(angles[len(Ids):], Ids, Heights):
    #         pos = np.array([np.sin(angle) * Radius, np.cos(angle) * Radius, height])
    #         allcfs.crazyfliesById[cfid].goTo(pos, 0, 3.0)
    #     timeHelper.sleep(swapTime+1.5)

    #     # swap 2
    #     for angle, cfid, height in zip(angles[0:len(Ids)], Ids, Heights):
    #         pos = np.array([np.sin(angle) * Radius, np.cos(angle) * Radius, height])
    #         allcfs.crazyfliesById[cfid].goTo(pos, 0, 3.0)
    #     timeHelper.sleep(swapTime+1.5)

    # allcfs.setParam("usd/logging", 0)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, np.min(Heights)])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(5)

    # timeHelper.sleep(10)

    for cfid in Ids:
        allcfs.crazyfliesById[cfid].setParam("hlCommander.yawrate", 0.0)

    # timeHelper.sleep(5)

    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(5.0)

    for proc in procs_list:
        proc.terminate()

if __name__ == "__main__":
    main()