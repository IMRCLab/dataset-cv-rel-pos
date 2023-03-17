#!/usr/bin/python3
import numpy as np
from crazyflie_py import *
import subprocess 
import shutil
import os
import time
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path
# Assumes all waypoints have the same length

Ids = [7, 231, 3]
Heights = [0.5, 0.5, 0.5]
path = "/home/akmaral/IMRS/dataset/optic-flow/circular-4/" # path to folder to save images
run_file = "/home/akmaral/IMRS/cv-mrs/data_collection_aideck.py" # script to stream and save images
cf_id_sd = 3

cf_config = {
    7: {
        'IP':"130.149.82.50", # aideck-7
        'waypoints': [
            [0.,0.,0.5],
            [0.,-0.20,0.5],
            [0.,-0.20,0.5],
            [0.,-0.20,0.5],
        ]
    },
    231: {
        'IP': "130.149.82.49", # aideck-6
        'waypoints': [
            [0.5,-0.5,0.5],
            [0.5,-0.25,0.5],
            [0.5,-0.5,0.5],
            [0.25,-0.5,0.5],
        ]
    },
    3: {
        'IP': "130.149.82.42", # aideck-2
        'waypoints': [
            [0.5,0.0,0.5],
            [0.20,0.,0.5],
            [0.5,0.,0.5],
            [0.5,-0.25,0.5],
        ]
    }
}

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
    
    folder_list = [os.path.join(path, folder) for folder in CFs[:2]]
    IPs_FOLDERs = [(ip, folder) for ip, folder in zip(IPs, folder_list)]
    # for image streaming part

    # creating folders for each CF images separately
    for _, folder_name in IPs_FOLDERs:
        os.mkdir(os.path.join(path, folder_name)) 

    cmds_list = [["python3", run_file, "-n", ip, "-path", os.path.join(path, folder_name)] for ip, folder_name in IPs_FOLDERs]
    procs_list = [subprocess.Popen(cmd) for cmd in cmds_list]
    
    for folder in folder_list:
        while True:
            time.sleep(1)
            if len(os.listdir(folder)) > 1:
                break
   

     # trigger = 1
    for cfid in Ids[:2]:
        allcfs.crazyfliesById[cfid].setParam("cvmrs.trigger", 1)
    timeHelper.sleep(2.)

    for cfid in Ids:
        allcfs.crazyfliesById[cfid].setParam("usd.logging", 1)

    # for the flight part
    allcfs.takeoff(targetHeight=np.min(Heights), duration=3.0)
    timeHelper.sleep(2.)

    # go to initial positions
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        cf.goTo(pos, 0, 3.0)

    timeHelper.sleep(2.)

    # allow auto-yaw
    for cfid in Ids[1:]:
        allcfs.crazyfliesById[cfid].setParam("hlCommander.yawrate", 5.0)
    timeHelper.sleep(3.)


    for i in range(len(cf_config[Ids[0]]['waypoints'])):
        for cfid in Ids:
            pos = np.array(cf_config[cfid]['waypoints'][i])
            allcfs.crazyfliesById[cfid].goTo(pos, 0, 3.0)
        timeHelper.sleep(5)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, np.min(Heights)])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(5)

    for cfid in Ids[1:]:
        allcfs.crazyfliesById[cfid].setParam("hlCommander.yawrate", 0.0)

    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(5.0)

    for cfid in Ids:
        allcfs.crazyfliesById[cfid].setParam("usd.logging", 0)

    for proc in procs_list:
        proc.terminate()

if __name__ == "__main__":
    main()