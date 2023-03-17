#!/usr/bin/python3
import numpy as np
from crazyflie_py import *
import subprocess 
import shutil
import os
import time
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path
# Assumes mrs case always, writing directly into real/mrs folder

Ids = [7, 2]
Heights = [0.5, 0.5]
folder_name = 'test_90'
path = '/home/akmaral/tubCloud/Shared/cvmrs/training_dataset/real/single/' + folder_name + '/Raw-Dataset/'
shutil.rmtree(path, ignore_errors=True)
os.makedirs(path)

run_file = "/home/akmaral/IMRS/cv-mrs/dataset-collection/real-data/data_collection_aideck.py"

cf_config = {
    7: {
        'IP':"130.149.82.50", # aideck-7
        'calibration': "/home/akmaral/tubCloud/Shared/cvmrs/calibration_real_a7.yaml",
        'waypoints': [
            [0.,0.,0.7],
            [0.,-0.25,0.7],
            [0.,0.25,0.7],
            [0.25,0.25,0.],
            [0.,0.25,0.7],
            [0.,-0.25,0.7],
            [0.,0.,0.7],
            [0.,-0.25,0.7],
            [0.,0.25,0.7],
            [0.25,0.25,0.7],
            [0.,0.25,0.7],
            [0.,-0.25,0.7],
        ]
    },
    2: {
        'IP': "130.149.82.51", # aideck-8
        'calibration': "/home/akmaral/tubCloud/Shared/cvmrs/calibration_real_a6.yaml",
        'waypoints': [
            [0.5,0.0,0.5],
            [0.25,0.25,0.5],
            [0.25,-0.25,0.5],
            [0.0,-0.25,0.5],
            [0.25,-0.25,0.5],
            [0.25,0.25,0.5],
            [0.5,0.0,0.5],
            [0.25,0.25,0.5],
            [0.25,-0.25,0.5],
            [0.0,-0.25,0.5],
            [0.25,-0.25,0.5],
            [0.25,0.25,0.5],
        ]
    }
}

SwapTimes = [4]

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    cfids = list(allcfs.crazyfliesById.keys())
    IPs = [cf_config[cfid]['IP'] for cfid in cfids]
    CFs = ["cf{}".format(cfid) for cfid in cfids]
    IDs = [cfid for cfid in cfids if 'IP' in cf_config[cfid]]
    CALIB_FILES = [cf_config[cfid]['calibration'] for cfid in cfids] 
    folder_list = [os.path.join(path, folder) for folder in CFs[:1]]
    IPs_FOLDERs_IDs = [(ip, folder, str(id)) for ip, folder, id in zip(IPs, folder_list, IDs)]
    # for image streaming part
    for _, folder_name, _ in IPs_FOLDERs_IDs:
        os.mkdir(os.path.join(path, folder_name)) 
           
    cmds_list = [["python3", run_file, "-n", ip, "-path", os.path.join(path, folder_name), "-id", id] for ip, folder_name, id in IPs_FOLDERs_IDs]
    procs_list = [subprocess.Popen(cmd) for cmd in cmds_list]
    
    for folder in folder_list:
        while True:
            time.sleep(1)
            if len(os.listdir(folder)) > 1:
                break
   

     # trigger = 1
    for cfid in IDs:
        allcfs.crazyfliesById[cfid].setParam("cvmrs.trigger", 1)
    timeHelper.sleep(2.)

   
    allcfs.setParam("usec.reset", 1)
    
    allcfs.setParam("usd.logging", 1)

    # for the flight part
    allcfs.takeoff(targetHeight=np.min(Heights), duration=3.0)
    timeHelper.sleep(2.)

    # go to initial positions
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        cf.goTo(pos, 0, 3.0)

    timeHelper.sleep(2.)

    # allow auto-yaw
    for cfid in IDs:
        allcfs.crazyfliesById[cfid].setParam("hlCommander.yawrate", 5.0)
    timeHelper.sleep(3.)
    
    for i in range(len(cf_config[Ids[0]]['waypoints'])):
        for cfid in Ids:
            pos = np.array(cf_config[cfid]['waypoints'][i])
            allcfs.crazyfliesById[cfid].goTo(pos, 0, 5.0)
        timeHelper.sleep(5)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, np.min(Heights)])
        cf.goTo(pos, 0, 5.0)
    timeHelper.sleep(5)

    for cfid in IDs:
        allcfs.crazyfliesById[cfid].setParam("hlCommander.yawrate", 0.0)

    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(5.0)

    allcfs.setParam("usd.logging", 0)
    timeHelper.sleep(2.0)
    

    for proc in procs_list:
        proc.terminate()

    for i in range(len(Ids)):
        shutil.copy(CALIB_FILES[i], os.path.join(path, folder_name) +'/calibration.yaml') 

if __name__ == "__main__":
    main()