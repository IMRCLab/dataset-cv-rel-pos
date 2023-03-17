#!/usr/bin/env python3

from re import I
import numpy as np

from crazyflie_py import *
import subprocess 
import shutil
import os
import time
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path
# SL
Ids = [1, 3]
Heights = [0.3, 0.5]
folder_name = 'test'
path = '/home/akmaral/tubCloud/Shared/cvmrs/training_dataset/real/mrs/' + folder_name + '/Raw-Dataset/'
shutil.rmtree(path, ignore_errors=True)
os.makedirs(path)

run_file = "/home/akmaral/IMRS/cv-mrs/data-collection/real/data_collection_aideck.py" # script to stream and save images
# id and ip-address of CFs being used
cf_config = {
    1: {
        'IP': "130.149.82.41",
        'calibration': "/home/akmaral/tubCloud/Shared/cvmrs/calibration_real_a7.yaml"
    },
    3: {
        'IP': "130.149.82.40",
        'calibration': "/home/akmaral/tubCloud/Shared/cvmrs/calibration_real_a6.yaml"
    }
}
cf_id = 3 # ID of the CF performing this circular motion
SwapTimes = [4]

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    cfids = list(allcfs.crazyfliesById.keys())
    IPs = [cf_config[cfid]['IP'] for cfid in cfids]
    CFs = ["cf{}".format(cfid) for cfid in cfids]
    CALIB_FILES = [cf_config[cfid]['calibration'] for cfid in cfids] 

    folder_list = [os.path.join(path, folder) for folder in CFs]
    IPs_FOLDERs_IDs = [(ip, folder, str(id)) for ip, folder, id in zip(IPs, folder_list, Ids)]

    # creating folders for each CF images separately
    for _, folder_name, _ in IPs_FOLDERs_IDs:
        os.mkdir(os.path.join(path, folder_name)) 

    cmds_list = [["python3", run_file, "-n", ip, "-path", os.path.join(path, folder_name), "-id", id] for ip, folder_name, id in IPs_FOLDERs_IDs]
    procs_list = [subprocess.Popen(cmd) for cmd in cmds_list]

    for folder in folder_list:
        while True:
            time.sleep(1)
            if len(os.listdir(folder)) > 1:
                break
    #upload trajectory, read from .csv file
    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / "../data/circle0.6.csv")
    
    TIMESCALE = 3.0
    allcfs.crazyfliesById[cf_id].uploadTrajectory(0, 0, traj1)

    # for the flight part
    allcfs.takeoff(targetHeight=np.min(Heights), duration=3.0)
    timeHelper.sleep(3.5)
    
    # go to initial positions
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        cf.goTo(pos, 0, 3.0)

    timeHelper.sleep(5)

    # allow auto-yaw
    for cfid in Ids:
        allcfs.crazyfliesById[cfid].setParam("hlCommander.yawrate", 5.0)

    # start trajectory
    allcfs.crazyfliesById[cf_id].startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)


    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, np.min(Heights)])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(5)

    for cfid in Ids:
        allcfs.crazyfliesById[cfid].setParam("hlCommander.yawrate", 0.0)

    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(5.0)

    for proc in procs_list:
        proc.terminate()
        
    for i in range(len(Ids)):
        shutil.copy(CALIB_FILES[i], os.path.join(path, folder_name) +'/calibration.yaml') 

if __name__ == "__main__":
    main()