#!/usr/bin/python3
import numpy as np
from crazyflie_py import *
import threading
import time
import shutil
import os
import subprocess 


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    folder_name = '6_intrinsic_3'
    path = '/home/whoenig/tmp/cvmrs/camera-calibration/' + folder_name
    # shutil.rmtree(path, ignore_errors=True)
    os.makedirs(path, exist_ok=True)
    run_file = "/home/whoenig/projects/tuberlin/cv-mrs/dataset-collection/real-data/data_collection_aideck.py"

    cf_config = {
        6: {
            'IP': "130.149.82.49", #aideck-6
        },
        7: {
            'IP':"130.149.82.50", # aideck-7
        },
        8: {
            'IP':"130.149.82.51", # aideck-8
        },
        # 231: {
        # },
        # 3: {
        # }
    }
    cfids = list(allcfs.crazyfliesById.keys())
    IPs = [cf_config[cfid]['IP'] for cfid in cfids if 'IP' in cf_config[cfid]]
    IDs = [cfid for cfid in cfids if 'IP' in cf_config[cfid]]
    folder_list = [os.path.join(path, "cf{}".format(cfid)) for cfid in cfids if 'IP' in cf_config[cfid]]
    IPs_FOLDERs_IDs = [(ip, folder, str(id)) for ip, folder, id in zip(IPs, folder_list, IDs)]

    # # creating folders for each CF images separately
    # for _, folder_name, _ in IPs_FOLDERs_IDs:
    #     os.mkdir(os.path.join(path, folder_name)) 

    allcfs.setParam("usec.reset", 1)

    allcfs.setParam("cvmrs.num_img", 1)
    allcfs.setParam("cvmrs.exposure", 30)

    allcfs.setParam("cvmrs.trigger", 1)

    cmds_list = [["python3", run_file, "-n", ip, "-path", os.path.join(path, folder_name), "-id", id] for ip, folder_name, id in IPs_FOLDERs_IDs]
    procs_list = [subprocess.Popen(cmd) for cmd in cmds_list]
    
    for folder in folder_list:
        while True:
            time.sleep(1)
            if len(os.listdir(folder)) > 1:
                break  

    # start recording to sdcard
    allcfs.setParam("usd.logging", 1)
    timeHelper.sleep(30.)
    
    # stop recording to sdcard
    allcfs.setParam("usd.logging", 0)
    timeHelper.sleep(2.0)

    for proc in procs_list:
        proc.terminate()

    # save calibration file
    # for i in range(len(IDs)):
    #     shutil.copy(CALIB_FILES[i], os.path.join(path, folder_name) +'/calibration.yaml') 

if __name__ == "__main__":
    main()