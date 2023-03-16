#!/usr/bin/python3
import numpy as np
from crazyflie_py import *
import threading
import time
import shutil
import os
import subprocess 


def move(cf, last_pos, waypoints):
    for k, pos in enumerate(waypoints):
        dist = np.linalg.norm(last_pos - pos)
        speed = 0.2 # m/s
        time_to_move = max(dist / speed, 1.5)
        # some robot seems to ignore the commands -> send multiple times...
        for _ in range(3):
            cf.goTo(pos, 0, time_to_move)
        time.sleep(time_to_move+0.5)
        last_pos = pos

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    folder_name = '23'
    path = '/home/whoenig/tmp/cvmrs/downwash/' + folder_name + '/Raw-Dataset/'
    # shutil.rmtree(path, ignore_errors=True)
    os.makedirs(path)
    run_file = "/home/whoenig/projects/tuberlin/cv-mrs/dataset-collection/real-data/data_collection_aideck.py"

    cf_config = {
        6: {
            # 'IP': "130.149.82.42", #aideck-6 (upward)
            # 'calibration': "/home/whoenig/tmp/cvmrs/camera-calibration/aideck6_4/cf6/calibration.yaml",
            # 'bbox_min': [-0.5,-0.5,0.4],
            # 'bbox_max': [0.5,0.5,0.6],
            # 'yawrate': 8.0,

            # front camera flight
            'yawrate': 0.0,
            'waypoints': [
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                ]
        },
        7: {
            'IP':"130.149.82.50", # aideck-7 (forward)
            'calibration': "/home/whoenig/tubCloud/projects/cvmrs/camera-calibration/aideck7_2/cf7/calibration.yaml",
            # 'bbox_min': [-0.5,-0.5,0.6],
            # 'bbox_max': [0.5,0.5,1.1],
            # 'yawrate': 0.0,

            # front camera flight
            'yawrate': 8.0,
            'waypoints': [
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                ]
        },
        8: {
            'IP':"130.149.82.51", # aideck-8 (45deg)
            'calibration': "/home/whoenig/tubCloud/projects/cvmrs/camera-calibration/aideck8_2/cf8/calibration.yaml",
            # 'bbox_min': [-0.5,-0.5,0.4],
            # 'bbox_max': [0.5,0.5,0.6],
            # 'yawrate': 8.0,

            # front camera flight
            'yawrate': 0.0,
            'waypoints': [
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                    [0.5,0.0,0.5],
                    [0.0,0.0,0.5],
                ]
        },
        12: {
            # 'bbox_min': [-0.5,-0.5,0.6],
            # 'bbox_max': [0.5,0.5,1.1],
            # 'yawrate': 0.0,

            # front camera flight
            'yawrate': 0.0,
            'waypoints': [
                    [0.5,0.0,0.8],
                    [0.0,0.0,0.8],
                    [0.5,0.0,0.8],
                    [0.0,0.0,0.8],
                    [0.5,0.0,0.8],
                    [0.0,0.0,0.8],
                    [0.5,0.0,0.8],
                    [0.0,0.0,0.8],
                    [0.5,0.0,0.8],
                ]
        }
    }
    cfids = list(allcfs.crazyfliesById.keys())
    IPs = [cf_config[cfid]['IP'] for cfid in cfids if 'IP' in cf_config[cfid]]
    IDs = [cfid for cfid in cfids if 'IP' in cf_config[cfid]]
    CALIB_FILES = [cf_config[cfid]['calibration']  for cfid in cfids if 'IP' in cf_config[cfid]] 
    folder_list = [os.path.join(path, "cf{}".format(cfid)) for cfid in cfids if 'IP' in cf_config[cfid]]
    IPs_FOLDERs_IDs = [(ip, folder, str(id)) for ip, folder, id in zip(IPs, folder_list, IDs)]
    bbox_min = np.array([-0.6,-0.6,0.2])
    bbox_max = np.array([0.6,0.6,1.2])

    allcfs.setParam("stabilizer.controller", 4) # switch to SJC controller

    allcfs.setParam("colAv.ellipsoidX", 0.15)
    allcfs.setParam("colAv.ellipsoidY", 0.15)
    allcfs.setParam("colAv.ellipsoidZ", 0.8)

    allcfs.setParam("colAv.bboxMinX", bbox_min[0])
    allcfs.setParam("colAv.bboxMinY", bbox_min[1])
    allcfs.setParam("colAv.bboxMinZ", bbox_min[2])

    allcfs.setParam("colAv.bboxMaxX", bbox_max[0])
    allcfs.setParam("colAv.bboxMaxY", bbox_max[1])
    allcfs.setParam("colAv.bboxMaxZ", bbox_max[2])

    # creating folders for each CF images separately
    for _, folder_name, _ in IPs_FOLDERs_IDs:
        os.mkdir(os.path.join(path, folder_name)) 

    allcfs.setParam("usec.reset", 1)
    allcfs.setParam("cvmrs.num_img", 1)

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
    timeHelper.sleep(2.)

    # for the flight part
    allcfs.setParam("motorPowerSet.enable", 0) # make sure mocap can see us
    timeHelper.sleep(0.5)
    allcfs.takeoff(targetHeight=0.5, duration=3.0)
    timeHelper.sleep(3.0)

    # enable collision avoidance
    allcfs.setParam("colAv.enable", 1)

    # auto-yaw and go to first position
    for cfid, cf in allcfs.crazyfliesById.items():
        cf.setParam("hlCommander.yawrate", cf_config[cfid]['yawrate'])
        pos = cf_config[cfid]['waypoints'][0]
        for _ in range(3):
            cf.goTo(pos, 0, 5)

    timeHelper.sleep(6.0)

    # disable collision avoidance
    allcfs.setParam("colAv.enable", 0)
    timeHelper.sleep(1.0)

    # # start thread for each cf
    # threads = []
    # for cfid, cf in allcfs.crazyfliesById.items():
    #     pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
    #     thread = threading.Thread(target=move, args=(cf, pos, np.asarray(cf_config[cfid]['waypoints'][1:])))
    #     thread.start()
    #     threads.append(thread)

    # # wait for all threads to be done
    # for thread in threads:
    #     thread.join()

    for k in range(8):
        for cfid, cf in allcfs.crazyfliesById.items():
            waypoints = cf_config[cfid]['waypoints'][1:]
            pos = waypoints[k]
            dist = 0.5
            speed = 0.2 # m/s
            time_to_move = max(dist / speed, 1.5)
            # some robot seems to ignore the commands -> send multiple times...
            for _ in range(3):
                cf.goTo(pos, 0, time_to_move)
        time.sleep(time_to_move+0.5)

    # re-enable collision avoidance
    allcfs.setParam("colAv.enable", 1)
    timeHelper.sleep(1.0)

    # stop auto-yaw and go back to initial position
    allcfs.setParam("hlCommander.yawrate", 0.0)
    for cfid, cf in allcfs.crazyfliesById.items():
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        for _ in range(3):
            cf.goTo(pos, 0, 5)

    timeHelper.sleep(7.0)

   
    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(3.0)

    # disable collision avoidance
    allcfs.setParam("colAv.enable", 0)

    # stop recording to sdcard
    allcfs.setParam("usd.logging", 0)
    timeHelper.sleep(2.0)

    for proc in procs_list:
        proc.terminate()

    # save calibration file
    for i in range(len(IDs)):
        shutil.copy(CALIB_FILES[i], os.path.join(path, folder_name) +'/calibration.yaml') 

if __name__ == "__main__":
    main()