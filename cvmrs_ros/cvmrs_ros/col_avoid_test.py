#!/usr/bin/python3
import numpy as np
from crazyflie_py import *
import subprocess 
import shutil
import os

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.setParam("colAv.enable", 1)

    allcfs.setParam("colAv.ellipsoidX", 0.15)
    allcfs.setParam("colAv.ellipsoidY", 0.15)
    allcfs.setParam("colAv.ellipsoidZ", 0.3)

    # for the flight part
    allcfs.takeoff(targetHeight=0.5, duration=3.0)
    timeHelper.sleep(3.0)

    # go to initial positions
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(3.0)

    # swap positions
    cf1 = allcfs.crazyflies[0]
    cf2 = allcfs.crazyflies[1]

    pos1 = np.array(cf2.initialPosition) + np.array([0, 0, 0.5])
    cf1.goTo(pos1, 0, 5.0)

    pos2 = np.array(cf1.initialPosition) + np.array([0, 0, 0.5])
    cf2.goTo(pos2, 0, 5.0)

    timeHelper.sleep(5.0)

   # swap positions (again)
    cf1 = allcfs.crazyflies[0]
    cf2 = allcfs.crazyflies[1]

    pos1 = np.array(cf1.initialPosition) + np.array([0, 0, 0.5])
    cf1.goTo(pos1, 0, 5.0)

    pos2 = np.array(cf2.initialPosition) + np.array([0, 0, 0.5])
    cf2.goTo(pos2, 0, 5.0)

    timeHelper.sleep(5.0)

    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(3.0)

if __name__ == "__main__":
    main()