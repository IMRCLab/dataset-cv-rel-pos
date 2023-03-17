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

    # allcfs.crazyfliesById[7].emergency()
    # allcfs.crazyfliesById[2].emergency()
    # allcfs.crazyfliesById[231].emergency()
    # allcfs.crazyfliesById[3].emergency()

    allcfs.setParam("usec.reset", 1)
    allcfs.setParam("kalman.quadIsFlying", 1) # pretend to be in-flight, to use the same sensor fusion

    # start recording to sdcard
    allcfs.setParam("usd.logging", 1)
    allcfs.takeoff(targetHeight=0.5, duration=3.0)

    timeHelper.sleep(15.)

    allcfs.land(targetHeight=0.02, duration=3.0)
    
    # stop recording to sdcard
    allcfs.setParam("usd.logging", 0)
    timeHelper.sleep(2.0)


if __name__ == "__main__":
    main()