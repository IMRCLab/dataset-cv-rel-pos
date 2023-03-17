#!/usr/bin/env python3

from crazyflie_py import *


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # reset all clocks to zero (broadcast)
    allcfs.setParam("usec.reset", 1)

    # start logging (broadcast)
    allcfs.setParam("usd.logging", 1)
    timeHelper.sleep(5.0)

    # stop logging
    allcfs.setParam("usd.logging", 0)
    timeHelper.sleep(2.0)


if __name__ == "__main__":
    main()
