import argparse

import matplotlib.pyplot as plt
import numpy as np
import rowan

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("file", nargs='+')
    args = parser.parse_args()

    fig, ax = plt.subplots(3, 1)

    for f in args.file:
        data = np.loadtxt(f,comments='#',delimiter=',',skiprows=1, usecols=(1,2,3,4,5,6,7,8))
        q = data[:,4:8]
        rpy = rowan.to_euler(rowan.normalize(q), "xyz")

        ax[0].plot(data[:,0], np.degrees(rpy[:,0]), label=f)

        ax[1].plot(data[:,0], np.degrees(rpy[:, 1]), label=f)

        ax[2].plot(data[:,0], np.degrees(rpy[:, 2]), label=f)

    ax[0].legend()
    plt.show()

