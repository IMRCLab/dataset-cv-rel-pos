import numpy as np
import argparse
import rowan
import csv
import matplotlib.pyplot as plt
import os
# python3 path-to-sd_card path-to-gap8
def quaternion_interpolation(time, q, t):
    q_i = []
    for i in range(0, len(time)):
        if time[i] < t[0] or time[i] > t[-1]:
            q_i.append(None)
            continue
        min_val_index = np.where(t - time[i] >= 0)[0][0] # if t = t[i]
        t1, q_1 = t[min_val_index-1], q[min_val_index-1]
        t2, q_2 = t[min_val_index], q[min_val_index]

        interpolation_parameter = (time[i]-t1)/(t2-t1)
        q_inter = rowan.interpolate.slerp(q_1, q_2, interpolation_parameter)
        q_i.append(q_inter[0]) # interpolated quaternion
    return q_i
def get_image_names(csv_file):
    img_names = []
    with open(csv_file, 'r') as file: 
        csvreader_object= csv.reader(file)
        next(csvreader_object) # skip headers
        for row in csvreader_object:
            img_names.append(row[0]) # need just image names
    return img_names

def interpolate_wifi_usd_csv(file_wifi_csv, file_usd_csv, file_interpolated_csv):

    data_sdcard = np.loadtxt(file_usd_csv, comments='#',delimiter=',',skiprows=1, usecols=[i for i in range(1, 9)])
    data_gap8 = np.loadtxt(file_wifi_csv, comments='#',delimiter=',',skiprows=1, usecols=[i for i in range(1, 9)])    

    time = data_gap8[:,0]
    image_names = get_image_names(file_wifi_csv)

    x_interp = np.interp(time, data_sdcard[:,0], data_sdcard[:,1])
    y_interp = np.interp(time, data_sdcard[:,0], data_sdcard[:,2])
    z_interp = np.interp(time, data_sdcard[:,0], data_sdcard[:,3])
    q_interp = quaternion_interpolation(time, data_sdcard[:,4:], data_sdcard[:,0])

    with open(file_interpolated_csv, "w") as file:
        file.write("image_name,timestamp,x,y,z,qw,qx,qy,qz\n")
        for i in range(x_interp.shape[0]):
            if q_interp[i] is None:
                continue
            t,x,y,z,q = time[i],x_interp[i],y_interp[i],z_interp[i],q_interp[i]
            file.write("{},{},{},{},{},{},{},{},{}\n".format(image_names[i], t, x, y, z, q[0],q[1],q[2],q[3])) 


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("sd_card")
    parser.add_argument("gap8")
    parser.add_argument("csvfile")

    args = parser.parse_args()
    sd_card = args.sd_card
    gap8 = args.gap8
    csvfile = args.csvfile

    interpolate_wifi_usd_csv(gap8, sd_card, csvfile)

    # fig, axs = plt.subplots(3)
    # fig.suptitle('GAP8 and SD-CARD')
    # axs[0].plot(data_sdcard[:,0], data_sdcard[:,1]) #, 'o')
    # axs[0].plot(time, x_interp, '-x')

    # axs[1].plot(data_sdcard[:,0], data_sdcard[:,2]) #, 'o')
    # axs[1].plot(time, y_interp, '-x')

    # axs[2].plot(data_sdcard[:,0], data_sdcard[:,3]) #, 'o')
    # axs[2].plot(time, z_interp, '-x')

    # plt.savefig('gap8_sdcard_interpolation.jpg')

if __name__ == "__main__":
    main()

    
