import csv
import yaml
import numpy as np
import os
import rowan
import cv2
import argparse
import glob
import shutil
import utils
from pathlib import Path
import matplotlib.pyplot as plt

from hardware.convert_log_to_csv import usdlog2csv
from hardware.interpolate import interpolate_wifi_usd_csv
from hardware.interpolate import get_image_names
from hardware.interpolate import quaternion_interpolation



def raw2sync(folder_raw, folder_sync):

    # shutil.rmtree(folder_sync, ignore_errors=True)
    # os.mkdir(folder_sync)
    main_folder = Path(folder_raw).parent
    # start by checking if we have indeed a raw dataset
    robot_names = [f.name for f in Path(folder_raw).glob("*") if f.is_dir]

    # check if synthetic data or not
    synthetic_data = True
    for robot in robot_names:
        folder = Path(folder_raw) / robot
        usd_files = [f for f in folder.glob("*") if f.suffix == '']
        if len(usd_files) > 0:
            synthetic_data = False
            break

    for robot in robot_names:
        folder = Path(folder_raw) / robot

        file_interpolated_csv = folder / (robot + '.csv')

        if synthetic_data:
            print("{} is from simulation (already interpolated)".format(robot))
        else:
            # convert raw log files
            usd_files = [f for f in folder.glob("*") if f.suffix == '']
            if len(usd_files) != 1:
                print("Error, no usd files for robot {}".format(robot))
                return
            file_usd = usd_files[0]
            file_usd_csv = file_usd.parent / (robot + '.csv')
            usdlog2csv(file_usd, file_usd_csv, False)

            if (folder / "calibration.yaml").exists():
                wifi_files = list(folder.glob("*wifi.csv"))
                if len(wifi_files) != 1:
                    print("Error, no wifi CSV file!")
                    return
                file_wifi_csv = wifi_files[0]
                
                interpolate_wifi_usd_csv(file_wifi_csv, file_usd_csv, file_interpolated_csv)

                print("{} has a camera!".format(robot))
            else:
                print("{} has NO camera!".format(robot))

    maxRobot = len(robot_names)
    x, y, z = 0.033, 0.033, 0.01
    CF_corners = np.array([[x, y, -z],
                 [x, y, z],
                 [-x, y, z],
                 [-x, y, -z],
                 [x, -y, -z],
                 [x, -y, z],
                 [-x, -y, z],
                 [-x, -y, -z]])
    img_size = (320,320)
    img_names, robot_numbers = [], []
    dataset, robot_calibration, images = {}, {}, {}
    for f in range(maxRobot):
        per_robot_number_path = Path(folder_sync) / str(f)
        shutil.rmtree(per_robot_number_path, ignore_errors=True)
        os.makedirs(per_robot_number_path)

    for robot in robot_names:
        folder = Path(folder_raw) / robot

        if (folder / "calibration.yaml").exists():
            camera_parameters = folder / 'calibration.yaml'
            # calibration key
            with open(camera_parameters) as f:
                robot_calibration[robot] = yaml.load(f, Loader=yaml.CSafeLoader)
            int_mtrx = np.array(robot_calibration[robot]['camera_matrix'])
            dist_v = np.array(robot_calibration[robot]['dist_coeff'])
            t_v = np.array(robot_calibration[robot]['tvec'])
            r_v = np.array(robot_calibration[robot]['rvec'])
            q = utils.opencv2quat(r_v)
            T_robot1_to_camera = utils.pose2transform(t_v, q)

            dataset['calibration'] = robot_calibration
            csv1 = folder / (robot + '.csv')
            img_names = get_image_names(csv1)
            robot1_states = np.loadtxt(csv1, comments='#',delimiter=',',skiprows=1, usecols=[i for i in range(1, 9)])
            time = robot1_states[:,0]
            x_interp, y_interp, z_interp, q_interp, robot_k_names = [], [], [], [], []

            for robot2 in robot_names:
                if robot == robot2:
                    continue

                csv2 = Path(folder_raw) / robot2 / (robot2 + '.csv')
                rob_in_w = np.loadtxt(csv2, comments='#',delimiter=',',skiprows=1, usecols=[i for i in range(1, 9)])    
                x_interp.append(np.interp(time, rob_in_w[:,0], rob_in_w[:,1]))
                y_interp.append(np.interp(time, rob_in_w[:,0], rob_in_w[:,2]))
                z_interp.append(np.interp(time, rob_in_w[:,0], rob_in_w[:,3]))
                q_interp.append(quaternion_interpolation(time, rob_in_w[:,4:], rob_in_w[:,0]))
                robot_k_names.append(robot2)
            for t in range(len(time)): # each timestamp/image
                per_image = {}
                per_image['visible_neighbors'] = []
                per_image['calibration'] = robot
                T_robot1_to_world = utils.pose2transform(robot1_states[t,1:4], robot1_states[t,4:8])
                per_image['pose'] = robot1_states[t,1:8].tolist()
                per_image['t'] = float(time[t])
                per_image['neighbors'] = []
                
                # print(folder / img_names[t])
                image = cv2.imread(str(folder / img_names[t]))
                for k in range(len(q_interp)): # for each robot
                    if q_interp[k][t] is None:
                        continue

                    robot_k_pos_in_world = np.array([x_interp[k][t],y_interp[k][t], z_interp[k][t]])
                    robot_k_in_cam = utils.apply_transform(T_robot1_to_camera @ np.linalg.inv(T_robot1_to_world), robot_k_pos_in_world)

                    per_robot = {}
                    per_robot['name'] = robot_k_names[k]
                    per_robot['pos_world'] = robot_k_pos_in_world.tolist()
                    per_image['neighbors'].append(per_robot)

                    # # check downwash
                    # if utils.check_downwash(robot_k_pos_in_world, robot1_states[t,1:4]):
                    #     per_image['downwash_caused_by'].append(robot_k_names[k])

                    if robot_k_in_cam[2]>0:
                        # Save all images - with/without robot
                        # x_hist.append(robot_k_in_cam[0])
                        # y_hist.append(robot_k_in_cam[1])
                        # z_hist.append(robot_k_in_cam[2])
                        # for locanet
                        pixels, _ = cv2.projectPoints(robot_k_in_cam, np.zeros(3), np.zeros(3), int_mtrx, dist_v)
                        pixels = np.round(pixels.flatten())

                        # center not in image -> robot k is not visible from other robot
                        if pixels[0] > img_size[0]-5 or pixels[0] < 5 or pixels[1] > img_size[1]-5 or pixels[1] < 5:
                            continue
                        # if zInImg >= img_size[0] or yInImg >= img_size[1] or zInImg <= 0 or yInImg <= 0: 
                        #     continue


                        # for yolov3
                        T_robot_k_to_world = utils.pose2transform(robot_k_pos_in_world, q_interp[k][t])

                        corners_in_cam = utils.apply_transform(T_robot1_to_camera @ np.linalg.inv(T_robot1_to_world) @ T_robot_k_to_world, CF_corners.T)
                        corners, _ = cv2.projectPoints(corners_in_cam, np.zeros(3), np.zeros(3), int_mtrx, dist_v)
                        corners = corners.reshape((8,2))

                        xmin = round(corners[:,0].min())
                        ymin = round(corners[:,1].min())
                        xmax = round(corners[:,0].max())
                        ymax = round(corners[:,1].max())
                        bbox = np.array([xmin,ymin,xmax,ymax])
                        # print(img_names[t], pixels, bbox)

                        # if the bounding box is unreasonbly big, the drone is like not visible at all
                        xsize = xmax - xmin
                        ysize = ymax - ymin
                        if xmin > img_size[0] or xmax < 0 or ymin > img_size[0] or ymax < 0:
                            continue

                        # clip result (assuming square images!)
                        pixels = np.clip(pixels, 0, 320)
                        bbox = np.clip(bbox, 0, 320)

                        if not synthetic_data:
                            # if the robot is more than half outside, skip
                            xsize_clipped = bbox[2] - bbox[0]
                            ysize_clipped = bbox[3] - bbox[1]
                            if xsize_clipped * ysize_clipped < xsize*ysize / 2:
                                continue
                            if xsize > img_size[0] / 3 or ysize > img_size[1] / 3:
                                continue

                        per_robot = {}
                        per_robot['name'] = robot_k_names[k]
                        per_robot['pix'] =  pixels.tolist()
                        per_robot['corners'] = corners.tolist()
                        per_robot['pos'] = robot_k_in_cam.tolist()
                        per_robot['pos_world'] = robot_k_pos_in_world.tolist()
                        per_robot['bb'] = bbox.tolist()
                        per_image['visible_neighbors'].append(per_robot)
                    
                num_neighbors = len(per_image['visible_neighbors'])
                cv2.imwrite(str(Path(folder_sync) / str(num_neighbors) / img_names[t]), image)
                robot_numbers.append(num_neighbors)

                per_image['calibration'] = robot
                images[str(num_neighbors) + "/" + img_names[t]] = per_image
                # images[str(folder / img_names[t])] = per_image

    dataset['images'] = images

    yaml.Dumper.ignore_aliases = lambda *args : True    
    with open(Path(folder_sync) / 'dataset_all.yaml', 'w') as outfile:
            yaml.dump(dataset, outfile)   

    # # Position Histogram
    # f, (ax1, ax2, ax3) = plt.subplots(1, 3, sharey=True)
    # ax1.hist(x_hist)
    # ax1.set_title('X')
    # ax2.hist(y_hist)
    # ax2.set_title('Y')
    # ax3.hist(z_hist)
    # ax3.set_title('Z')
    # plt.savefig(path + 'position_histogram.jpg')
    # # plt.show()
    # Total cases: no robot, one or two, etc.
    labels, counts = np.unique(np.array(robot_numbers), return_counts=True)
    plt.title('The Whole Dataset')
    plt.bar(labels, counts, align='center')
    plt.gca().set_xticks(labels)
    plt.ylabel('Number of Images')
    plt.xlabel('Number of Robots')
    plt.savefig(str(main_folder) +'/' + 'robot_number.jpg')

    # # compute more stats
    # num_cases_with_downwash = 0
    # num_cases_with_downwash_detected = 0
    # for img_name, img in dataset['images'].items():
    #     visible_robots = [n['name'] for n in img['visible_neighbors']]
    #     for robot_causing_downwash in img['downwash_caused_by']:
    #         num_cases_with_downwash += 1
    #         if robot_causing_downwash in visible_robots:
    #             num_cases_with_downwash_detected += 1
    #             print("Downwash detected in ", img_name)
    #         else:
    #             print("Downwash not detected in ", img_name, robot_causing_downwash)

    # print("Downwash detection: {} / {}".format(num_cases_with_downwash_detected, num_cases_with_downwash))


def main():
    parser = argparse.ArgumentParser(description='Create synchronized dataset from a raw dataset')
    parser.add_argument('folder_raw', help="Path to raw dataset") 
    parser.add_argument('folder_sync', help="Path to synchronized dataset") 
    args = parser.parse_args()
    raw2sync(args.folder_raw, args.folder_sync)

if __name__ == "__main__":
    main()
