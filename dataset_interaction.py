
import cv2
import shutil
import os
import yaml
import argparse
import fnmatch
import random
import numpy as np
from pathlib import Path
import utils
import itertools
import rowan


def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        params["lb"] = (x,y)
        cv2.destroyAllWindows()


def interaction(yaml_path):

    folder = Path(yaml_path).parent

    with open(yaml_path, 'r') as f:
        dataset = yaml.load(f, Loader=yaml.CSafeLoader)

    # Repeat calibration for each used camera in the dataset
    result = dict()
    for calib_name, calib in dataset["calibration"].items():
        print("Considering ", calib_name)

        # find images with 1 neighbor
        centers = []
        for image, value in dataset['images'].items():
            calib_name_img = value["calibration"]
            if calib_name == calib_name_img:
                neighbors = value['visible_neighbors']
                if len(neighbors) >= 1:
                    cx, cy = neighbors[0]['pix']
                    centers.append((cx, cy, image))

        if len(centers) < 5:
            print("Skipping... ", len(centers))
            continue

        # extract 5 images, with "diverse centers"
        picked_centers = [] #[random.choice(centers)] # pick first one randomly
        do_not_use = []
        # print("pick ", picked_centers)
        while len(picked_centers) < 5:
            # we want to maximize the minimum distance
            maxdist = 0
            maxentry = None
            for cx, cy, image in centers:
                if image in [name for _,_,name,_,_ in picked_centers] or image in do_not_use:
                    continue
                # print("consider ", image)

                mindist = np.inf
                minentry = None
                for pcx, pcy, pimg, _, _ in picked_centers:
                    dist = np.linalg.norm(np.array([cx,cy]) - np.array([pcx,pcy]))
                    if dist < mindist:
                        mindist = dist
                        minentry = (pcx, pcy, pimg)
                        # print("new min ", dist, minentry, np.array([cx,cy]) - np.array([pcx,pcy]))
                if mindist > maxdist:
                    maxdist = mindist
                    maxentry = (cx, cy, image)
                    # print("new max ", maxentry)

            # ask the user for input
            if maxentry is None:
                print("Warning - not enough data points!")
                break
            cx, cy, image = maxentry
            img = cv2.imread(str(folder / image))
            cv2.circle(img, (int(cx), int(cy)), 2, (255, 0, 0), 2)
            rect = cv2.selectROI('image', img)
            if rect == (0,0,0,0):
                do_not_use.append(image)
                print("user abort")
            else:
                cxu, cyu = rect[0] + rect[2] / 2, rect[1] + rect[3] / 2
                print(rect, cx, cy)
                picked_centers.append((cx, cy, image, cxu, cyu))

        # # extract 5 images,randomly
        # random.shuffle(centers)
        # picked_centers = []
        # while len(picked_centers) < 5:
        #     cx, cy, image = centers[0]

        #     img = cv2.imread(str(folder / image))
        #     cv2.circle(img, (int(cx), int(cy)), 2, (255, 0, 0), 2)
        #     rect = cv2.selectROI('image', img)
        #     if rect == (0,0,0,0):
        #         print("user abort")
        #     else:
        #         cxu, cyu = rect[0] + rect[2] / 2, rect[1] + rect[3] / 2
        #         print(rect, cx, cy)
        #         picked_centers.append((cx, cy, image, cxu, cyu))
        #     del centers[0]

        # print(picked_centers)

        # picked_centers = [(32.0, 221.0, '1/cf8_00099.jpg', 34.5, 230.5), (296.0, 208.0, '1/cf8_00117.jpg', 298.5, 191.5), (72.0, 47.0, '1/cf8_00121.jpg', 62.5, 58.0), (241.0, 63.0, '1/cf8_00124.jpg', 228.5, 62.5), (157.0, 178.0, '1/cf8_00148.jpg', 157.5, 179.0)]

        # picked_centers = [(237.0, 268.0, '1/cf8_00079.jpg', 257.5, 257.5), (32.0, 39.0, '1/cf8_00134.jpg', 59.5, 59.5)]

        # picked_centers = [(237.0, 268.0, '1/cf8_00079.jpg', 254.0, 256.5), (32.0, 39.0, '1/cf8_00134.jpg', 23.5, 51.5), (60.0, 239.0, '1/cf8_00156.jpg', 85.0, 240.5), (197.0, 83.0, '1/cf8_00207.jpg', 186.0, 78.0), (138.0, 172.0, '1/cf8_00248.jpg', 147.5, 165.5)]

        # picked_centers = [(237.0, 268.0, '1/cf8_00079.jpg', 254.5, 258.0), (32.0, 39.0, '1/cf8_00134.jpg', 24.0, 49.0), (60.0, 239.0, '1/cf8_00156.jpg', 85.0, 240.0), (197.0, 83.0, '1/cf8_00207.jpg', 186.0, 76.5), (186.0, 197.0, '1/cf8_00163.jpg', 197.5, 183.5)]

        # optimization
        rs = np.linspace(-5, 5, 10)
        ps = np.linspace(-5, 5, 10)
        ys = np.linspace(-5, 5, 10)

        t_v = np.array(calib['tvec'])
        r_v = np.array(calib['rvec'])
        int_mtrx = np.array(calib['camera_matrix'])
        dist_v = np.array(calib['dist_coeff'])
        q = utils.opencv2quat(r_v)
        rpy = np.degrees(rowan.to_euler(q, convention="xyz"))
        print("Nominal settings", rpy)

        besterror = np.inf
        bestresult = None
        for r, p, y in itertools.product(rs, ps, ys):

            # augment calibration
            augrpy = np.radians(rpy + np.array([r, p, y]))
            augq = rowan.from_euler(augrpy[0], augrpy[1], augrpy[2], convention="xyz")
            T_robot1_to_camera = utils.pose2transform(t_v, augq)

            error = 0
            for _,_,image,cxu,cyu in picked_centers:
                entry = dataset['images'][image]

                pose = np.array(entry["pose"])
                T_robot1_to_world = utils.pose2transform(pose[0:3], pose[3:7])
                # T_camera_to_world = T_robot1_to_world @ T_camera_to_robot1
                # robot_1_pos_in_world = pose[0:3]
                robot_k_pos_in_world = np.array(entry["visible_neighbors"][0]["pos_world"])

                robot_k_in_cam = utils.apply_transform(T_robot1_to_camera @ np.linalg.inv(T_robot1_to_world), robot_k_pos_in_world)

                pixels, _ = cv2.projectPoints(robot_k_in_cam, np.zeros(3), np.zeros(3), int_mtrx, dist_v)
                # print(pixels, entry["visible_neighbors"][0]['pix'])

                # compute distance to what we want
                pixels = np.array(pixels).flatten()
                pixels_desired = np.array([cxu,cyu])
                error += np.linalg.norm(pixels - pixels_desired)
            # print(error, r, p, y)
            if error < besterror:
                besterror = error
                bestresult = (r, p, y)

        print(bestresult, besterror)

        r, p, y = bestresult
        bestrpy = rpy + np.array([r, p, y])
        print(bestrpy)
        result[calib_name] = bestrpy

    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', help="dataset.yaml file")
    args = parser.parse_args()
    interaction(args.dataset)


if __name__ == "__main__":
    main()
