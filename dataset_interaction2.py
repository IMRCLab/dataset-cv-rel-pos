
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


def interaction(yaml_path):

    folder = Path(yaml_path).parent

    with open(yaml_path, 'r') as f:
        dataset = yaml.load(f, Loader=yaml.CSafeLoader)

    exclude = []
    include = []

    # load existing filter, if there is one
    filter_file = folder / "filter.yaml"
    if filter_file.exists():
        with open(filter_file, 'r') as f:
            data = yaml.load(f, Loader=yaml.CSafeLoader)
        exclude = data['exclude']
        include = data['include']
        print("Found existing filter with {} exclude and {} include entries".format(len(exclude), len(include)))

    for image, value in dataset['images'].items():
        if image in exclude or image in include:
            continue
        neighbors = value['visible_neighbors']
        if len(neighbors) > 0:
            img = cv2.imread(str(folder / image))
            for neighbor in neighbors:
                xmin, ymin, xmax, ymax = neighbor['bb']
                cx, cy = neighbor['pix']
                cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 1)
                cv2.circle(img, (int(cx), int(cy)), 2, (255, 0, 0), 2)
            cv2.imshow('image', img)
            keyboard = cv2.waitKey(0) % 256
            filter_image = keyboard == ord('c') # press 'c', to not include image
            if filter_image:
                exclude.append(image)
                print("skip ", image)
            else:
                include.append(image)
                print("add ", image)
            if keyboard == ord('q'):
                break

    print("Got {} exclude and {} include entries".format(len(exclude), len(include)))

    data = {'exclude': exclude,
            'include': include}
    with open(folder / "filter.yaml", "w") as f:
        yaml.dump(data, f)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', help="dataset.yaml file")
    args = parser.parse_args()
    interaction(args.dataset)


if __name__ == "__main__":
    main()
