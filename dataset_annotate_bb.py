
import cv2
import shutil
import os
import yaml
import argparse
import fnmatch
from pathlib import Path

def annotate_bb(yaml_path, output_folder):

    folder = Path(yaml_path).parent

    with open(yaml_path, 'r') as f:
        dataset = yaml.load(f, Loader=yaml.CSafeLoader)

    for image, value in dataset['images'].items():
        neighbors = value['visible_neighbors']
        if len(neighbors) > 0:
            img = cv2.imread(str(folder / image))
            for neighbor in neighbors:
                xmin, ymin, xmax, ymax = neighbor['bb']
                cx, cy = neighbor['pix']
                cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 2)
                cv2.circle(img, (int(cx), int(cy)), 2, (255, 0, 0), 2)
                # for cx_corner, cy_corner in neighbor['corners']:
                #     cv2.circle(img, (int(cx_corner), int(cy_corner)), 2, (100, 100, 0), 2)
            os.makedirs((Path(output_folder) / image).parent, exist_ok=True)
            cv2.imwrite(str(Path(output_folder) / image), img)
            # print(image)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', help="dataset.yaml file")
    parser.add_argument('output', help="output folder")
    args = parser.parse_args()
    annotate_bb(args.dataset, args.output)


if __name__ == "__main__":
    main()
