import yaml
# import numpy as np
import argparse
from collections import defaultdict

def stats(dataset_file):

    with open(dataset_file, 'r') as f:
        dataset = yaml.load(f, Loader=yaml.CSafeLoader)

    visible_neighbors = defaultdict(int)
    num_cases_with_downwash = 0
    num_cases_with_downwash_detected = 0
    for image_name in dataset['images']:
        entry = dataset['images'][image_name]
        num_visible_neighbors = len(entry['visible_neighbors'])
        visible_neighbors[num_visible_neighbors] += 1

    #     visible_robots = [n['name'] for n in entry['visible_neighbors']]
    #     for robot_causing_downwash in entry['downwash_caused_by']:
    #         num_cases_with_downwash += 1
    #         if robot_causing_downwash in visible_robots:
    #             num_cases_with_downwash_detected += 1
    #             print("Downwash detected in ", image_name)
    #         else:
    #             print("Downwash not detected in ", image_name, robot_causing_downwash)

    # print("Downwash detection: {} / {}".format(num_cases_with_downwash_detected, num_cases_with_downwash))

    print("visible neighbors", visible_neighbors)

def main():
    parser = argparse.ArgumentParser(description='Show statistics of a particular dataset.yaml file')
    parser.add_argument('dataset_yaml', help="Path to output file")
    args = parser.parse_args()
    stats(args.dataset_yaml)

if __name__ == "__main__":
    main()
