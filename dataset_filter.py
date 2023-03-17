import yaml
import argparse
import random
from pathlib import Path
from collections import defaultdict

def split(dataset_yaml_in, dataset_yaml_out, num_max_files, dataset_yaml_out2=None):

    with open(dataset_yaml_in, 'r') as f:
        dataset = yaml.load(f, Loader=yaml.CSafeLoader)

    # filter images
    filtered_by_num_neighbors = defaultdict(list)
    for image_name in dataset['images']:
        entry = dataset['images'][image_name]
        num_neighbors = len(entry['visible_neighbors'])
        if num_neighbors < len(num_max_files):
            filtered_by_num_neighbors[num_neighbors].append(image_name)

    # shuffle and limit to number of files
    filtered = []
    remainder = []
    for num_neighbors, entry in filtered_by_num_neighbors.items():
        random.shuffle(entry)
        if len(entry) < num_max_files[num_neighbors]:
            print("Warning, only using {} files for {} robots".format(len(entry), num_neighbors))
        else:
            remainder.extend(entry[num_max_files[num_neighbors]:])
        filtered.extend(entry[0:num_max_files[num_neighbors]])

    random.shuffle(filtered)
    random.shuffle(remainder)

    # copy relevant data
    def save(filtered, dataset_yaml_out):
        result = {
            'calibration': {},
            'images': {},
        }

        required_calibrations = set()
        for image_name in filtered:
            rel_image_name = str((Path(dataset_yaml_in).parent / image_name).relative_to(Path(dataset_yaml_out).parent))
            entry = dataset['images'][image_name]
            result['images'][rel_image_name] = entry
            required_calibrations.add(entry['calibration'])

        # copy required calibration information
        for calib_name in required_calibrations:
            result['calibration'][calib_name] = dataset['calibration'][calib_name]

        yaml.Dumper.ignore_aliases = lambda *args : True
        with open(dataset_yaml_out, 'w') as outfile:
                yaml.dump(result, outfile)

    save(filtered, dataset_yaml_out)
    if dataset_yaml_out2 is not None:
        save(remainder, dataset_yaml_out2)


def main():
    parser = argparse.ArgumentParser(description='Create synchronized dataset from a raw dataset')
    parser.add_argument('dataset_yaml_in', help="Path to input file")
    parser.add_argument('dataset_yaml_out', help="Path to output file")
    parser.add_argument('-remainder', default=None, help="Path to output file")
    parser.add_argument('-n', type=int, nargs='+', action='append', help="maximum number of images to take")
    args = parser.parse_args()

    # flatten argument
    flat_list = [item for sublist in args.n for item in sublist]

    split(args.dataset_yaml_in, args.dataset_yaml_out, flat_list, args.remainder)

if __name__ == "__main__":
    main()
