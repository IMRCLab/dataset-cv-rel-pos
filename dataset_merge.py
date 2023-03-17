import yaml
import argparse
from pathlib import Path

def merge(inputs, dataset_yaml):

    # get a list of all files
    dataset_files = []
    for input in inputs:
        p = Path(input)
        # if it's a folder, search recursively
        if p.is_dir():
            print("searching ", p)
            dataset_files.extend(p.glob("*/**/dataset_all.yaml"))
        else:
            dataset_files.append(p)

    result = {
        'calibration': {},
        'images': {},
    }

    for dataset_file in dataset_files:
        print(dataset_file)

        with open(dataset_file, 'r') as f:
            dataset = yaml.load(f, Loader=yaml.CSafeLoader)

        # copy calibration information and make it unique, if not identical
        calib_name_dict = dict()
        for calib_name in dataset['calibration']:
            calib_name_new = calib_name
            if calib_name in result['calibration']:
                if dataset['calibration'][calib_name] != result['calibration'][calib_name]:
                    # calibration already exists and is different -> rename
                    i = 1
                    while True:
                        calib_name_new = calib_name + "_" + str(i)
                        if calib_name_new not in result['calibration']:
                            break
                        i = i + 1

            result['calibration'][calib_name_new] = dataset['calibration'][calib_name]
            calib_name_dict[calib_name] = calib_name_new

        # copy image part, update calibration information
        for image_name in dataset['images']:
            # compute relative path
            rel_image_name = str((Path(dataset_file).parent / image_name).relative_to(Path(dataset_yaml).parent))
            # print(rel_image_name)

            result['images'][rel_image_name] = dataset['images'][image_name]
            result['images'][rel_image_name]['calibration'] = calib_name_dict[dataset['images'][image_name]['calibration']]


    yaml.Dumper.ignore_aliases = lambda *args : True
    with open(dataset_yaml, 'w') as outfile:
            yaml.dump(result, outfile)

def main():
    parser = argparse.ArgumentParser(description='Create synchronized dataset from a raw dataset')
    parser.add_argument('input', type=str, nargs='+', action='append', help="Folder to recursively search, or file")
    parser.add_argument('dataset_yaml', help="Path to output file")
    args = parser.parse_args()

    # flatten argument
    inputs = [item for sublist in args.input for item in sublist]

    merge(inputs, args.dataset_yaml)

if __name__ == "__main__":
    main()
