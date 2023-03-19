import argparse
import yaml
from pathlib import Path
import os
import shutil
import cv2

def clean(dataset_yaml_in, output_folder):
    robot_names = [str(f).split('/')[-1] for f in Path(dataset_yaml_in).parent.iterdir() if f.is_dir()]
    output_folder = Path(output_folder)
    if output_folder.exists() and output_folder.is_dir():
        shutil.rmtree(output_folder)
        os.mkdir(output_folder)

    for robot in robot_names:
        folder = output_folder / robot
        os.mkdir(folder)
    # dataset_yaml_out = {
    #     'calibration': {},
    #     'images': {},
    # }
    with open(dataset_yaml_in, 'r') as f:
        dataset = yaml.load(f, Loader=yaml.CSafeLoader)

    # copy calibration information and make it unique, if not identical
        # calib_name_dict = dict()
        # for calib_name in dataset['calibration']:
        #     calib_name_new = calib_name
        #     if calib_name in dataset_yaml_out['calibration']:
        #         if dataset['calibration'][calib_name] != dataset_yaml_out['calibration'][calib_name]:
        #             # calibration already exists and is different -> rename
        #             i = 1
        #             while True:
        #                 calib_name_new = calib_name + "_" + str(i)
        #                 if calib_name_new not in dataset_yaml_out['calibration']:
        #                     break
        #                 i = i + 1

        #     dataset_yaml_out['calibration'][calib_name_new] = dataset['calibration'][calib_name]
        #     calib_name_dict[calib_name] = calib_name_new
    for image_name in dataset['images']:
            # dataset_yaml_out['images'][image_name] = dataset['images'][image_name]
            image = cv2.imread(str((Path(dataset_yaml_in).parent / image_name)))
            if (output_folder / Path(image_name).parent).exists():
                 cv2.imwrite(str(Path(output_folder) / image_name), image)
            else:
                 os.makedirs(output_folder / Path(image_name).parent)
                 cv2.imwrite(str(Path(output_folder) / image_name), image)
            
    # yaml.Dumper.ignore_aliases = lambda *args : True
    # with open(Path(output_folder) / 'dataset.yaml', 'w') as outfile:
    #         yaml.dump(dataset_yaml_out, outfile)
    shutil.copy(dataset_yaml_in, output_folder / 'dataset.yaml')


def main():
    parser = argparse.ArgumentParser(description='Clean synthetic dataset')
    parser.add_argument('dataset_yaml_in', help="Path to input file")
    parser.add_argument('folder_out', help="Path to output folder")
    args = parser.parse_args()

    clean(args.dataset_yaml_in, args.folder_out)

if __name__ == "__main__":
    main()