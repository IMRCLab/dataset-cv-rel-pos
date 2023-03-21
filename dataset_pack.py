import argparse
import yaml
from pathlib import Path
import os
import shutil
import cv2

def clean(dataset_yaml_in, dataset_yaml_out):
    output_folder = Path(dataset_yaml_out).parent
    os.mkdir(output_folder)    
    result = {
        'calibration': {},
        'images': {},
    }
    with open(dataset_yaml_in, 'r') as f:
        dataset = yaml.load(f, Loader=yaml.CSafeLoader)

    result['calibration'] = dataset['calibration']
    for full_image_name in dataset['images']:
        image_name = Path(full_image_name).name
        image_name_new = image_name
        if (output_folder / Path(image_name_new)).exists():
            j = 1
            while True:
                image_name_new = image_name[:-4] + "_" + str(j) + image_name[-4:]
                if image_name_new not in result['images']:
                    break
                j = j + 1
        shutil.copy(str((Path(dataset_yaml_in).parent / full_image_name)),str(Path(output_folder) / image_name_new))
        rel_image_name = str((Path(output_folder) / image_name_new).relative_to(Path(dataset_yaml_out).parent))
        result['images'][rel_image_name] = dataset['images'][full_image_name]

            
    yaml.Dumper.ignore_aliases = lambda *args : True
    with open(Path(dataset_yaml_out), 'w') as outfile:
            yaml.dump(result, outfile)
    


def main():
    parser = argparse.ArgumentParser(description='Clean synthetic dataset')
    parser.add_argument('dataset_yaml_in', help="Path to input file")
    parser.add_argument('dataset_yaml_out', help="Path to output file")
    args = parser.parse_args()

    clean(args.dataset_yaml_in, args.dataset_yaml_out)

if __name__ == "__main__":
    main()