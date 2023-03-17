import yaml
import argparse
from pathlib import Path

def dataset_filter(dataset_in_yaml, filter_yaml, dataset_out_yaml):

    with open(dataset_in_yaml, 'r') as f:
        dataset = yaml.load(f, Loader=yaml.CSafeLoader)

    with open(filter_yaml, 'r') as f:
        data = yaml.load(f, Loader=yaml.CSafeLoader)
        exclude = data['exclude']

    for image_name in exclude:
        del dataset['images'][image_name]

    print("removed {} entries. Now has {} entries.".format(len(exclude), len(dataset['images'])))

    yaml.Dumper.ignore_aliases = lambda *args : True
    with open(dataset_out_yaml, 'w') as outfile:
            yaml.dump(dataset, outfile)

def main():
    parser = argparse.ArgumentParser(description='Apply filter.yaml for a dataset dataset')
    # parser.add_argument('dataset_in_yaml', help="Path to output file")
    # parser.add_argument('filter_yaml', help="Path to output file")
    # parser.add_argument('dataset_out_yaml', help="Path to output file")
    parser.add_argument('folder', help="Path to dataset")
    args = parser.parse_args()

    folder = Path(args.folder)
    folder_sync = folder / "Synchronized-Dataset"

    dataset_filter(folder_sync / "dataset_all.yaml", folder_sync / "filter.yaml", folder_sync / "dataset_all.yaml")

if __name__ == "__main__":
    main()
