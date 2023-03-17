import argparse
from pathlib import Path
import tempfile
import os
import shutil
import yaml
import subprocess

import sys
sys.path.insert(0, str(Path(__file__).parent / "baselines/yolov3"))
from baselines.yolov3.get_yolo_labels import get_yolo_labels as yolo_create_labels

sys.path.insert(0, str(Path(__file__).parent / "baselines/deepMulti_robot"))
from baselines.deepMulti_robot.get_locanet_labels import get_labels as locanet_create_labels
from baselines.deepMulti_robot.train import train as locanet_train


def train(dataset_yaml, model_name, backends, initial_model):

    models_folder = (Path(__file__).parent / "models").absolute()
    for backend in backends:
        if backend == "yolo":
      
            with tempfile.TemporaryDirectory() as tmpdirname:
                # convert dataset to yolo format
                yolo_create_labels(dataset_yaml, tmpdirname, "train")
                # create cf.yaml file
                cfg = {
                    'train': tmpdirname + "/yolov3/images/train/",
                    'val': tmpdirname + "/yolov3/images/val/",
                    'nc': 1,
                    'names': ['crazyflie']
                }
                with open(Path(tmpdirname) / "cfg.yaml", "w") as f:
                    yaml.dump(cfg, f)

                args = [
                    "python3", "train.py",
                    "--img", str(320),
                    "--batch", str(8),
                    "--epochs", str(1),
                    "--exist-ok",
                    "--data", str(Path(tmpdirname) / "cfg.yaml"),
                    "--project", str(models_folder / "yolo"),
                    "--name", model_name
                    ]
                if initial_model is not None:
                    args.extend(["--weights", str(models_folder / "yolo" / initial_model / "weights" / "best.pt")])

                # run training
                subprocess.run(args,
                    cwd= Path(__file__).parent / "baselines/yolov3",
                    stderr=sys.stderr, stdout=sys.stdout)

        elif backend == "locanet":
             with tempfile.TemporaryDirectory() as tmpdirname:
                # convert dataset to yolo format
                locanet_create_labels(dataset_yaml, tmpdirname, "train")
                # create cf.yaml file

                if initial_model is None or initial_model == "":
                    initial_weights = ""
                else:
                    initial_weights = models_folder / "locanet" / initial_model

                cfg = {
                    'INPUT_CHANNEL': 1, # RGB: 3, Grey: 1
                    'LOCA_STRIDE':   8,
                    'LOCA_CLASSES': {0: "crazyflie"},
                    'TRAIN_ANNOT_PATH': "{}/locanet/train.txt".format(tmpdirname),
                    'TRAIN_BATCH_SIZE': 8,
                    'TRAIN_INPUT_SIZE': [320, 320],
                    'TRAIN_LR_INIT': 1e-3,
                    'TRAIN_LR_END': 1e-6,
                    'TRAIN_WARMUP_EPOCHS': 2,
                    'TRAIN_EPOCHS': 1,
                    'OUTPUT_FILE': model_name,
                    'WEIGHT_PATH': str(models_folder / "locanet") + "/",
                    'INITIAL_WEIGHTS': str(initial_weights),
                }
                with open(Path(tmpdirname) / "cfg.yaml", "w") as f:
                    yaml.dump(cfg, f)

                # train!
                # locanet_train(Path(tmpdirname) / "cfg.yaml")

                # # run training
                # subprocess.run([
                #     "python3", "train.py",
                #     str(Path(tmpdirname) / "cfg.yaml"),
                #     ],
                #     cwd= Path(__file__).parent / "baselines/deepMulti_robot",
                #     stderr=sys.stderr, stdout=sys.stdout)

                # run training
                subprocess.run([
                    "./train.sh",
                    str(Path(tmpdirname) / "cfg.yaml"),
                    ],
                    cwd= Path(__file__).parent / "baselines/deepMulti_robot",
                    stderr=sys.stderr, stdout=sys.stdout)
        else:
            raise Exception("Unknown backend!")


def main():
    parser = argparse.ArgumentParser(description='Run inference for a given model on a dataset')
    parser.add_argument('dataset_yaml', help="Path to dataset.yaml file")
    parser.add_argument('model', help="Name of the model")
    parser.add_argument('-initial_model', default=None, help="Name of a model to start with")
    parser.add_argument('-b', default=["yolo", "locanet"], type=str, nargs='+', help="Backends to use")
    args = parser.parse_args()

    # flatten argument
    # flat_list = [item for sublist in args.n for item in sublist]

    train(args.dataset_yaml, args.model, args.b, args.initial_model)

if __name__ == "__main__":
    main()
