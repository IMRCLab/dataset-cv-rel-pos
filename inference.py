import argparse
from pathlib import Path
import tempfile
import os
import shutil

import sys
sys.path.insert(0, str(Path(__file__).parent / "baselines/yolov3"))
from baselines.yolov3.get_yolo_labels import get_yolo_labels as yolo_create_labels
from baselines.yolov3.inference import run as yolo_inference

sys.path.insert(0, str(Path(__file__).parent / "baselines/deepMulti_robot"))
from baselines.deepMulti_robot.get_locanet_labels import get_labels as locanet_create_labels
from baselines.deepMulti_robot.inference import inference as locanet_inference


def inference(dataset_yaml, model_name, backends):

    # testset_name = Path(dataset_yaml).parent.parent.name
    testset_name = Path(dataset_yaml).stem
    models_folder = Path(__file__).parent / "models"

    result_folder = Path(__file__).parent / "results" / model_name / testset_name
    os.makedirs(result_folder, exist_ok=True)
    shutil.copy(dataset_yaml, result_folder / "groundtruth.yaml")
    for backend in backends:
        print(backend)
        if backend == "yolo":
            # python3 get_yolo_labels.py -f /home/whoenig/imrc/cv-mrs/dataset/synth/Pablo-test/Synchronized-Dataset/dataset_filtered.yaml -mode test
            # python3 inference.py /home/whoenig/imrc/cv-mrs/dataset/synth/Pablo-test/ --weights runs/train/synth-blender1-3k2/weights/best.pt
            # 
            with tempfile.TemporaryDirectory() as tmpdirname:
                # convert dataset to yolo format
                yolo_create_labels(dataset_yaml, tmpdirname, "test")
                # run inference
                model = models_folder / "yolo" / model_name / "weights/best.pt"
                yolo_inference(tmpdirname + "/", str(model), (320, 320))
                # save the resulting file
                result_yaml = Path(tmpdirname) / "yolov3/inference_yolo.yaml"
                result_target = result_folder / "yolo.yaml"
                shutil.copy(result_yaml, result_target)
        elif backend == "locanet":
             with tempfile.TemporaryDirectory() as tmpdirname:
                # convert dataset to locanet format
                locanet_create_labels(dataset_yaml, tmpdirname, "test")
                # run inference
                model = models_folder / "locanet" / model_name
                locanet_inference(Path(tmpdirname) / "locanet" / "test.txt", model)
                # save the resulting file
                result_yaml = Path(tmpdirname) / "locanet/inference_locanet.yaml"
                result_target = result_folder / "locanet.yaml"
                shutil.copy(result_yaml, result_target)
        else:
            raise Exception("Unknown backend!")

    # del sys.path[0:2]
    # print(sys.path)
    # from plot_results import plot_results
    # plot_results(result_folder)

    import subprocess
    subprocess.run([
        "python3", "plot_results.py",
        result_folder
        ],
        cwd= Path(__file__).parent,
        stderr=sys.stderr, stdout=sys.stdout)


def main():
    parser = argparse.ArgumentParser(description='Run inference for a given model on a dataset')
    parser.add_argument('dataset_yaml', help="Path to dataset.yaml file")
    parser.add_argument('model', help="Name of the model")
    parser.add_argument('-b', default=["yolo", "locanet"], type=str, nargs='+', help="Backends to use")
    args = parser.parse_args()

    inference(args.dataset_yaml, args.model, args.b)

if __name__ == "__main__":
    main()
