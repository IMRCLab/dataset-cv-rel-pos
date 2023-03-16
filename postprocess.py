from pathlib import Path
import shutil
import argparse
from raw2sync_dataset import raw2sync
from dataset_annotate_bb import annotate_bb
from calibration_manual_extrinsic import compute_calibration
import dataset_interaction
import dataset_interaction2
from dataset_apply_filter import dataset_filter

def update_calibs(folder_raw, calibs):
    for cf in calibs:
        folder_raw_cf = folder_raw / cf
        folder_raw_cf_wifi = folder_raw_cf / (cf + "_wifi.csv")
        if folder_raw_cf_wifi.exists():
            compute_calibration(calibs[cf]['file'], calibs[cf]['rvec'], calibs[cf]['tvec'])
            shutil.copy(calibs[cf]['file'], folder_raw_cf / "calibration.yaml")

def postprocess(folder, keepcalib):
    folder = Path(folder)
    folder_raw = folder / "Raw-Dataset"
    if not folder_raw.exists():
        print("ERROR no raw dataset")
        exit()

    if not keepcalib:
        # copy correct calibration files
        calibs = {
            "cf6": {
                "file": "/home/whoenig/tmp/cvmrs/camera-calibration/6_intrinsic_1/cf6/calibration.yaml",
                "rvec": [5, 0, 90],            # deg (up/down, ?, left/right)
                "tvec": [0.0, 0.0, -0.025],
            },
            "cf7": {
                "file": "/home/whoenig/tmp/cvmrs/camera-calibration/7_intrinsic_1/cf7/calibration.yaml",
                "rvec": [94, -4.4, 87.4],            # deg (up/down, ?, left/right)
                "tvec": [0.0, 0.0, -0.025],
            },
            "cf8": {
                "file": "/home/whoenig/tmp/cvmrs/camera-calibration/8_intrinsic_2/cf8/calibration.yaml",
                "rvec": [48, -3, 87],            # deg (up/down, ?, move left: increase)
                "tvec": [0.0, 0.0, -0.025],
            },
        }

        update_calibs(folder_raw, calibs)

    # remove synchronized-dataset
    folder_sync = folder / "Synchronized-Dataset"
    if folder_sync.exists():
        shutil.rmtree(folder_sync)

    # rerun raw2sync
    raw2sync(folder_raw, folder_sync)

    if not keepcalib:
        # run interactive part1
        new_rpy = dataset_interaction.interaction(folder_sync / "dataset_all.yaml")
        for calib_name, rpy in new_rpy.items():
            calibs[calib_name]["rvec"] = rpy
        print(calibs)
        update_calibs(folder_raw, calibs)
        shutil.rmtree(folder_sync)
        raw2sync(folder_raw, folder_sync)

        # run interactive part2
        dataset_interaction2.interaction(folder_sync / "dataset_all.yaml")

        # filter dataset
        dataset_filter(folder_sync / "dataset_all.yaml", folder_sync / "filter.yaml", folder_sync / "dataset_all.yaml")

    # remove bb subfolders
    folder_bb = folder / "bb"
    for i in range(1,10):
        folder_bb_sub = folder_bb / str(i)
        if folder_bb_sub.exists():
            shutil.rmtree(folder_bb_sub)

    # run bb annotation
    annotate_bb(folder_sync / "dataset_all.yaml", folder_bb)


def main():
    parser = argparse.ArgumentParser(description='Create synchronized dataset from a raw dataset')
    parser.add_argument('folder', help="Path to dataset")
    parser.add_argument('--keepcalib', action='store_true')
    args = parser.parse_args()
    postprocess(args.folder, args.keepcalib)

if __name__ == "__main__":
    main()
