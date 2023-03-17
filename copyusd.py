from pathlib import Path
import shutil
import yaml
import argparse
import os


def postprocess(folder):

    with open(Path(__file__).parent / "copyusd.yaml") as f:
        cfg = yaml.safe_load(f)

    cards = [f.parent for f in Path("/media").glob("**/config.txt")]
    print("Using ", cards[0])

    for name, data in cfg["experiments"].items():
        print("Experiment ", name)
        target = Path(folder) / name / "Raw-Dataset"
        if not target.exists():
            print("Wrong folder?")
            continue
        for fname in data["usd"]:
            src = cards[0] / fname
            if src.exists():
                cfname = fname.split("_")[0]
                dst = Path(folder) / name / "Raw-Dataset" / cfname
                print("copy ", src, " to ", dst)
                os.makedirs(dst, exist_ok=True)
                shutil.copy(src, dst)


def main():
    parser = argparse.ArgumentParser(description='Create synchronized dataset from a raw dataset')
    parser.add_argument('folder', help="Path to dataset")
    args = parser.parse_args()
    postprocess(args.folder)

if __name__ == "__main__":
    main()
