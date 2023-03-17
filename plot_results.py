import yaml
import argparse
import numpy as np
from pathlib import Path
from munkres import Munkres
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt
from collections import defaultdict
import utils
import rowan


def compute_success_rate(ground_truth, predictions):
    success_rate_pos, success_rate_neg, success_rate = np.zeros(5), np.zeros(5), 0
    for image_name, entry in ground_truth["images"].items():
        gt_neighbors = entry["visible_neighbors"]

        other_entry = predictions["images"][image_name]
        other_neighbors = other_entry["visible_neighbors"]

        if len(gt_neighbors) == len(other_neighbors):
            success_rate += 1
            success_rate_pos[len(gt_neighbors)] += 1
        else:
            success_rate_neg[len(gt_neighbors)] -= 1

    return success_rate_pos, success_rate_neg, success_rate

def find_euc(pr_list, gt_list):
    matrix = []
    for i in range(len(gt_list)):
        tmp_list = []
        for j in pr_list:
            tmp_list.append(np.linalg.norm(np.asarray(gt_list[i]) - np.asarray(j)))
        matrix.append(tmp_list)
    return matrix
# # returns indices for each row and total sum of cost
# def hungarian(m,mx):
#     indexes = m.compute(mx)
#     total = 0
#     for row, column in indexes:
#         value = mx[row][column]
#         total += value
#     return indexes, total
    

def plot_results(folder):

    # change settings to match latex
    plt.rcParams.update({
        "text.usetex": True,
        "font.family": "sans-serif",
        "font.sans-serif": "Helvetica",
        "font.size": 12,
        "figure.figsize": (6, 4),
    })

    folder = Path(folder)

    yaml_files = folder.glob("*.yaml")

    results = dict()
    for yaml_file in yaml_files:
        with open(yaml_file, 'r') as stream:
            result = yaml.load(stream, Loader=yaml.CSafeLoader)
            results[yaml_file.stem] = result

    gt = results["groundtruth"]
    del results["groundtruth"]

    with PdfPages(folder / 'result.pdf') as pdf:

        # # success rate
        # fig, axs = plt.subplots(1, len(results), sharey=True, sharex=True, squeeze=False)
        # for k, (name, entry) in enumerate(results.items()):
        #     success_rate_pos, success_rate_neg, success_rate = compute_success_rate(gt, entry)
        #     print("Success rate of {}  is {:.1f} percent for {} images.".format(name, success_rate*100/len(gt["images"]), len(gt["images"])))

        #     axs[0,k].set_title(name, fontsize=10)
        #     l2 = axs[0,k].bar([*range(0, len(success_rate_pos.tolist()), 1)], success_rate_pos.tolist(),width=0.90, color='b',label='positive')[0]
        #     axs[0,k].bar([*range(0, len(success_rate_neg.tolist()), 1)], success_rate_neg.tolist(),width=0.90, color='r',label='negative')
        #     fig.gca().set_xticks([*range(0, len(success_rate_pos.tolist()), 1)])
        # fig.text(0.5, 0.04, 'Number of Robots', va='center', ha='center')
        # fig.text(0.04, 0.5, 'Number of Images', va='center', ha='center', rotation='vertical')
        # # line_labels = ["Positive", "Negative"]
        # # fig.legend([l1, l2], labels=line_labels, loc="upper right", borderaxespad=0.1)
        # pdf.savefig(fig)
        # plt.close()

        # success rate (new)
        success = dict()
        for k, (name, entry) in enumerate(results.items()):
            success_rate_pos, success_rate_neg, success_rate = compute_success_rate(gt, entry)
            success[name] = success_rate_pos / (success_rate_pos - success_rate_neg) * 100
            print("Success rate of {}  is {:.1f} percent for {} images.".format(name, success_rate*100/len(gt["images"]), len(gt["images"])))

        width = 0.8 / len(results)

        fig, ax = plt.subplots(constrained_layout=True)

        offset = -1.5*width
        for name, data in success.items():
            offset += width
            rects = ax.bar(np.arange(len(data)) + offset, data, width, label=name)
            ax.bar_label(rects, fmt="%.0f" ,padding=3, fontsize=10)

        # Add some text for labels, title and custom x-axis tick labels, etc.
        ax.set_xticks([0, 1, 2, 3])
        ax.set_ylabel('Correct number predicted [\%]')
        ax.set_xlabel('Number of neighboring robots')
        ax.legend(loc='upper center', ncol=2)
        ax.set_ylim(0,110)

        pdf.savefig(fig)
        plt.close()

        # success rate by classification error
        success_by_rel_error = dict()
        for name in results.keys():
            success_by_rel_error[name] = defaultdict(int)

        for image_name, entry in gt["images"].items():
            gt_neighbors = entry["visible_neighbors"]

            for name, other_entry in results.items():
                other_neighbors = other_entry["images"][image_name]["visible_neighbors"]
                index = len(gt_neighbors) - len(other_neighbors)
                success_by_rel_error[name][index] += 1
        print(success_by_rel_error)

        width = 0.25  # the width of the bars
        multiplier = 0

        fig, ax = plt.subplots(constrained_layout=True)

        for name, data in success_by_rel_error.items():
            offset = width * multiplier
            print(np.array(list(data.keys())))
            rects = ax.bar(np.array(list(data.keys())) + offset, data.values(), width, label=name)
            ax.bar_label(rects, padding=3)
            multiplier += 1

        # Add some text for labels, title and custom x-axis tick labels, etc.
        ax.set_ylabel('Number of pictures')
        ax.set_xlabel('Misclassification (num neighbors - num neighbors predicted)')
        # ax.set_title('Penguin attributes by species')
        ax.legend()

        pdf.savefig(fig)
        plt.close()

        # eucl. error
        eucl_error = dict()
        eucl_dist = dict()
        for name in results.keys():
            eucl_error[name] = []
            eucl_dist[name] = []

        for image_name, entry in gt["images"].items():
            gt_neighbors = entry["visible_neighbors"]

            # skip images with no robot
            if len(gt_neighbors) == 0:
                continue

            # skip images, where not all methods found the right number of robots
            skip = False
            for name, other_entry in results.items():
                other_neighbors = other_entry["images"][image_name]["visible_neighbors"]
                if len(gt_neighbors) != len(other_neighbors):
                    skip = True
                    break

            if skip:
                continue

            # compute errors
            gt_positions = [e['pos'] for e in gt_neighbors]

            for name, other_entry in results.items():
                other_neighbors = other_entry["images"][image_name]["visible_neighbors"]
                other_positions = [e['pos'] for e in other_neighbors]

                m = Munkres()
                # row: gt_position to other_position
                # i.e., matrix[i][j] means dist from gt_position[i] to other_position[j]
                matrix = find_euc(other_positions, gt_positions) # matrix of Euclidean distance between all prediction vs. g-t
                # optimal assignment
                indexes = m.compute(matrix)
                for row, column in indexes:
                    cost = matrix[row][column]
                    true_dist = np.linalg.norm(gt_positions[row])
                    pred_dist = np.linalg.norm(other_positions[column])
                    eucl_error[name].append(cost)
                    eucl_dist[name].append((true_dist, pred_dist - true_dist))

        pos = np.arange(len(results)) * 0.3
        fig, ax = plt.subplots(1, 1, constrained_layout=True, figsize=(2,4))
        ax.boxplot([e for e in eucl_error.values()], positions=pos)
        ax.set_ylabel("Euclidean Error [m]")
        ax.set_xticks(pos, [e for e in eucl_error.keys()])
        ax.set_xticklabels([e for e in eucl_error.keys()])
        ax.set_xlim(-0.1,0.4)
        pdf.savefig(fig)
        plt.close()

        # Eucl. error by gt distance
        fig, ax = plt.subplots(1, 1)
        for name, data in eucl_dist.items():
            data = np.array(data)
            ax.scatter(data[:,0], data[:,1], label=name, alpha=0.5)
            # ax.hist2d(data[:,0], data[:,1], label=name)
        ax.set_xlabel("Distance to neighbor [m]")
        ax.set_ylabel("Predicted distance - distance [m]")
        ax.legend()
        pdf.savefig(fig)
        plt.close()

        # Downwash
        for camera_angle in [0, 45, 90]:
            num_downwash_cases = 0
            num_downwash_true_pos = defaultdict(int)
            num_downwash_false_neg = defaultdict(int)
            num_downwash_false_pos = defaultdict(int)
            
            for image_name, entry in gt["images"].items():

                calib_name = entry["calibration"]
                calib = gt["calibration"][calib_name]
                t_v = np.array(calib['tvec'])
                r_v = np.array(calib['rvec'])
                q = utils.opencv2quat(r_v)
                rpy = np.degrees(rowan.to_euler(q, convention="xyz"))
                if np.abs(rpy[0] - camera_angle) > 10:
                    continue 

                T_robot1_to_camera = utils.pose2transform(t_v, q)
                T_camera_to_robot1 = np.linalg.inv(T_robot1_to_camera)
                pose = np.array(entry["pose"])
                T_robot1_to_world = utils.pose2transform(pose[0:3], pose[3:7])
                T_camera_to_world = T_robot1_to_world @ T_camera_to_robot1
                robot_1_pos_in_world = pose[0:3]

                gt_neighbors = entry["visible_neighbors"]
                has_downwash = False
                for neighbor in gt_neighbors:
                    robot_k_in_cam = np.array(neighbor["pos"])
                    robot_k_pos_in_world = utils.apply_transform(T_camera_to_world, robot_k_in_cam)

                    if utils.check_downwash(robot_k_pos_in_world, robot_1_pos_in_world):
                        has_downwash = True
                        print(image_name)
                        # gt_num_downwash += 1
                if has_downwash:
                    num_downwash_cases += 1

                for name, other_entry in results.items():
                    predict_downwash = False
                    other_neighbors = other_entry["images"][image_name]["visible_neighbors"]
                    for neighbor in other_neighbors:
                        robot_k_in_cam = np.array(neighbor["pos"])
                        robot_k_pos_in_world = utils.apply_transform(T_camera_to_world, robot_k_in_cam)

                        if utils.check_downwash(robot_k_pos_in_world, robot_1_pos_in_world):
                            # num_downwash[name] += 1
                            predict_downwash = True
                    if has_downwash and predict_downwash:
                        num_downwash_true_pos[name] += 1
                    if has_downwash and not predict_downwash:
                        num_downwash_false_neg[name] += 1
                    if not has_downwash and predict_downwash:
                        num_downwash_false_pos[name] += 1

            # print(num_downwash_cases)
            # print(num_downwash_true_pos)
            # print(num_downwash_false_neg)
            # print(num_downwash_false_pos)

            dw_result = dict()
            for name, _ in results.items():
                if num_downwash_true_pos[name] + num_downwash_false_pos[name] > 0:
                    precision = num_downwash_true_pos[name] / (num_downwash_true_pos[name] + num_downwash_false_pos[name])
                else:
                    precision = np.nan
                if num_downwash_true_pos[name] + num_downwash_false_neg[name] > 0:
                    recall = num_downwash_true_pos[name] / (num_downwash_true_pos[name] + num_downwash_false_neg[name])
                else:
                    recall = np.nan

                dw_result[name] = [precision, recall]

            multiplier = 0
            width = 0.2  # the width of the bars
            fig, ax = plt.subplots(constrained_layout=True)
            for name, data in dw_result.items():
                offset = width * multiplier
                rects = ax.bar(np.arange(2) + offset, data, width, label=name)
                ax.bar_label(rects, padding=3)
                multiplier += 1

            ax.set_title('Downwash Prediction ({} deg camera, {} images)'.format(camera_angle, num_downwash_cases))
            ax.set_ylabel('[\%]')
            ax.set_xticks(np.arange(2) + width, ["Precision", "Recall"])

            ax.legend()

            pdf.savefig(fig)
            plt.close()



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("foldername")
    args = parser.parse_args()

    plot_results(args.foldername)

if __name__ == "__main__":
    main()
