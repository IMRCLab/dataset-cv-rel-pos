# dataset-cv-rel-pos
# A Dataset and Comparative Study for Vision-Based Relative Position Estimation of Multiple UAVs Flying in Close Proximity
We propose to use the often ignored yaw degree-of-freedom of multirotors to spin a single, cheap and lightweight
monocular camera at a high angular rate for omnidirectional awareness. We provide a dataset collected with real-world
physical flights as well as with 3D-rendered scenes and compare two existing learning-based methods in different settings
with respect to success rate, relative position estimation, and downwash prediction accuracy. \
[Paper on arXiv](http://arxiv.org/abs/2303.03898) and [Video](https://youtu.be/DTKB0QzB2Qk) are available.
## Data Collection
Our dataset consists of the following: 
* gray-scale images (320×320)
* camera calibration parameters 
* annotations for each image: 
    * robot poses in world frame
    * relative position of all visible neighbors with respect to the camera
    * pixels of their robot center
    * bounding boxes in the image for each visible robot.

Synthetic and Real-world dataset can be downloaded [here](https://tubcloud.tu-berlin.de/s/Sa5rN5JK7poGawr).

## 1. Synthetic data collection
For synthetic images each robot can be equipped with virtual cameras with different mounting angle and perform auto-yaw rotation
while recording images in our simulated environment. 
```
https://github.com/IMRCLab/crazyswarm2/tree/blender_viz
``` 

## 2. Real-world data collection

### 2.1 Crazyflie firmware

**Configuration**

```
cd crazyflie-firmware
make cf2_defconfig
make menuconfig
```

In the interactive shell, select the WiFi mode. This will write a file `build/.config`.
When successful, the `build/.config` file should contain:

```
CONFIG_DECK_AI=y
# CONFIG_DECK_AI_WIFI_NO_SETUP is not set
# CONFIG_DECK_AI_WIFI_SETUP_AP is not set
CONFIG_DECK_AI_WIFI_SETUP_STA=y
# Credentials for access-point
CONFIG_DECK_AI_SSID="<ENTER SSID>"
CONFIG_DECK_AI_PASSWORD="<ENTER PASSWORD>"
```

**Building and Flashing**

```
make -j
cfloader flash cf2.bin stm32-fw -w radio://0/80/2M
```

### 2.2 Camera Calibration

Record images of the checkerboard 
```
python3 data_collection_aideck.py -n  IP-ADDRESS -path PATH-TO-SAVE-IMAGES -id ROBOT-ID
```
Get image and object points from images. All points are recorded into `img_obj_points.yaml` file. 
```
python3 opencv_calibration.py PATH-TO-FOLDER 
```
Compute camera intrinsic parameters and save them into `calibration.yaml` file.
```
python3 offline_calibration_intrinsic.py PATH-TO-IMG_OBJ_POINTS.YAML
```


### 2.3 ROS
ROS2 workspace needs to have [crazyswarm2](https://github.com/IMRCLab/crazyswarm2). 

**Set Up**

Add a symlink of cvmrs_ros to your ROS2 workspace

```
ln <PATH-TO>/dataset-cv-rel-pos/cvmrs_ros <PATH-TO>ros2_ws/src/ -s
```

**Build** 

```
colcon build --symlink-install 
```

**Usage**

Note that all configuration files are the ones used in cvmrs_ros/config. This allows to commit those files without changing the default value of the (public) crazyswarm2 repository. 

```
ros2 launch cvmrs_ros launch.py
```

and in a separate terminal

```
ros2 run cvmrs_ros random_flight
```

## 3. Data Post-Processing
This script converts raw data into synchronized, classifies images based on number of visible robots (0,1,2,etc.), and creates a single `dataset.ymal` file. If `keep_calib` is set to FALSE, then camera extrinsic parameters will be calculated again. After getting synchronized data, dataset will be filtered by excluding frames
with very wrong labels. 
```
python3 postprocess.py PATH-TO-FOLDER --keepcalib 
```
Merging dataset from several folders is possible
```
python3 dataset_merge.py LIST-OF-FOLDERS PATH-TO-OUTPUT-FILE
```
Specify the preferred number of images from each category. Numbers can be provided as a list in ascending order.
```
python3 dataset_filter.py PATH-TO-INPUT-FILE PATH-TO-OUTPUT-FILE -n LIST-OF-ROBOT-NUMBERS
```
Get statistics of a particular `dataset.yaml`
```
python3 dataset_stats.py PATH-TO-FILE
```

## 4. Model training, inference
Get labels and start training. For the training from scratch `--inital_model` should be skipped. If `-b` is provided with a single name, then training of the specified one will be done.
```
python3 train.py PATH-TO-TRAIN-YAML-FILE NEW-MODEL-NAME --inital_model PRE-TRAINED-MODEL-NAME -b yolo locanet
```
Test trained model's performance. If `-b` is provided with a single name, then inference of the specified one will be done.
```
python3 inference.py PATH-TO-TEST-YAML-FILE NEW-MODEL-NAME -b yolo locanet
```
In order to reproduce Fig.4 and Fig.5 from the paper
```
python3 plot_results.py PATH-TO-RESULTS-FOLDER
```