# dataset-cv-rel-pos
# A Dataset and Comparative Study for Vision-Based Relative Position Estimation of Multiple UAVs Flying in Close Proximity
We propose to use the often ignored yaw degree-of-freedom of multirotors to spin a single, cheap and lightweight
monocular camera at a high angular rate for omnidirectional awareness. We provide a dataset collected with real-world
physical flights as well as with 3D-rendered scenes and compare two existing learning-based methods in different settings
with respect to success rate, relative position estimation, and downwash prediction accuracy.

## Data Collection
Our dataset consists of the following: 
* gray-scale images (320×320)
* camera calibration parameters 
* annotations for each image: 
    * robot poses in world frame
    * relative position of all visible neighbors with respect to the camera
    * pixels of their robot center
    * bounding boxes in the image for each visible robot.

Synthetic and Real-world dataset can be downloaded here.
## 1. Synthetic data collection
For synthetic images each robot can be equipped with virtual cameras with different mounting angle and perform auto-yaw rotation
while recording images in our simulated environment. 
```
https://github.com/IMRCLab/crazyswarm2/tree/blender_viz
``` 

## 2. Real-world data collection
### 2.1 GAP8 firmware with Docker

**Installing the bootloader**

One-time for "new" AI decks, then the firmware can be replaced wirelessly.

```
https://github.com/bitcraze/aideck-gap8-bootloader
```

**Building and flashing wirelessly**

```
docker pull bitcraze/aideck
docker run --rm -v ${PWD}:/module bitcraze/aideck tools/build/make-example examples/other/wifi-img-streamer image
cfloader flash examples/other/wifi-img-streamer/BUILD/GAP8_V2/GCC_RISCV_FREERTOS/target.board.devices.flash.img deck-bcAI:gap8-fw -w radio://0/80/2M
```

**Attaching the debugger**

First, connect ARM-JTAG-20-10 ADAPTER to ARM-USB-TINY-H. After, ARM-JTAG-20-10 ADAPTER should get connected to GAP8 JTAG interface on the AI-deck as given [here](https://www.bitcraze.io/documentation/repository/AIdeck_examples/master/getting-started/jtag-programming/). **Note:** Purple color is on the left side when viewing the board from the top and camera front.

**Note** If you have the JTAG adapter attached before powering the board, the camera might not start up properly. Therefore, it might be necessary to unplug the JTAG if the AI-deck/Crazyflie is power cycled and plug it back in after it is powered again.

**Building and Flashing**

**Note:** Before running the following docker commands, make sure that Docker is configured to work without Sudo (non-root) with [these instructions](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).

**Note:** WiFi Settings are now configured as part of the STM32 firmware.


```
docker run --rm -v ${PWD}:/module --device /dev/ttyUSB0 --privileged -P bitcraze/aideck tools/build/make-example examples/other/wifi-img-streamer flash
```

### 2.2 ESP32 firmware with Docker


**Attaching the debugger**

Connect ARM-JTAG-20-10 ADAPTER to ARM-USB-TINY-H. Then, ARM-JTAG-20-10 ADAPTER should get connected to NINA JTAG interface on the AI-deck. **Note:** Purple color is on the left side when viewing the board from the top and camera front.

**Initial Set-up**
```
docker pull bitcraze/aideck-nina
```
**Install IDF**
```
git clone https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v4.3.1
git submodule sync
git submodule update --init --recursive
./install.sh
```
**Building and Flashing**
```
cd aideck-esp-firmware
source ../esp-idf/export.sh
idf.py app bootloader
docker run --rm -it -v $PWD:/module/ --device /dev/ttyUSB0 --privileged -P bitcraze/aideck-nina /bin/bash -c "/openocd-esp32/bin/openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f board/esp-wroom-32.cfg -c 'program_esp32 build/bootloader/bootloader.bin 0x1000 verify' -c 'program_esp32 build/aideck_esp.bin 0x10000 verify reset exit'"
```

### 2.3 Crazyflie firmware

**Configuration**

```
cd crazyflie-firmware
make cf2_defconfig
make menuconfig
```

In the interactive shell, select the WiFi mode. This will write a file `build/.config`.
The WiFi credentials can be found on the TUB Cloud/imrc/network.xlsx.

When successful, the `build/.config` file should contain:

```
CONFIG_DECK_AI=y
# CONFIG_DECK_AI_WIFI_NO_SETUP is not set
# CONFIG_DECK_AI_WIFI_SETUP_AP is not set
CONFIG_DECK_AI_WIFI_SETUP_STA=y

#
# Credentials for access-point
#
CONFIG_DECK_AI_SSID="<SSID from network.xlsx>"
CONFIG_DECK_AI_PASSWORD="<password from network.xlsx>"
```

**Building and Flashing**

```
make -j
cfloader flash cf2.bin stm32-fw -w radio://0/80/2M
```

###  Debugging

#### GAP8

* Note: The output will also be shown in the CF console (e.g., with cfclient or Crazyswarm2)
  
* Connect CP2102 USB<->Serial adapter with Header left Pin 2 to RXD and Header left Pin 10 to GND
* Use `dmesg` to find the device name (e.g., `/dev/ttyUSB0`)
* Connect using cutecom (115200 baud, 8 data bits, no parity, 1 stop bit, no flow control)

#### NINA

* Connect CP2102 USB<->Serial adapter with Header left Pin 6 to RXD and Header left Pin 10 to GND
* Use `dmesg` to find the device name (e.g., `/dev/ttyUSB0`)
* Connect using cutecom (115200 baud, 8 data bits, no parity, 1 stop bit, no flow control)

### 2.4 Camera Calibration

* Use checkerboard (8x5, 31mm squares)
* Make sure that the (ROS) coordinate system is: x long horizontal, y short vertical, z pointing away from front of checkerboard. Origin needs to be at the bottom-right square
* Use `ros2 topic echo /poses` to find data for checkerboard, put it in a file named `checkerboard.yaml`
* Use `ros2 run cvmrs_ros camera_calibration` to record data (either flying or moving manually with ROS running)
* Get image and object points from images. Object points are in robot frame. All points are recorded into `img_obj_points.yaml` file. This also computes the actual `calibration.yaml`
```
python3 opencv_calibration.py PATH-TO-FOLDER
```
* If needed, the calibration.yaml can be re-written with different settings by calling
```
python3 offline_calibration.py PATH-TO-YAML
```

### 2.5 ROS

## Set Up

Add a symlink of cvmrs_ros to your ROS2 workspace

```
ln <PATH-TO>/dataset-cv-rel-pos/cvmrs_ros <PATH-TO>ros2_ws/src/ -s
```

## Build

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DMARC_BUILD=OFF --packages-ignore cffirmware joy_linux spacenav wiimote
```

## Usage

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
python3 postprocess.py PATH-TO-FOLDER --keep_calib TRUE or FALSE
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
python3 dataset_stats.py PATH-TO-YAML-FILE
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
python3 plot_results.py PATH-TO-MAIN-FOLDER
```