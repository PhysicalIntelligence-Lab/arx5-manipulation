# arx5-manipulation
This is the PhiLab arx5 manipulation repo.

## Setup
The project is completely based on [arx5-sdk@Stanford](https://github.com/real-stanford/arx5-sdk). 
First, for the setup, we can use the same instruction on [https://github.com/real-stanford/arx5-sdk](https://github.com/real-stanford/arx5-sdk).
<!-- (Alert: this script is not tested)
```shell
sudo chmod +x ./arx_setup.sh
./arx_setup.sh
``` -->
The steps are basically the following (Mamba can be replaced by conda, but mamba is recommended):
```shell
mamba env create -f conda_environments/py310_environment.yaml
# if you do not have mamba, you can also use conda, which takes significantly longer
# Currently available python versions: 3.8, 3.9, 3.10, 3.11 
conda activate arx-py310 
mkdir build && cd build
cmake ..
make -j
# At this point, you should be able to run test scripts below.
```
Then install USB Can Tools.
```shell
sudo apt install can-utils
sudo apt install net-tools
```

Refer to the **For adapters using SLCAN framework** section of the original arx5 sdk repo.

After the above steps are all set, it must be noted that in each launch of the robot arm after turning on the device, the following command must be executed:
```shell
sudo slcand -o -f -s8 /dev/arxcan0 can0 && sudo ifconfig can0 up
sudo slcand -o -f -s8 /dev/arxcan1 can1 && sudo ifconfig can1 up
sudo ip link set up can0 type can bitrate 1000000
sudo ip link set up can1 type can bitrate 1000000
```
It must be noted that we have two arms, one uses can0 and the other uses can1.
<!-- This script will load the rules.d file into /etc/udev/rules.d and reload. -->

## Script Usage
### arm_follow
description: A teleoperation demo for one arm following the other.
usage: 
```shell
python arm_follow L5
```
Where `L5` is the arm model. ARX L5 arm are no longer on sale; however `R5` has the same urdf model as `L5`. Only `L5` is implemented.

### collect data
description: Collect data using teleoperation (An upgrade from arm_follow). Also utilizes two realsense cameras.

usage:
```shell
python collect data <episode_number>
```
The collected data is stored at default location `~/data`.
Also note that install pyrealsense using pip before running this script. Replace the `episode_numnber` with the index of your the current episode index.
### process data
description: process the data in `~/data` into zarr array.

usage:
```shell
python process_data.py
```
*Note*: Please modifiy the maximum episode number in the script! The default value is 10. If the episode number is not 10 please modify! Also intall python zarr package before using.


