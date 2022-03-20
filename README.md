# Ground Segmentation Benchmark 

All the baseline methods are organized.

* CascadedSeg
* GPF
* LineFit
* Patchwork (ver.1)
* R-GPF
* Mono plane estimation by RANSAC
* [Gaussian Floor Segmentation](https://github.com/SmallMunich/FloorSegmentation/tree/master/Gaussian_process_based_Real-time_Ground_Segmentation_for_Autonomous_Land_Vehicles)


![Image text](config/materials/seq00_results.png)

## Contents

0. [Description]()
1. [Requirements]()
2. [Prepare DataSet]()
3. [Getting Started]()
4. [Citation]()


## Requirements
### Test Env.

The code is tested successfully at
* Linux 18.04 LTS
* ROS Melodic


### Settings

* Install [ROS](http://wiki.ros.org/melodic/Installation) on a machine
* Install jsk_visualization (For visualization of GLE of Patchwork)
 
```
sudo apt update
sudo apt-get install ros-melodic-jsk-recognition
sudo apt-get install ros-melodic-jsk-common-msgs
sudo apt-get install ros-melodic-jsk-rviz-plugins
```
### Install Package
* Clone our package with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
```
cd catkin_ws/src
git clone https://github.com/LimHyungTae/gseg.git (can be changed)
catkin build gseg_benchmark
```

## Prepare Dataset
### Offline KITTI dataset
1. Download [SemanticKITTI](http://www.semantic-kitti.org/dataset.html#download) Odometry dataset including Velodyne point clouds, calibration data, and label data.
2. Set the `data_path` in `shellscripts/common.sh` for your machine.

The `data_path` consists of `velodyne` folder and `labels` folder as follows:
```
data_path
______00
    |___labels
        |___000000
        |___000001
        |___ ...
    |___velodyne
        |___000000.bin
        |___000001.bin
        |___ ...
______01
    |___labels
        |___ ...
    |___velodyne
        |___ ...
```


## Getting Started

### Set Parameters of Benchmark
Set parameters about dataset path, running method, saving csv output files in shellscripts/common.sh
Make directories to load and save data files and apply them in rosparam setting.

```
rosparam set /data_path "/data/SemanticKITTI/"      # path of downloaded KITTI data
rosparam set /stop_for_each_frame false             # set as true to make it stop every frame 
rosparam set /init_idx 0                            # set first frame to run
rosparam set /save_csv_file true                    # set as false if csv output files are not needed
rosparam set /output_csvpath "/data/patchwork/"     # path of output files to be generated
```
###Play Sample Data

Start roscore:
```asm
$ roscore
``` 
Open a new terminal and launch node with specification of algorithm and data sequence:
```asm
$ cd ${path of Ground-Segmentation-Benchmark}/launch
$ roslaunch gseg_benchmarker.launch alg:=${name of algorithm} seq:=${sequence}
```
* There are 7 algorithms provided: `cascaded_gseg`, `gpf`, `r_gpf`, `ransac`, `linefit`, `patchwork`, `gaussian`
* If you do not set `seq` or set as `seq:=all`, then the csv output files of all datasets from "00" to "10" will be saved automatically.   
* Rviz result will be shown automatically.


---
## Contributor

* Jeewon Kim (as a research intern @ [URL](https://urobot.kaist.ac.kr/)): `ddarong2000@kaist.ac.kr`
* Hyungtae Lim: `shapelim@kaist.ac.kr`

## Errors
if the following error occurs
```
/usr/include/flann/util/serialization.h:35:14: error: ‘class std::unordered_map<unsigned int, std::vector<unsigned int> >’ has no member named ‘serialize’
```
then run
```asm
$ sudo gedit /usr/include/flann/util/serialization.h
```
and change all terms of "map" into "unordered_map".