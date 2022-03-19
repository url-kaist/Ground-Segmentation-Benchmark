_# Ground Segmentation Benchmark 

All the baseline methods are organized.

## Contributor

* Jeewon Kim (as a research intern @ [URL](https://urobot.kaist.ac.kr/)): `ddarong2000@kaist.ac.kr`
* Hyungtae Lim: `shapelim@kaist.ac.kr`

## Contents

0. [Test Env.]()
1. [Requirements]()
2. [Prepare DataSet]()
3. [How to Run]()
4. [Citation]()

## Test Env.

The code is tested successfully at
* Linux 18.04 LTS
* ROS Melodic

## Requirements


### ROS Settings

* Install [ROS](http://wiki.ros.org/melodic/Installation) on a machine
* unavlib(to be deleted)
* Install jsk_visualization (For visualization of GLE of Patchwork)
 
```
sudo apt update
sudo apt-get install ros-melodic-jsk-recognition
sudo apt-get install ros-melodic-jsk-common-msgs
sudo apt-get install ros-melodic-jsk-rviz-plugins
```
* Clone our package with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
```
cd catkin_ws/src
git clone https://github.com/LimHyungTae/gseg.git (can be changed)
cd .. && catkin build gseg_benchmark
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
        |___000000
        |___ ...
    |___velodyne
        |___000000.bin
        |___ ...
```


## How to Run 

```asm
$ roscore
```

`shellscripts` 폴더 내의 파일들 참조

* 각 .sh 파일로 sequence를 전체 다 저장하는 코드가 있음
* "'00'"이 그냥 bash shell에서는 숫자로 들어가는데, [Zsh](https://github.com/ohmyzsh/ohmyzsh/wiki/Installing-ZSH) 에서만 현재 사용 가능


```asm
$ cd ${path of Ground-Segmentation-Benchmark}/launch
$ roslaunch gseg_benchmarker.launch alg:=${name of algorithm} seq:=${sequence}
```
* There are 7 ground segmentation algorithms provided: `cascaded_gseg`, `gpf`, `r_gpf`, `ransac`, `linefit`, `patchwork`, `gaussian`
* If you do not set `seq` or set as `seq:=all`, then the csv output files of all datasets from "00" to "10" will be saved automatically.   
* Rviz result will be shown automatically.

## Parameters of Benchmark

```
rosparam set /data_path "/data/SemanticKITTI/sequences"
rosparam set /output_csvpath "/data/patchwork22/"
rosparam set /stop_for_each_frame true
rosparam set /save_csv_file true
rosparam set /init_idx 0
```

아래의 알고리즘들이 구현되어 있음

* CascadedSeg
* GPF
* LineFit
* Patchwork (ver.1)  
* R-GPF
* Mono plane estimation by RANSAC


![Image text](config/materials/seq00_results.png)

**RVIZ**는 `ground4r_gpf.rviz` 사용하면 됨!

#### Point label 관련
* point의 member 변수들은 `utils/common.hpp`에 나와있음: `x, y, z, intensity, label, id`로 구분됨. 여기서 id는 각 object의 아이디임 (본 레포에서는 안 쓰일듯)
* label은 int로 돼있는데, 각 int가 나타내는 건 [SemanticKITTI API](https://github.com/PRBonn/semantic-kitti-api/blob/master/config/semantic-kitti.yaml)에 나와있음
* Patchwork에서는 vegetation도 아랫 부분은 ground라고 간주하고 뽑았으나, Patchwork++에서는 그렇지 않음

--- 

#### 진행 사항

* **msg의 node.msg는 절대 변경하지 마셈!** 변경하면 bag 파일의 node를 callback 못 받음...bag 파일을 재생성해야 callback을 받을 수 있음
* 사용하기 용이한 flags들 설정 (변수로?)
* launch 파일 간소화하기
* 해당 알고리즘에 대한 rviz 파일을 따로 생성하길 권장

### Errors
if the following error occurs
```
/usr/include/flann/util/serialization.h:35:14: error: ‘class std::unordered_map<unsigned int, std::vector<unsigned int> >’ has no member named ‘serialize’
```
then run
```asm
$ sudo gedit /usr/include/flann/util/serialization.h
```
and change all terms of "map" into "unordered_map".