# Rosbag generation

## How to use

### kitti2node.py
0. build를 통해 nonplanar_gpf를 컴파일 해야함. 그래야 nonplanar_gpf.msg를 import 할 수 있음
1. kitti2node의 344~345 input path와 output path를 지정
2. 만약 일부분의 stamp만 필요하면 `kitti2node.py`의 381을 True로 지정하고 원하는 `initial_stamp`와 `final_stamp`를 지정함
3. 2에서 `init_stamp`가 하나 더 있는 이유는 이게 너무 0.0초에 node가 생성돼서 씹히는 경우가 생겨서 그럼 (이해 안 되면 임형태한테 물어보길...)

`frame_range = [init_stamp] + range(init_stamp, final_stamp, interval)`에 해당하는 줄을 말함

4. 아래 명령어 실행

```
python kitti2imnode.py -t None -r None -s $sequence$ --kitti_type "odom_noimg"
```

예시: 00번을 ros node로 만들 때 

```
python kitti2imnode.py -t None -r None -s 00 --kitti_type "odom_noimg"
```

### plot_z_variation.py

각 sequence 별 z 분석 (viz 폴더에 결과들 있음!)

# Descriptions

사실 이 코드는 이미지도 bag에 넣을 수 있는데, 용량이 너무 커지므로 여기서는 lidar(pointcloud)와 pose만 다룸


* SuMa의 pose를 /home/shapelim/hdd2/kitti_semantic/dataset/sequences/00/poses.txt에 여기 넣어둬야 함!

* label은 원래 uint32인데, intensity에 넣기 위해서 float 32로 변경해서 넣어줌. C++ 에서 uint32로 되돌려서 파싱 필요

np.uint32 <-> uint32_t in c++
np.float32 <-> float in c++

Originally, 

CAM2LIDAR = np.array([[-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03],
                      [- 6.481465826011e-03, 8.051860151134e-03, - 9.999466081774e-01, - 7.337429464231e-02],
                      [9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01],
                     [0, 0, 0, 1]])
                     
바닥 기준이라면

CAM2BASE = np.array([[-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03],
                      [- 6.481465826011e-03, 8.051860151134e-03, - 9.999466081774e-01, 1.65],
                      [9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01],
                      [0, 0, 0, 1]])
 
 ***sensor height***는 1.723m 로 잡으면 됨
                      
 
