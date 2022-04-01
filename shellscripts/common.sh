#!/bin/bash
echo "Loading common parameters for ground segmentation benchmark..."

# if you are using SSD, then set as absolute path and delete 'HOME' of data_path in main_offline.cpp
rosparam set /data_path "/data/SemanticKITTI/sequences/" # path of downloaded KITTI dataset. It must include '/' at the end part

rosparam set /stop_for_each_frame false                  # set as 'true' to make it stop every frame

rosparam set /init_idx 0                                 # index of the first frame to run

rosparam set /save_csv_file false                        # set as 'false' if csv output files are not needed

rosparam set /save_pcd_flag false                        # set as 'false' if csv output files are not needed

rosparam set /output_path "/data/"                       # path of output files to be generated

echo "Loading complete!"
