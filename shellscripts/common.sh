#!/bin/bash
echo "Loading common parameters for ground segmentation benchmark..."

# Use absolute directory path for data_path including HOME path if you are not using SSD
rosparam set /data_path "/home/user/data/SemanticKITTI/" # path of downloaded KITTI dataset. It must include '/' at the end part

rosparam set /init_idx 0                                 # index of the first frame to run

rosparam set /stop_for_each_frame false                  # set as 'true' to make it stop every frame

rosparam set /save_csv_file true                        # set as 'false' if csv output files are not needed

rosparam set /save_pcd_flag true                        # set as 'false' if csv output files are not needed

# Use path relative to HOME directory
rosparam set /output_path "/data/"                       # path of output files to be generated

echo "Loading complete!"