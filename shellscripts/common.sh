#!/bin/bash
echo "Loading common parameters for ground segmentation benchmark..."

rosparam set /data_path "/media/jeewon/Elements/semantic_kitti_raw"
rosparam set /output_csvpath "/data/"
rosparam set /stop_for_each_frame false
rosparam set /save_csv_file true
rosparam set /init_idx 0
rosparam set /show_rviz true

echo "Loading complete!"