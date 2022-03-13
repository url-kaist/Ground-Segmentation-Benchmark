echo "Loading common parameters for ground segmentation benchmark..."

rosparam set /data_path "/media/jeewon/Elements/semantic_kitti_raw"
rosparam set /output_csvpath "/data/patchwork22/"
rosparam set /stop_for_each_frame true
rosparam set /save_csv_file true
rosparam set /init_idx 0

echo "Loading complete!"
