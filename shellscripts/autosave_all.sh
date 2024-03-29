#!/bin/bash
GSEG_BENCHMARK_PATH=$(rospack find gseg_benchmark)
rosparam load $GSEG_BENCHMARK_PATH/config/params.yaml
bash $GSEG_BENCHMARK_PATH/shellscripts/common.sh

rosparam set /algorithm "cascaded_gseg"

for seq in "'00'" "'01'" "'02'" "'03'" "'04'" "'05'" "'06'" "'07'" "'08'" "'09'" "'10'"
do
	rosparam set /sequence ${seq}
	sleep 1
	rosrun gseg_benchmark benchmark_offline
done

rosparam set /algorithm "gpf"
rosparam set /gpf/mode "multiple" # "single" or "multiple"

for seq in "'00'" "'01'" "'02'" "'03'" "'04'" "'05'" "'06'" "'07'" "'08'" "'09'" "'10'"
do
	rosparam set /sequence ${seq}
	sleep 1
	rosrun gseg_benchmark benchmark_offline
done

rosparam set /algorithm "gpregression"

for seq in  "'00'" "'01'" "'02'" "'03'" "'04'" "'05'" "'06'" "'07'" "'08'" "'09'" "'10'"
do
	rosparam set /sequence ${seq}
	sleep 1
	rosrun gseg_benchmark benchmark_offline
done

rosparam set /algorithm "linefit"

for seq in "'00'" "'01'" "'02'" "'03'" "'04'" "'05'" "'06'" "'07'" "'08'" "'09'" "'10'"
do
	rosparam set /sequence ${seq}
	sleep 1
	rosrun gseg_benchmark benchmark_offline
done

rosparam set /algorithm "patchwork"

for seq in "'00'" "'01'" "'02'" "'03'" "'04'" "'05'" "'06'" "'07'" "'08'" "'09'" "'10'"
do
	rosparam set /sequence ${seq}
	sleep 1
	rosrun gseg_benchmark benchmark_offline
done

rosparam set /algorithm "r_gpf"

for seq in "'00'" "'01'" "'02'" "'03'" "'04'" "'05'" "'06'" "'07'" "'08'" "'09'" "'10'"
do
	rosparam set /sequence ${seq}
	sleep 1
	rosrun gseg_benchmark benchmark_offline
done

rosparam set /algorithm "ransac"

for seq in "'00'" "'01'" "'02'" "'03'" "'04'" "'05'" "'06'" "'07'" "'08'" "'09'" "'10'"
do
	rosparam set /sequence ${seq}
	sleep 1
	rosrun gseg_benchmark benchmark_offline
done
