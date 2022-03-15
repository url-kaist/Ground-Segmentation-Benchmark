GSEG_BENCHMARK_PATH=$(rospack find gseg_benchmark)
rosparam load $GSEG_BENCHMARK_PATH/config/params.yaml
bash $GSEG_BENCHMARK_PATH/shellscripts/common.sh

rosparam set /algorithm "gaussian"

for seq in "'00'" "'01'" "'02'" "'03'" "'04'" "'05'" "'06'" "'07'" "'08'" "'09'" "'10'" 
do
	rosparam set /sequence ${seq}
	sleep 1
	rosrun gseg_benchmark benchmark_offline
done 
