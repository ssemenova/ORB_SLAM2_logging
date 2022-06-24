folder=/media/nvidia/data_odometry_gray-KITTI/dataset/sequences
results_folder=~/results/stats_7/
echo $results_folder

for (( i=0; i<12; i++ ))
do
	num=$(printf "%02d" $i)

    # Only do some datasets, other ones aren't useful
	case $i in 
		0|1|2|4|8|9|10) continue;;
	esac
	case $num in 
		00|01|02) calibfile="00-02";;
		03) calibfile="03";;
		04|05|06|07|08|09|10|11|12) calibfile="04-12";;
	esac

	echo "$num"
	./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI$calibfile.yaml $folder/$num > ${results_folder}/kitti${num}_connected.txt
	cp tracking.txt ${results_folder}/kitti${num}_stats.txt
	cp CameraTrajectory.txt ${results_folder}/kitti${num}_trajectory.txt
done
