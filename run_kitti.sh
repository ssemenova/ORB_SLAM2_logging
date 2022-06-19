folder=/media/nvidia/data_odometry_gray-KITTI/dataset/sequences

for (( i=0; i<12; i++ ))
do
	num=$(printf "%02d" $i)
	#if [[ $i < 12	]]
	#	num=
	#then
	#	num="$i"
	#fi

	case $num in 
		00|01|02) calibfile="00-02";;
		03) calibfile="03";;
		04|05|06|07|08|09|10|11|12) calibfile="04-12";;
	esac

	echo "./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI$calibfile.yaml $folder/$num/ > ~/results/stats/kitti$num.txt"
	./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI$calibfile.yaml $folder/$num > ~/results/stats/kitti${num}_connected.txt
	cp tracking.txt ~/results/stats/kitti${num}_stats.txt
done
