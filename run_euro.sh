all_datasets=/media/nvidia/euroc

# Machine hall
for (( i=1; i<=5; i++ ))
do
	case $i in 
        1) folder=${all_datasets}/machine_hall/MH_01_easy;
            timestamps='MH01.txt';
            name='mh01';;
        2) folder=${all_datasets}/machine_hall/MH_02_easy;
            timestamps='MH02.txt';
            name='mh02';;
        3) folder=${all_datasets}/machine_hall/MH_03_medium;
            timestamps='MH03.txt';
            name='mh03';;
        4) folder=${all_datasets}/machine_hall/MH_04_difficult;
            timestamps='MH04.txt';
            name='mh04';;
        5) folder=${all_datasets}/machine_hall/MH_05_difficult;
            timestamps='MH05.txt';
            name='mh05';;
        6) folder=${all_datasets}/vicon_room1/V1_01_easy;
            timestamps='V101.txt';
            name='v101';;
        7) folder=${all_datasets}/vicon_room1/V1_02_medium;
            timestamps='V102.txt';
            name='v102';;
        8) folder=${all_datasets}/vicon_room1/V1_03_difficult;
            timestamps='V103.txt';
            name='v103';;
        9) folder=${all_datasets}/vicon_room2/V2_01_easy;
            timestamps='V201.txt';
            name='v201';;
        10) folder=${all_datasets}/vicon_room2/V2_02_medium;
            timestamps='V202.txt';
            name='v202';;
        11) folder=${all_datasets}/vicon_room2/V2_03_difficult;
            timestamps='V203.txt';
            name='v203';;

	esac

    echo $name
    ./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml ${folder}/mav0/cam0/data ${folder}/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/${timestamps} > ~/results/stats/euroc_${name}_connected.txt
	cp tracking.txt ~/results/stats/euroc_${name}_stats.txt
    cp KeyFrameTrajectory.txt  ~/results/stats/euroc_${name}_traj.txt
done
