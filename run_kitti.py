import os
import numpy as np

folder = "/media/nvidia/data_odometry_gray-KITTI/dataset/sequences/"
results_folder = "~/results/stats_incremental/kitti/"

seqs = ["03","05","06"] #,"07"]

for seq in seqs:
    # Calibration file
    if seq == "03":
        calibration_file = "03"
    else:
        calibration_file = "04-12"
    calibration_full_path = "Examples/Stereo/KITTI" + calibration_file + ".yaml "

    # Dataset location
    sequence_full_path = folder + seq

    # Results location
    output_full_path = results_folder + seq + "/"
    os.system("mkdir " + output_full_path)

    # ratio_multiply = format_float * 100

    for ratio in np.arange(0.3,2.5,.05):
        format_float = "{:.2f}".format(ratio)
        print("Sequence " + output_full_path + ", ratio " + format_float)
        print("\n")

        run_string = "./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt " + calibration_full_path + " " + sequence_full_path + " " + format_float + " > " + output_full_path + format_float + "_connected.txt"

        cp_tracking = "cp tracking.txt " + output_full_path + format_float + "_stats.txt"
        cp_traj = "cp CameraTrajectory.txt " + output_full_path + format_float + "_trajectory.txt"

        print(run_string)
        print(cp_tracking)
        print(cp_traj)
        print("\n")

        os.system(run_string)
        os.system(cp_tracking)
        os.system(cp_traj)
