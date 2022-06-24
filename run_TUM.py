import os
import numpy as np

all_datasets = "/media/nvidia/TUM/"
results_folder = "~/results/stats_incremental/euroc/"

seqs = [ "360_hemisphere", "desk", "pioneer_360", "pioneer_slam2", "pioneer_slam", "long_office_household"]

for seq in seqs:
    sequence_full_path = all_datasets + "rgbd_dataset_" + seq

    if freiburg2 in seq:
        settings_num = 2
    elif freiburg3 in seq:
        settings_num = 3

    # Results location
    output_full_path = results_folder + seq
    os.system("mkdir " + output_full_path)

    for ratio in np.arange(0.5, 1.5, .1):
        format_float = "{:.2f}".format(ratio)
        print("Sequence " + output_full_path + ", ratio " + format_float)
        print("\n")

        run_string = "./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM{}.yaml {}/ {}/associations.txt {} > {}/{}_connected.txt".format(
            settings_num, sequence_full_path, sequence_full_path, format_float, output_full_path, format_float
        )

        cp_tracking = "cp tracking.txt " + output_full_path + format_float + "_stats.txt"
        cp_traj = "cp CameraTrajectory.txt " + output_full_path + format_float + "_trajectory.txt"

        print(run_string)
        print(cp_tracking)
        print(cp_traj)
        print("\n")

        os.system(run_string)
        os.system(cp_tracking)
        os.system(cp_traj)
≈