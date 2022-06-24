import os
import numpy as np

all_datasets = "/media/nvidia/euroc"
results_folder = "~/results/stats_incremental/euroc/"

seqs = [ "MH04", "MH05", "V101", "V102", "V103"]

for seq in seqs:
    if seqs == "MH04":
        sequence_full_path = all_datasets + "/machine_hall/MH_04_difficult"
    elif seqs == "MH05":
        sequence_full_path = all_datasets + "/machine_hall/MH_05_difficult"
    elif seqs == "V101":
        sequence_full_path = all_datasets + "/vicon_room1/V1_01_easy"
    elif seqs = "V102":
        sequence_full_path =  all_datasets + "/vicon_room1/V1_02_medium"
    elif seqs = "V103":
        sequence_full_path = all_datasets + "/vicon_room1/V1_03_difficult"

    timestamps = seq + ".txt"

    # Results location
    output_full_path = results_folder + seq + "/"
    os.system("mkdir " + output_full_path)

    for ratio in np.arange(0.5, 1.5, .1):
        format_float = "{:.2f}".format(ratio)
        print("Sequence " + output_full_path + ", ratio " + format_float)
        print("\n")

        run_string = "./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml {}/mav0/cam0/data {}/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/{} > {}/{}_connected.txt".format(sequence_full_path, sequence_full_path, timestamps, format_float)

        cp_tracking = "cp tracking.txt " + output_full_path + format_float + "_stats.txt"
        cp_traj = "cp CameraTrajectory.txt " + output_full_path + format_float + "_trajectory.txt"

        print(run_string)
        print(cp_tracking)
        print(cp_traj)
        print("\n")

        os.system(run_string)
        os.system(cp_tracking)
        os.system(cp_traj)
