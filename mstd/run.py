import time
import os

print("Processes start")
command = "./bin/main_icp " \
        + f"--sequence_dir=/data/datasets/dataset_std/park_avia_01 " \
        + f"--method=icp"
print(command)
os.system(command)
