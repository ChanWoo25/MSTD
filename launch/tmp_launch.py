import time
import os

print("Processes start")
for seq in [2]:
    for valid_voxel_thres in [1, 3]:
        command = f"python3 /root/ws_std/src/MSTD/launch/launch_pseudo.py --seq {seq} --valid_voxel_thres {valid_voxel_thres}"
        print(command)
        os.system(command)

        print("sleep 1 seconds ...")
        time.sleep(1)
print("Processes end")
