#! /bin/bash

echo "Processes start"

for 
python3 /root/ws_std/src/MSTD/launch/launch_pseudo_1.py \
  --seq 1 \
  --valid_voxel_thres 1

echo "sleep 1 seconds ..."; sleep 1s

python3 /root/ws_std/src/MSTD/launch/launch_pseudo_2.py

echo "Processes end"
