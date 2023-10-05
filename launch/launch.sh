#! /bin/bash

echo "Processes start"

for seq_id in 1 2 ; do
  for z_max in 10.0 12.0 15.0 20.0 ; do
    echo "Start seq_id:${seq_id}, z_max:${z_max} ..."
    python3 /root/ws_std/src/MSTD/launch/launch.py \
      --seq_id ${seq_id} \
      --z_max ${z_max}
    echo "End   seq_id:${seq_id}, z_max:${z_max} ..."
    sleep 1s
  done
done

echo "Processes end"
