#! /bin/bash
# ./bin/main_std \
#   --sequence_dir=/data/datasets/dataset_std/park_avia_01_gicp \
#   --result_dir=/data/results/MSTD/231011_Test/park_avia_01-z_max=15.0

# ./bin/main_icp \
#   --sequence_dir=/data/datasets/dataset_std/park_avia_01 \
#   --pose_fn=/data/datasets/dataset_std/park_avia_01/pose_given.txt \
#   --save_dir=/data/datasets/dataset_std/park_avia_01_icp \
#   --method=icp

# ./bin/main_icp \
#   --sequence_dir=/data/datasets/dataset_std/park_avia_02 \
#   --pose_fn=/data/datasets/dataset_std/park_avia_02/pose_given.txt \
#   --save_dir=/data/datasets/dataset_std/park_avia_02_icp \
#   --method=icp

# ./bin/main_icp \
#   --sequence_dir=/data/datasets/dataset_std/park_avia_01 \
#   --pose_fn=/data/datasets/dataset_std/park_avia_01/pose_given.txt \
#   --save_dir=/data/datasets/dataset_std/park_avia_01_icp \
#   --method=icp \
#   --icp_thres=1.0 \
#   --ds_voxel_size=0.25


# ./bin/main_icp \
#   --sequence_dir=/data/datasets/dataset_std/park_avia_02 \
#   --pose_fn=/data/datasets/dataset_std/park_avia_02/pose_given.txt \
#   --save_dir=/data/datasets/dataset_std/park_avia_02_gicp \
#   --method=gicp

# ./bin/main_std \
#   --sequence_dir=/data/datasets/dataset_std/park_avia_01 \
#   --result_dir=/data/results/MSTD/231011_Test/park_avia_01-z_max=15.0

# ./bin/main_std \
#   --sequence_dir=/data/datasets/dataset_kitti/00 \
#   --result_dir=/data/results/MSTD/231013_Test/kitti_default \
#   --pose_fn=/data/datasets/dataset_std/kitti/kitti00.txt

# ./bin/main_scancontext \
#   --sequence_dir=/data/datasets/dataset_kitti/00 \
#   --result_dir=/data/results/MSTD/231013_Test/kitti_default \
#   --pose_fn=/data/datasets/dataset_std/kitti/kitti00.txt \
#   --visualize=false

# KITTI
for SEQ in 00 02 05 08 ; do
  for SC_CAND_NUM in 10 50; do
    ./bin/main_scancontext \
      --sequence_dir=/data/datasets/dataset_kitti/${SEQ} \
      --result_dir=/data/results/MSTD/231013_Test/kitti_default \
      --pose_fn=/data/datasets/dataset_std/kitti/kitti${SEQ}.txt \
      --visualize=false \
      --seq_name=kitti${SEQ} \
      --SC_CAND_NUM=${SC_CAND_NUM}
  done
done

# Test #
# ./bin/main_scancontext \
#   --sequence_dir=/data/datasets/dataset_kitti/08 \
#   --result_dir=/data/results/MSTD/231013_Test/kitti_default \
#   --pose_fn=/data/datasets/dataset_std/kitti/kitti08.txt \
#   --visualize=true \
#   --seq_name=kitti08 \
#   --SC_CAND_NUM=10
