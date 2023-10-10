#! /bin/bash
./bin/main_icp \
  --sequence_dir=/data/datasets/dataset_std/park_avia_01 \
  --pose_fn=/data/datasets/dataset_std/park_avia_01/pose_given.txt \
  --save_dir=/data/datasets/dataset_std/park_avia_01_icp \
  --method=icp

./bin/main_icp \
  --sequence_dir=/data/datasets/dataset_std/park_avia_02 \
  --pose_fn=/data/datasets/dataset_std/park_avia_02/pose_given.txt \
  --save_dir=/data/datasets/dataset_std/park_avia_02_icp \
  --method=icp

./bin/main_icp \
  --sequence_dir=/data/datasets/dataset_std/park_avia_01 \
  --pose_fn=/data/datasets/dataset_std/park_avia_01/pose_given.txt \
  --save_dir=/data/datasets/dataset_std/park_avia_01_gicp \
  --method=gicp

./bin/main_icp \
  --sequence_dir=/data/datasets/dataset_std/park_avia_02 \
  --pose_fn=/data/datasets/dataset_std/park_avia_02/pose_given.txt \
  --save_dir=/data/datasets/dataset_std/park_avia_02_gicp \
  --method=gicp
