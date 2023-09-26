import os
RESULT_DIR = '/data/results/MSTD/230926_pseudo_loop_gt_detection'
os.makedirs(RESULT_DIR, exist_ok=True)

DESCRIPTION = """\
# description here
"""
commit = '1196c59ddbd46027b534e8124272dd70a75d0a7d'

# Iterate parameters you want here
paths = []
for i in range(2):
    seq_name = 'park_avia'
    bag_path = '/data/datasets/dataset_std/park_avia/park%d.bag'%(i+1)
    pose_path = '/data/datasets/dataset_std/park_avia/park%d_pose.txt'%(i+1)
    SAVE_DIR = f"{RESULT_DIR}/{seq_name}_{i+1:02d}"
    os.makedirs(SAVE_DIR, exist_ok=True)

    config = \
f"""\
# Experiment commit {commit}
{DESCRIPTION}
# pre process
ds_size: 0.25
maximum_corner_num: 100
# key points
plane_detection_thre: 0.01
plane_merge_normal_thre: 0.2
voxel_size: 2.0
voxel_init_num: 10
proj_image_resolution: 0.5
proj_dis_min: 0
proj_dis_max: 5
corner_thre: 10
# std descriptor
descriptor_near_num: 10
descriptor_min_len: 2
descriptor_max_len: 50
non_max_suppression_radius: 2
std_side_resolution: 0.2
# candidate search
skip_near_num: 50
candidate_num: 50
sub_frame_num: 10
vertex_diff_threshold: 0.5
rough_dis_threshold: 0.01
normal_threshold: 0.2
dis_threshold: 0.5
icp_threshold: 0.4
# need to be written
lidar_path: "{bag_path}"
pose_path: "{pose_path}"
seq_name: {seq_name}
save_pseudo_loop_gt_fn: "{SAVE_DIR}/pseudo_loop_gt.txt"
is_benchmark: False
"""
    save_fn = f'{SAVE_DIR}/config.yaml'
    with open(save_fn, 'w') as f:
        print(f'Write on {save_fn}')
        f.write(config)
