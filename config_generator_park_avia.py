import os
RESULT_DIR = '/data/results/MSTD/231003_except_trees'
os.makedirs(RESULT_DIR, exist_ok=True)

DESCRIPTION = """\
# description here
"""

def write_config_file(seq_name:str, seq_id:int, z_max:float):
    SAVE_DIR = f"{RESULT_DIR}/{seq_name}_{seq_id:02d}-z_max={z_max}"
    os.makedirs(SAVE_DIR, exist_ok=True)
    bag_path  = '/data/datasets/dataset_std/park_avia/park%d.bag'%seq_id
    pose_path = '/data/datasets/dataset_std/park_avia/park%d_pose.txt'%seq_id
    config = \
f"""\
# Experiment commit: not updated
{DESCRIPTION}
# pre process
ds_size: 0.25
maximum_corner_num: 500
# key points
plane_detection_thre: 0.01
plane_merge_normal_thre: 0.1
voxel_size: 1.0
voxel_init_num: 10
proj_image_resolution: 0.25
proj_dis_min: 0
proj_dis_max: 5
corner_thre: 10
# std descriptor
descriptor_near_num: 15
descriptor_min_len: 2
descriptor_max_len: 30
non_max_suppression_radius: 2
std_side_resolution: 0.2
# candidate search
skip_near_num: 50
candidate_num: 50
sub_frame_num: 10
vertex_diff_threshold: 0.8
rough_dis_threshold: 0.015
normal_threshold: 0.2
dis_threshold: 0.5
icp_threshold: 0.2
# need to be written
log_dir: "{SAVE_DIR}"
lidar_path: "{bag_path}"
pose_path: "{pose_path}"
seq_name: "{seq_name}"
align: true
seq_id: 1
z_max: {z_max}
is_benchmark: True
"""
    save_fn = f'{SAVE_DIR}/config.yaml'
    with open(save_fn, 'w') as f:
        print(f'Write on {save_fn}')
        f.write(config)

for seq_id in [1, 2]:
    for z_max in [10.0, 12.0, 15.0, 20.0]:
        write_config_file('park_avia', seq_id, z_max)
