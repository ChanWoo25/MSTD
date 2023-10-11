import os
from pathlib import Path
RESULT_DIR = '/data/results/MSTD/231011_Test'
os.makedirs(RESULT_DIR, exist_ok=True)

DESCRIPTION = """\
# description here
"""

def write_config_file(seq_name:str, seq_id:int, z_max:float):
    SAVE_DIR = f"{RESULT_DIR}/{seq_name}_{seq_id:02d}-z_max={z_max}"
    os.makedirs(SAVE_DIR, exist_ok=True)
    exid = Path(SAVE_DIR).name
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
proj_dis_min: 0.0
proj_dis_max: 5.0
corner_thre: 10.0
# std descriptor
descriptor_near_num: 15
descriptor_min_len: 2.0
descriptor_max_len: 30.0
non_max_suppression_radius: 2.0
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
result_dir: "{SAVE_DIR}"
seq_name: "{seq_name}"
align: true
z_max: {z_max}
exid: {exid}
is_benchmark: true
"""
    save_fn = f'{SAVE_DIR}/config.yaml'
    with open(save_fn, 'w') as f:
        print(f'Write on {save_fn}')
        f.write(config)

for seq_id in [1, 2]:
    for z_max in [15.0]:
        write_config_file('park_avia', seq_id, z_max)
