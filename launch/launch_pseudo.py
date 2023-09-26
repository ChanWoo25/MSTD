import roslaunch
import rospy
import rosparam
import copy

WORKSPACE = "/root/ws_std"
MSTD_DIR = f"{WORKSPACE}/src/MSTD"
LAUNCH_DIR = f"{MSTD_DIR}/launch"
CONFIG_DIR = f"{MSTD_DIR}/config"
RVIZ_CONFIG_DIR = f"{MSTD_DIR}/rviz_cfg"

EXPERIMENT_DIR = f"/data/results/MSTD/230926_pseudo_loop_gt_detection"

def run_experiment(result_dir, **args):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rospy.init_node("std_detector")

    launch_files = []

    # param = copy.deepcopy(default_param)
    cli_args = [f"{LAUNCH_DIR}/pseudo_loop_gt_detector.launch"]
    cli_args.append(f"config_path:={result_dir}/config.yaml")
    cli_args.append(f"is_benchmark:=false")
    cli_args.append(f"rviz:=false")

    # for key, val in args.items():
    #     param[key] = val
    # param['lidar_path'] = lidar_path
    # param['pose_path'] = pose_path
    # param['seq_name'] = seq_name
    # param['is_benchmark'] = is_benchmark
    # param['rviz'] = is_rviz
    # print(f"lidar_path: {param['lidar_path']}")
    # print(f"pose_path: {param['pose_path']}")
    # print(f"is_benchmark: {param['is_benchmark']}")

    # for key, val in param.items():
    #     print(f"{key}:={val}")

    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    print("roslaunch_file: ", roslaunch_file)
    roslaunch_args = cli_args[1:]
    launch_files.append((roslaunch_file, roslaunch_args))

    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    parent.start()

    try:
        parent.spin()
    finally:
        parent.shutdown()

for i in range(2):
    seq_name = "park_avia"
    result_dir = f"{EXPERIMENT_DIR}/{seq_name}_{i+1:02d}"
    run_experiment(result_dir)


# # Kitti Launch
# seq_type = "kitti"
# default_param_list = rosparam.load_file(f"{CONFIG_DIR}/config_{seq_type}.yaml")
# default_param, default_ns = default_param_list[0]
# print("\n########## Parameter Setting ##########")
# for key, val in default_param.items():
#     print(f"{key:>30s}: {val}")
# print("#######################################\n")

# def run_experiment(seq_name, lidar_path, pose_path, is_benchmark, is_rviz, **args):
#     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#     roslaunch.configure_logging(uuid)
#     rospy.init_node("std_detector")


#     launch_files = []

#     param = copy.deepcopy(default_param)
#     cli_args = [f"{LAUNCH_DIR}/demo_{seq_name}.launch"]
#     for key, val in args.items():
#         param[key] = val
#     param['lidar_path'] = lidar_path
#     param['pose_path'] = pose_path
#     param['seq_name'] = seq_name
#     param['is_benchmark'] = is_benchmark
#     param['rviz'] = is_rviz
#     print(f"lidar_path: {param['lidar_path']}")
#     print(f"pose_path: {param['pose_path']}")
#     print(f"is_benchmark: {param['is_benchmark']}")

#     for key, val in param.items():
#         cli_args.append(f"{key}:={val}")
#         print(f"{key}:={val}")

#     roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
#     print("roslaunch_file: ", roslaunch_file)
#     roslaunch_args = cli_args[1:]
#     launch_files.append((roslaunch_file, roslaunch_args))

#     parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
#     parent.start()

#     try:
#         parent.spin()
#     finally:
#         parent.shutdown()

# seq_name = "kitti"
# seq_num = "00"
# lidar_path = f"/data/datasets/dataset_{seq_name}/{seq_num}/velodyne/"
# pose_path = f"/data/datasets/dataset_std/{seq_name}/{seq_name}{seq_num}.txt"
# run_experiment(seq_name, lidar_path, pose_path,
#                is_benchmark=True,
#                is_rviz=False)
