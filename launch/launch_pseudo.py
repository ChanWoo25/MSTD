import roslaunch
import rospy

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

    cli_args = [f"{LAUNCH_DIR}/pseudo_loop_gt_detector.launch"]
    cli_args.append(f"config_path:={result_dir}/config.yaml")
    cli_args.append(f"is_benchmark:=false")
    cli_args.append(f"rviz:=true")

    launch_files = []
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

for seq in [1, 2]:
    for valid_voxel_thres in [1, 3, 5, 10]:
        seq_name = "park_avia"
        result_dir = f"{EXPERIMENT_DIR}/{seq_name}_{seq:02d}_vvt={valid_voxel_thres}"
        run_experiment(result_dir)
