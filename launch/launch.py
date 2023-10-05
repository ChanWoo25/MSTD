import roslaunch
import rospy
from pathlib import Path

WORKSPACE = "/root/ws_std"
MSTD_DIR = f"{WORKSPACE}/src/MSTD"
LAUNCH_DIR = f"{MSTD_DIR}/launch"
CONFIG_DIR = f"{MSTD_DIR}/config"
RVIZ_CONFIG_DIR = f"{MSTD_DIR}/rviz_cfg"


rospy.init_node("std_detector")

def run_experiment(result_dir):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cli_args = [f"{LAUNCH_DIR}/demo_livox_park_avia.launch"]
    cli_args.append(f"config_path:={result_dir}/config.yaml")
    cli_args.append(f"rviz:=false")

    launch_files = []
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    roslaunch_args = cli_args[1:]
    launch_files.append((roslaunch_file, roslaunch_args))

    print("roslaunch_file: ", roslaunch_file)
    rospy.sleep(1.0)
    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    parent.start()

    try:
        parent.spin()
    finally:
        parent.shutdown()
        rospy.sleep(2.0)

EXPERIMENT_DIR = f"/data/results/MSTD/231003_except_trees"

import argparse
parser = argparse.ArgumentParser()
parser.add_argument(
    "--seq_id",
    type=int, action="store")
parser.add_argument(
    "--z_max",
    type=float, action="store")
args = parser.parse_args()

seq_name = "park_avia"
result_dir = f"{EXPERIMENT_DIR}/{seq_name}_{args.seq_id:02d}-z_max={args.z_max:.1f}"
if not Path(result_dir).exists():
    print(f"{result_dir} doesn't exists!")
    exit(1)

run_experiment(result_dir)
