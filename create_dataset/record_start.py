import subprocess
import time
import os
import signal
import argparse  # 追加
from datetime import datetime

def run_simulation_loop(base_map_path, n_maps, wait_time_per_sim=30, start_map=1): # start_mapを追加
    """
    連番のマップを順番に読み込んでシミュレータを起動・終了する
    """
    # 基準となるマップディレクトリ（絶対パスに変換）
    abs_base_path = os.path.abspath(base_map_path)
    bag_base_dir = "./rosbags"
    os.makedirs(bag_base_dir, exist_ok=True)
    
    print(f"Starting simulation loop for {n_maps} maps. (Starting from map_{start_map:03d})")
    print(f"Base Map Path: {abs_base_path}")

    # rangeの開始を start_map に変更
    for i in range(start_map, n_maps + 1):
        map_id_str = f"map_{i:03d}"
        # 引数として渡す相対パスを作成
        lanelet_file_rel_path = f"dataset_maps/{map_id_str}/lanelet2_map.osm"
        
        print(f"\n[{i}/{n_maps}] Running Simulation with: {map_id_str}")
        
        # 起動コマンド
        cmd = [
            "ros2", "launch", "autoware_launch", "only_map.launch.xml",
            f"map_path:={abs_base_path}",
            f"lanelet2_map_file:={lanelet_file_rel_path}",
            "vehicle_model:=palta",
            "sensor_model:=palta_sensor_kit",
            "pointcloud_map_file:=pointcloud",
            "use_lidar_host_time_stamp:=true",
            "gnss_enabled:=false",
            "rviz_initial_pose_auto_fix_target:=vector_map",
            "require_rosbag_for_auto:=false"
        ]

        # プロセス起動
        process = subprocess.Popen(cmd, start_new_session=True)

        print(f"Waiting for {wait_time_per_sim} seconds...")
        time.sleep(wait_time_per_sim)

        # ---------------------------------------------------------
        # ROSBAG 記録の開始
        # ---------------------------------------------------------
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_out_dir = os.path.join(bag_base_dir, f"{map_id_str}_{timestamp}")

        topics_to_record = [
            "/localization/kinematic_state",
            "/localization/acceleration",
            "/perception/object_recognition/tracking/objects",
            "/perception/traffic_light_recognition/traffic_signals",
            "/planning/mission_planning/route",
            "/vehicle/status/turn_indicators_status",
            "/tf",
            "/tf_static"
        ]

        rosbag_cmd = ["ros2", "bag", "record", "-o", bag_out_dir] + topics_to_record
        
        print(f"Starting rosbag record: {bag_out_dir}")
        rosbag_proc = subprocess.Popen(rosbag_cmd, start_new_session=True)

        goal_log_path = "/tmp/goals_achieved.log"

        if os.path.exists(goal_log_path):
            os.remove(goal_log_path)

        goal_sender_cmd = [
            "ros2",
            "launch",
            "tier4_automatic_goal_rviz_plugin",
            "automatic_goal_sender.launch.xml",
            "goals_list_file_path:=/home/tatsukinishimura/Diffusion-Planner/create_dataset/goals_list.yaml",
            "goals_achieved_dir_path:=/tmp/",
            "allow_loop:=false",
        ]

        goal_proc = subprocess.Popen(goal_sender_cmd, start_new_session=True)

        time.sleep(2)
        with open(goal_log_path, 'r', encoding='utf-8') as f:
            f.seek(0, os.SEEK_END)
            
            while True:
                line = f.readline()
                print(line, end="") # 改行が二重になるのを防ぐため end="" を追加
                if not line:
                    time.sleep(5)
                    continue
                
                if "G1" in line:
                    print(f"\n発見しました: {line.strip()}")
                    break

        # ---------------------------------------------------------
        # 終了処理
        # ---------------------------------------------------------
        print(f"Terminating all nodes for {map_id_str}...")
        
        for p in [process, goal_proc, rosbag_proc]:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    print("Force killing process group...")
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    p.wait()
            except ProcessLookupError:
                pass 

        subprocess.run(["pkill", "-f", "only_map.launch.xml"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-f", "automatic_goal_sender.launch.xml"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-f", "ros2 bag record"], stderr=subprocess.DEVNULL) 
        
        print(f"Finished {map_id_str}. Waiting for cleanup...")
        time.sleep(3)

if __name__ == "__main__":
    # コマンドライン引数の設定
    parser = argparse.ArgumentParser(description="Run Autoware simulation loop for multiple maps.")
    parser.add_argument(
        "--start", 
        type=int, 
        default=1, 
        help="開始するマップの番号（例: 3 を指定すると map_003 から開始）"
    )
    args = parser.parse_args()

    BASE_MAP_DIR = "/home/tatsukinishimura/autoware_map/room206_for_data_collection/" 
    TOTAL_MAPS = 500
    
    # 引数から受け取った開始番号を関数に渡す
    run_simulation_loop(BASE_MAP_DIR, TOTAL_MAPS, wait_time_per_sim=3, start_map=args.start)