import subprocess
import time
import os
import signal
from datetime import datetime

def run_simulation_loop(base_map_path, n_maps, wait_time_per_sim=30):
    """
    連番のマップを順番に読み込んでシミュレータを起動・終了する
    """
    # 基準となるマップディレクトリ（絶対パスに変換）
    abs_base_path = os.path.abspath(base_map_path)
    bag_base_dir = "./rosbags"
    os.makedirs(bag_base_dir, exist_ok=True)
    
    print(f"Starting simulation loop for {n_maps} maps.")
    print(f"Base Map Path: {abs_base_path}")

    for i in range(1, n_maps + 1):
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
        process = subprocess.Popen(cmd,start_new_session=True)

        # ---------------------------------------------------------
        # TODO: ここで ROSBAG record や 経路送信(API) の処理を挟む
        # ---------------------------------------------------------
        print(f"Waiting for {wait_time_per_sim} seconds...")
        time.sleep(wait_time_per_sim)


        # ---------------------------------------------------------
        # ROSBAG 記録の開始
        # ---------------------------------------------------------
        # タイムスタンプの取得 (例: 20260514_171622)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # 出力先のディレクトリ名 (例: /.../rosbags/map_001_20260514_171622)
        bag_out_dir = os.path.join(bag_base_dir, f"{map_id_str}_{timestamp}")

        # 記録するトピックのリスト
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

        # コマンドの組み立て: ros2 bag record -o <出力先> <トピック1> <トピック2> ...
        rosbag_cmd = ["ros2", "bag", "record", "-o", bag_out_dir] + topics_to_record
        
        print(f"Starting rosbag record: {bag_out_dir}")
        rosbag_proc = subprocess.Popen(rosbag_cmd, start_new_session=True)



        goal_log_path = "/tmp/goals_achieved.log"

        # 1. 'rm -f' の処理を Python 側で行う（ファイルが存在する場合のみ削除）
        if os.path.exists(goal_log_path):
            os.remove(goal_log_path)

        # 2. コマンドをリスト形式に分解する
        goal_sender_cmd = [
            "ros2",
            "launch",
            "tier4_automatic_goal_rviz_plugin",
            "automatic_goal_sender.launch.xml",
            "goals_list_file_path:=/home/tatsukinishimura/Diffusion-Planner/create_dataset/goals_list.yaml",
            "goals_achieved_dir_path:=/tmp/",
            "allow_loop:=false",
        ]

        # 3. shell=True なしで起動
        goal_proc = subprocess.Popen(goal_sender_cmd,start_new_session=True)

        time.sleep(2)
        with open(goal_log_path, 'r', encoding='utf-8') as f:
            # ファイルの最後までシーク（既存の過去ログを無視する場合）
            f.seek(0, os.SEEK_END)
            
            while True:
                line = f.readline()
                print(line)
                if not line:
                    # 新しい行がない場合は指定秒数待機
                    time.sleep(5)
                    continue
                
                # 読み込んだ行に特定の文面が含まれているか確認
                if "G1" in line:
                    print(f"発見しました: {line.strip()}")
                    break

      # 2. 【修正】 安全な終了処理（プロセスグループ全体に送る）
        print(f"Terminating all nodes for {map_id_str}...")
        
        for p in [process, goal_proc,rosbag_proc]:
            try:
                # PIDではなくグループID(pgid)に対して信号を送る
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
                
                try:
                    p.wait(timeout=3) # Autowareは重いので長めに待つ
                except subprocess.TimeoutExpired:
                    print("Force killing process group...")
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    p.wait()
            except ProcessLookupError:
                pass # 既に死んでいる場合は無視

        # 3. 【追加】 念には念を：名前で残党を狩る
        # launchファイル名を含むプロセスを全て終了させる
        subprocess.run(["pkill", "-f", "only_map.launch.xml"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-f", "automatic_goal_sender.launch.xml"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-f", "ros2 bag record"], stderr=subprocess.DEVNULL) # これも追加しておくと安心
        
        # ROS 2のデーモンが残って悪さをすることもあるので、不安ならこれも追加
        # subprocess.run(["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL)

        print(f"Finished {map_id_str}. Waiting for cleanup...")
        time.sleep(3) # 次のループへ行く前に、ポートや共有メモリが解放されるのを待つ

if __name__ == "__main__":
    # treeコマンドの結果に基づいたパス設定
    BASE_MAP_DIR = "/home/tatsukinishimura/autoware_map/room206_for_data_collection/" # room206_for_data_collection フォルダ内で実行する場合
    TOTAL_MAPS = 10    # 生成済みのマップ数
    
    run_simulation_loop(BASE_MAP_DIR, TOTAL_MAPS,wait_time_per_sim=3)