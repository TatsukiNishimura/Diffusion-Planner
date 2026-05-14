import subprocess
import time
import os
import signal

def publish_initial_pose_via_cmd():
    # トピック名
    topic_name = "/initialpose"
    # メッセージ型
    msg_type = "geometry_msgs/msg/PoseWithCovarianceStamped"
    
    # YAML形式のデータ作成
    # タイムスタンプは 'stamp: {sec: 0, nanosec: 0}' とすることで、
    # 受信側のAutowareが現在の時刻として処理してくれることが多いです。
    data = (
        "{"
        "header: {frame_id: 'map'}, "
        "pose: {"
            "pose: {"
                "position: {x: 0.010169, y: -0.127890, z: 0.0}, "
                "orientation: {x: 0.0, y: 0.0, z: 0.725101, w: 0.688642}"
            "}, "
            "covariance: ["
                "0.25, 0.0, 0.0, 0.0, 0.0, 0.0, "
                "0.0, 0.25, 0.0, 0.0, 0.0, 0.0, "
                "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
                "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
                "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
                "0.0, 0.0, 0.0, 0.0, 0.0, 0.068539"
            "]"
        "}"
        "}"
    )

    # コマンドの組み立て
    # --once: 1回だけ送信して終了する
    cmd = [
        "ros2", "topic", "pub", "--once",
        topic_name,
        msg_type,
        data
    ]

    print(f"Sending Initial Pose...")
    # 実行
    subprocess.run(cmd)

def run_simulation_loop(base_map_path, n_maps, wait_time_per_sim=30):
    """
    連番のマップを順番に読み込んでシミュレータを起動・終了する
    """
    # 基準となるマップディレクトリ（絶対パスに変換）
    abs_base_path = os.path.abspath(base_map_path)
    
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

        # # 一応５回くらい送っておく
        # for _ in range(5):
        #     publish_initial_pose_via_cmd()
        #     time.sleep(0.1)
        # time.sleep(1)


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
        
        for p in [process, goal_proc]:
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
        
        # ROS 2のデーモンが残って悪さをすることもあるので、不安ならこれも追加
        # subprocess.run(["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL)

        print(f"Finished {map_id_str}. Waiting for cleanup...")
        time.sleep(3) # 次のループへ行く前に、ポートや共有メモリが解放されるのを待つ

if __name__ == "__main__":
    # treeコマンドの結果に基づいたパス設定
    BASE_MAP_DIR = "/home/tatsukinishimura/autoware_map/room206_for_data_collection/" # room206_for_data_collection フォルダ内で実行する場合
    TOTAL_MAPS = 10    # 生成済みのマップ数
    
    run_simulation_loop(BASE_MAP_DIR, TOTAL_MAPS,wait_time_per_sim=3)