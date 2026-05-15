# Autoware Map Perturbation & Simulation Dataset Collector

このプロジェクトは、Autowareを用いた学習・評価用データセットを自動生成するためのツール群です。既存のLanelet2マップのセンターラインをランダムに変形（摂動）させ、多様な走行環境を作成した上で、シミュレーション走行とrosbag記録を自動化します。

---

## 1. Map Dataset Generator (`offset_centerline.py`)

既存のLanelet2マップ（`.osm`）を読み込み、指定したLaneletのセンターラインを法線方向にオフセットさせて、バリエーション豊かな地図データセットを作成します。

### 主な機能
* **サイン波オフセット**: 道路の始点と終点は固定し、中央付近のオフセットが最大になるよう滑らかに変形させます。
* **カーブ減衰アルゴリズム**: 急なカーブ（デフォルト45度以上）では、マップの破綻を防ぐためにオフセット量を自動的に0に近づけます。
* **正規分布によるランダム性**: 平均 `mu`、標準偏差 `sigma` に基づくランダムなオフセット量を適用します。
* **再現性の確保**: フォルダ番号に基づいたシード値を設定しており、同じ番号のマップは常に同じ形状で再生成可能です。

### 設定パラメータ (mainブロック内)
| 変数名 | 説明 |
| :--- | :--- |
| `INPUT_OSM_FILE` | 元となるLanelet2マップのパス |
| `START_INDEX` | 生成を開始する連番の開始値 (例: 33) |
| `N_MAPS_TO_GENERATE` | 生成するマップの総数 |
| `TARGET_LANELETS` | オフセットを適用するLanelet IDのリスト |
| `MU` / `SIGMA` | オフセット量の正規分布パラメータ |
| `LIMIT` | 最大オフセット制限（メートル） |

---

## 2. Simulation Loop Runner (`record_start.py`)

生成されたマップデータセットを順番に読み込み、Autowareの起動、走行、データ記録、終了のサイクルを自動で回します。

### 主な機能
* **自動ローンチ**: 各マップを個別に読み込み、`only_map.launch.xml` を介してAutowareを起動します。
* **自動ゴール送信**: `automatic_goal_sender.launch.xml` を使用し、あらかじめ定義された `goals_list.yaml` に基づいて車両に目的地を送信します。
* **rosbag 自動記録**: 走行中の主要なトピック（自己位置、物体認識、信号、経路、TF等）を自動的に保存します。
* **終了検知**: `/tmp/goals_achieved.log` を監視し、車両がゴールに到達したことを検知して次のシミュレーションへ移行します。

### コマンドライン引数
| 引数 | 説明 | デフォルト値 |
| :--- | :--- | :--- |
| `--start` | シミュレーションを開始するマップの連番 (例: `--start 33`) | `1` |

### 内部設定
* **記録トピック**: `/localization/kinematic_state`, `/perception/object_recognition/tracking/objects`, `/tf` など、データセット作成に必要な情報を網羅しています。
* **保存先**: `rosbags/map_XXX_YYYYMMDD_HHMMSS/` フォルダに保存されます。

---

## クイックスタート

### 1. マップの生成
まず、必要な数だけ変形済みマップを作成します。
```bash
python3 offset_centerline.py
```

### 2. シミュレーションの実行
生成されたマップ（例：33番以降）を使用してシミュレーションを開始します。
```
python3 record_start.py --start 33
```

## 必要環境
- Autoware: autoware_launch および関連パッケージ
- Plugin: tier4_automatic_goal_rviz_plugin
- Python 3: xml.etree.ElementTree などの標準ライブラリを使用