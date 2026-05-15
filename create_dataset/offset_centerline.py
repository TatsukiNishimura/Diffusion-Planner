import os
import json
import random
import math
import xml.etree.ElementTree as ET

def calculate_normals(nodes):
    """経路上の各ノードにおける進行方向の法線（左方向）ベクトルを計算する"""
    normals = []
    n = len(nodes)
    if n < 2:
        return [(0.0, 0.0)] * n

    for i in range(n):
        if i == 0:
            dx = nodes[1]['x'] - nodes[0]['x']
            dy = nodes[1]['y'] - nodes[0]['y']
        elif i == n - 1:
            dx = nodes[n-1]['x'] - nodes[n-2]['x']
            dy = nodes[n-1]['y'] - nodes[n-2]['y']
        else:
            dx = nodes[i+1]['x'] - nodes[i-1]['x']
            dy = nodes[i+1]['y'] - nodes[i-1]['y']
        
        length = math.hypot(dx, dy)
        if length == 0:
            normals.append((0.0, 0.0))
        else:
            normals.append((-dy / length, dx / length))
            
    return normals

def calculate_curvature_weights(nodes, max_angle_deg=45.0):
    """
    各ノードでの曲がり具合（前後のベクトルのなす角）を計算し、
    カーブがきついほど 0 に近づく減衰係数(0.0 ~ 1.0)を返す。
    """
    n = len(nodes)
    weights = [1.0] * n
    if n < 3:
        return weights

    max_angle_rad = math.radians(max_angle_deg)

    for i in range(1, n - 1):
        dx1 = nodes[i]['x'] - nodes[i-1]['x']
        dy1 = nodes[i]['y'] - nodes[i-1]['y']
        
        dx2 = nodes[i+1]['x'] - nodes[i]['x']
        dy2 = nodes[i+1]['y'] - nodes[i]['y']
        
        len1 = math.hypot(dx1, dy1)
        len2 = math.hypot(dx2, dy2)
        
        if len1 == 0 or len2 == 0:
            continue
            
        # 内積から角度(ラジアン)を計算
        cos_theta = (dx1 * dx2 + dy1 * dy2) / (len1 * len2)
        # 浮動小数点の誤差対策（acosの定義域 -1.0 ~ 1.0 に収める）
        cos_theta = max(-1.0, min(1.0, cos_theta))
        angle = math.acos(cos_theta)
        
        # 角度が0(直線)のとき1.0、max_angle_deg以上のとき0.0になるよう線形に減衰
        weight = max(0.0, 1.0 - (angle / max_angle_rad))
        weights[i] = weight
        
    # 前後のノード間でオフセット量が急変しないよう移動平均でスムージング
    smoothed_weights = [1.0] * n
    smoothed_weights[0] = (weights[0] * 2 + weights[1]) / 3
    smoothed_weights[-1] = (weights[-1] * 2 + weights[-2]) / 3
    for i in range(1, n - 1):
        smoothed_weights[i] = (weights[i-1] + weights[i]*2 + weights[i+1]) / 4
        
    return smoothed_weights

def apply_offset_to_osm(input_file, output_file, lanelet_offsets):
    """指定されたオフセット量でOSMのCenterlineを書き換えて保存する"""
    tree = ET.parse(input_file)
    root = tree.getroot()

    # Nodeのパース
    nodes_data = {}
    for node in root.findall('node'):
        node_id = node.get('id')
        x, y = None, None
        x_tag, y_tag = None, None
        for tag in node.findall('tag'):
            if tag.get('k') == 'local_x':
                x = float(tag.get('v'))
                x_tag = tag
            elif tag.get('k') == 'local_y':
                y = float(tag.get('v'))
                y_tag = tag
        if x is not None and y is not None:
            nodes_data[node_id] = {'x': x, 'y': y, 'x_tag': x_tag, 'y_tag': y_tag}

    # Wayのパース
    ways_data = {way.get('id'): [nd.get('ref') for nd in way.findall('nd')] for way in root.findall('way')}

    # 対象Wayの特定
    centerline_info = {}
    for rel in root.findall('relation'):
        rel_id = rel.get('id')
        if any(tag.get('k') == 'type' and tag.get('v') == 'lanelet' for tag in rel.findall('tag')):
            for member in rel.findall('member'):
                if member.get('type') == 'way' and member.get('role') == 'centerline':
                    way_id = member.get('ref')
                    offset = lanelet_offsets.get(rel_id, 0.0)
                    centerline_info[way_id] = {'max_offset': offset}

    # 座標の書き換え（距離ベースのサイン波 ＋ カーブ減衰）
    processed_nodes = set()
    for way_id, info in centerline_info.items():
        if way_id not in ways_data:
            continue
            
        way_nodes = [{'id': ref, 'x': nodes_data[ref]['x'], 'y': nodes_data[ref]['y']} 
                     for ref in ways_data[way_id] if ref in nodes_data]
        
        n_nodes = len(way_nodes)
        if n_nodes < 2:
            continue

        normals = calculate_normals(way_nodes)
        # カーブのきつさに応じた減衰係数を計算 (45度以上でオフセット0に向かう)
        curvature_weights = calculate_curvature_weights(way_nodes, max_angle_deg=45.0) 
        max_offset = info['max_offset']
        
        # 距離ベースでサイン波を計算するための累積距離計算
        distances = [0.0] * n_nodes
        for i in range(1, n_nodes):
            dx = way_nodes[i]['x'] - way_nodes[i-1]['x']
            dy = way_nodes[i]['y'] - way_nodes[i-1]['y']
            distances[i] = distances[i-1] + math.hypot(dx, dy)
        total_length = distances[-1]
        
        for i, node_info in enumerate(way_nodes):
            node_id = node_info['id']
            
            # インデックスではなく距離ベースのサイン波を生成
            if total_length > 0:
                scale = math.sin(math.pi * distances[i] / total_length)
            else:
                scale = 0
                
            # サイン波 × カーブ減衰係数 で最終的なオフセットを決定
            current_offset = max_offset * scale * curvature_weights[i]
            
            if node_id not in processed_nodes and current_offset != 0:
                nx, ny = normals[i]
                new_x = node_info['x'] + nx * current_offset
                new_y = node_info['y'] + ny * current_offset
                nodes_data[node_id]['x_tag'].set('v', f"{new_x:.4f}")
                nodes_data[node_id]['y_tag'].set('v', f"{new_y:.4f}")
                processed_nodes.add(node_id)

    # 保存
    tree.write(output_file, encoding='utf-8', xml_declaration=True)


def generate_map_dataset(input_osm, base_output_dir, n_maps, lanelet_ids, start_index=1, mu=0.0, sigma=0.2, limit=0.5):
    """N個のランダムなマップを生成し、フォルダ別に保存する"""
    
    # ベースフォルダが存在しなければ作成
    os.makedirs(base_output_dir, exist_ok=True)
    
    end_index = start_index + n_maps - 1
    print(f"Generating {n_maps} maps (from map_{start_index:03d} to map_{end_index:03d}) in '{base_output_dir}'...")

    # start_index から指定した数だけ生成
    for i in range(start_index, start_index + n_maps):
        # フォルダ名の決定 (例: map_001, map_051 ...)
        map_id_str = f"map_{i:03d}"
        target_dir = os.path.join(base_output_dir, map_id_str)
        os.makedirs(target_dir, exist_ok=True)
        
        # 出力ファイルパス
        output_osm_path = os.path.join(target_dir, 'lanelet2_map.osm')
        output_json_path = os.path.join(target_dir, 'metadata.json')
        
        # --- 再現性のためのシード値設定 ---
        # フォルダ番号(i)をシード値にすることで、どの番号から再開しても同じ番号には同じ形状が生成される
        seed_value = 1000 + i 
        random.seed(seed_value)
        
        # --- 正規分布を用いたランダムオフセットの生成 ---
        current_offsets = {}
        for lid in lanelet_ids:
            val = random.gauss(mu, sigma)
            # 道路外にはみ出さないようにクリッピング
            val = max(min(val, limit), -limit)
            current_offsets[lid] = round(val, 4)
            
        # 1. OSMファイルの生成と保存
        apply_offset_to_osm(input_osm, output_osm_path, current_offsets)
        
        # 2. メタデータ(JSON)の保存
        metadata = {
            "map_id": map_id_str,
            "seed": seed_value,
            "offset_distribution": {"mu": mu, "sigma": sigma, "limit": limit},
            "offsets": current_offsets
        }
        with open(output_json_path, 'w') as f:
            json.dump(metadata, f, indent=4)
            
        # 進捗の表示 (10件ごと、または最後の1件)
        generated_count = i - start_index + 1
        if generated_count % 10 == 0 or i == end_index:
            print(f"  [{generated_count}/{n_maps}] Generated {map_id_str}")

    print("All maps generated successfully!")


if __name__ == "__main__":
    # ======== 設定 ========
    INPUT_OSM_FILE = 'lanelet2_map.osm'      # 元となるOSMファイル
    OUTPUT_BASE_DIR = 'dataset_maps'         # 生成先の大元のフォルダ
    
    START_INDEX = 33                        # 連番を始める番号 (例: 51から始めたい場合は 51 に変更)
    N_MAPS_TO_GENERATE = 466                 # 生成するパターンの数 (例: START_INDEX=51, N=50なら 51〜100を作成)
    
    # オフセットをランダムに変化させたいLaneletのIDリスト
    # TARGET_LANELETS = ['33', '38', '43', '48', '53', '58', '63', '68', '73', '78'] 
    TARGET_LANELETS = ['33', '58', '68', '78']
    
    # 正規分布のパラメータ (mu:平均, sigma:標準偏差, limit:最大オフセット制限m)
    MU = 0.0
    SIGMA = 0.3
    LIMIT = 0.8
    # ======================

    generate_map_dataset(
        input_osm=INPUT_OSM_FILE, 
        base_output_dir=OUTPUT_BASE_DIR, 
        n_maps=N_MAPS_TO_GENERATE, 
        lanelet_ids=TARGET_LANELETS,
        start_index=START_INDEX,
        mu=MU, sigma=SIGMA, limit=LIMIT
    )