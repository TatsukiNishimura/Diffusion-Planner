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

    # 座標の書き換え（サイン波による滑らかな接続）
    processed_nodes = set()
    for way_id, info in centerline_info.items():
        if way_id not in ways_data:
            continue
            
        way_nodes = [{'id': ref, 'x': nodes_data[ref]['x'], 'y': nodes_data[ref]['y']} 
                     for ref in ways_data[way_id] if ref in nodes_data]
        
        if len(way_nodes) < 2:
            continue

        normals = calculate_normals(way_nodes)
        max_offset = info['max_offset']
        n_nodes = len(way_nodes)
        
        for i, node_info in enumerate(way_nodes):
            node_id = node_info['id']
            scale = math.sin(math.pi * i / (n_nodes - 1)) if n_nodes > 1 else 0
            current_offset = max_offset * scale
            
            if node_id not in processed_nodes and current_offset != 0:
                nx, ny = normals[i]
                new_x = node_info['x'] + nx * current_offset
                new_y = node_info['y'] + ny * current_offset
                nodes_data[node_id]['x_tag'].set('v', f"{new_x:.4f}")
                nodes_data[node_id]['y_tag'].set('v', f"{new_y:.4f}")
                processed_nodes.add(node_id)

    # 保存
    tree.write(output_file, encoding='utf-8', xml_declaration=True)


def generate_map_dataset(input_osm, base_output_dir, n_maps, lanelet_ids, mu=0.0, sigma=0.2, limit=0.5):
    """N個のランダムなマップを生成し、フォルダ別に保存する"""
    
    # ベースフォルダが存在しなければ作成
    os.makedirs(base_output_dir, exist_ok=True)
    
    print(f"Generating {n_maps} maps in '{base_output_dir}'...")

    for i in range(1, n_maps + 1):
        # フォルダ名の決定 (例: map_001, map_002 ...)
        map_id_str = f"map_{i:03d}"
        target_dir = os.path.join(base_output_dir, map_id_str)
        os.makedirs(target_dir, exist_ok=True)
        
        # 出力ファイルパス
        output_osm_path = os.path.join(target_dir, 'lanelet2_map.osm')
        output_json_path = os.path.join(target_dir, 'metadata.json')
        
        # --- 再現性のためのシード値設定 ---
        # フォルダ番号(i)をシード値にすることで、何度実行してもmap_001は同じ形状になる
        seed_value = 1000 + i 
        random.seed(seed_value)
        
        # --- 正規分布を用いたランダムオフセットの生成 ---
        current_offsets = {}
        for lid in lanelet_ids:
            val = random.gauss(mu, sigma)
            # 道路外にはみ出さないようにクリッピング
            val = max(min(val, limit), -limit)
            current_offsets[lid] = round(val, 4) # JSONが見やすいように丸める
            
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
            
        if i % 10 == 0 or i == n_maps:
            print(f"  [{i}/{n_maps}] Generated {map_id_str}")

    print("All maps generated successfully!")


if __name__ == "__main__":
    # ======== 設定 ========
    INPUT_OSM_FILE = 'lanelet2_map.osm'      # 元となるOSMファイル
    OUTPUT_BASE_DIR = 'dataset_maps'         # 生成先の大元のフォルダ
    N_MAPS_TO_GENERATE = 10                  # 生成するパターンの数
    
    # オフセットをランダムに変化させたいLaneletのIDリスト
    # （ご提示いただいたファイルから抽出した全IDです）
    TARGET_LANELETS = ['33', '38', '43', '48', '53', '58', '63', '68', '73', '78'] 
    
    # 正規分布のパラメータ (mu:平均, sigma:標準偏差, limit:最大オフセット制限m)
    MU = 0.0
    SIGMA = 0.2
    LIMIT = 0.5
    # ======================

    generate_map_dataset(
        input_osm=INPUT_OSM_FILE, 
        base_output_dir=OUTPUT_BASE_DIR, 
        n_maps=N_MAPS_TO_GENERATE, 
        lanelet_ids=TARGET_LANELETS,
        mu=MU, sigma=SIGMA, limit=LIMIT
    )