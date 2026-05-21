#!/usr/bin/env python3
"""
Usage:
    python3 ./ros_scripts/parse_rosbag_for_directory_to_webdataset.py <target_dir_list> --save_root <save_root> [options]

Example:
    # First, install webdataset: pip install webdataset
    python3 ./ros_scripts/parse_rosbag_for_directory_to_webdataset.py \
        driving_dataset/random_206/bag/ \
        --save_root processed_dataset/test_webdataset/ \
        --min_frames 100 \
        --min_distance 10.0 \
        --ego_wheel_base 0.3 \
        --ego_length 0.873 \
        --ego_width 0.54 \
        --step 1 \
        --num_workers 8

Description:
    This script performs bulk conversion of ROSBAGs into WebDataset shards (.tar).
    It uses a C++ binary for high-speed feature extraction and packages the results
    into WebDataset format, which is highly efficient for large-scale deep learning 
    training, especially over networks or with massive datasets.
"""

import argparse
import logging
import subprocess
import time
import json
import os
from multiprocessing import Pool, cpu_count
from pathlib import Path
import numpy as np
from tqdm import tqdm
import sys

try:
    import webdataset as wds
except ImportError:
    wds = None

# Reuse the binary reading logic
from convert_cpp_bin_to_python_npz import TrainingDataReader

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CPP_BINARY = (
    PROJECT_ROOT / "cpp_tools" / "build" / "autoware_diffusion_planner_tools" / "data_converter"
)

def parse_args():
    parser = argparse.ArgumentParser(description="Bulk ROSBAG to WebDataset conversion")
    parser.add_argument("target_dir_list", type=Path, nargs="+", help="List of directories to search for ROSBAGs")
    parser.add_argument("--save_root", type=Path, required=True, help="Root directory for saving output")
    parser.add_argument("--cpp_binary_path", type=Path, default=DEFAULT_CPP_BINARY)
    parser.add_argument("--step", type=int, default=1)
    parser.add_argument("--limit", type=int, default=-1)
    parser.add_argument("--min_frames", type=int, default=1700)
    parser.add_argument("--min_distance", type=float, default=50.0)
    parser.add_argument("--search_nearest_route", type=int, default=1)
    parser.add_argument("--convert_yellow", type=int, default=0)
    parser.add_argument("--convert_red", type=int, default=0)
    parser.add_argument("--interpolation", type=int, default=0)
    parser.add_argument("--ego_wheel_base", type=float, default=2.75)
    parser.add_argument("--ego_length", type=float, default=4.34)
    parser.add_argument("--ego_width", type=float, default=1.70)
    parser.add_argument("--num_workers", type=int, default=cpu_count())
    parser.add_argument("--max_samples_per_shard", type=int, default=1000, help="Maximum samples per .tar shard")
    return parser.parse_args()

def process_single_bag(args_tuple):
    (
        cpp_binary_path,
        bag_path,
        save_root,
        step,
        limit,
        min_frames,
        min_distance,
        search_nearest_route,
        convert_yellow,
        convert_red,
        interpolation,
        ego_wheel_base,
        ego_length,
        ego_width,
        max_samples_per_shard
    ) = args_tuple

    date = bag_path.parent.name
    time_str = bag_path.name

    # Map discovery logic
    map_dir = bag_path.parent.parent.parent / "map" / date
    vector_map_path = map_dir / "lanelet2_map.osm"
    if (map_dir / time_str).is_dir():
        vector_map_path = map_dir / time_str / "lanelet2_map.osm"

    # Save logic: each bag gets its own folder of shards to avoid collisions during parallel processing
    bag_save_root = (save_root / date / time_str).resolve()
    bag_save_root.mkdir(parents=True, exist_ok=True)

    # Check if shards already exist
    if list(bag_save_root.glob("webdataset_shard_*.tar")):
        return f"Skipped (already exists): {bag_save_root}"

    try:
        # 1. Run C++ extraction
        cpp_command = [
            str(cpp_binary_path),
            str(bag_path),
            str(vector_map_path),
            str(bag_save_root),
            f"--step={step}",
            f"--limit={limit}",
            f"--min_frames={min_frames}",
            f"--min_distance={min_distance}",
            f"--search_nearest_route={search_nearest_route}",
            f"--convert_yellow={convert_yellow}",
            f"--convert_red={convert_red}",
            f"--interpolation={interpolation}",
            f"--ego_wheel_base={ego_wheel_base}",
            f"--ego_length={ego_length}",
            f"--ego_width={ego_width}",
        ]
        
        result = subprocess.run(cpp_command, capture_output=True, text=True)
        if result.returncode != 0:
            return f"Error (C++ failed) for {bag_path}: {result.stderr}"

        # 2. Package into WebDataset
        bin_files = sorted(list(bag_save_root.glob("*.bin")))
        if not bin_files:
            return f"Completed (No samples) for: {bag_path}"

        reader = TrainingDataReader()
        shard_pattern = str(bag_save_root / "webdataset_shard_%06d.tar.gz")
        
        with wds.ShardWriter(shard_pattern, maxcount=max_samples_per_shard) as sink:
            for bin_file in bin_files:
                token = bin_file.stem
                json_file = bin_file.with_suffix(".json")
                
                # Read data
                data_dict = reader.read_binary_file(str(bin_file))
                
                # Load metadata if exists
                meta_data = {}
                if json_file.exists():
                    with open(json_file, 'r') as f:
                        meta_data = json.load(f)
                
                # Create WebDataset sample
                # .pyd for pickled python data (dict of numpy arrays)
                # .json for readable metadata
                sample = {
                    "__key__": token,
                    "pyd": data_dict,
                    "json": meta_data
                }
                sink.write(sample)
                
                # Clean up temporary files
                bin_file.unlink()
                if json_file.exists():
                    json_file.unlink()

        return f"Completed: {bag_save_root} ({len(bin_files)} frames -> Compressed WebDataset shards)"
    except Exception as e:
        return f"Error processing {bag_path}: {str(e)}"

if __name__ == "__main__":
    if wds is None:
        print("Error: 'webdataset' is not installed. Please install it with: pip install webdataset")
        sys.exit(1)

    start_time = time.perf_counter()
    args = parse_args()
    
    args.save_root.mkdir(parents=True, exist_ok=True)

    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
        handlers=[
            logging.FileHandler(args.save_root / "webdataset_conversion.log", mode="w"),
            logging.StreamHandler()
        ],
    )

    metadata_list = []
    for target_dir in args.target_dir_list:
        metadata_list.extend(list(target_dir.glob("**/metadata.yaml")))
    
    bag_dir_list = sorted(list(set(m.parent for m in metadata_list if m.is_file())))

    logging.info(f"Found {len(bag_dir_list)} bag directories to process")
    logging.info(f"Using {args.num_workers} parallel workers")

    process_args = [
        (
            args.cpp_binary_path,
            bag_path,
            args.save_root,
            args.step,
            args.limit,
            args.min_frames,
            args.min_distance,
            args.search_nearest_route,
            args.convert_yellow,
            args.convert_red,
            args.interpolation,
            args.ego_wheel_base,
            args.ego_length,
            args.ego_width,
            args.max_samples_per_shard
        )
        for bag_path in bag_dir_list
    ]

    with Pool(processes=args.num_workers) as pool:
        for result in tqdm(pool.imap_unordered(process_single_bag, process_args), total=len(bag_dir_list), desc="Bags processed"):
            logging.info(result)

    elapsed_seconds = int(time.perf_counter() - start_time)
    hours, minutes, seconds = elapsed_seconds // 3600, (elapsed_seconds % 3600) // 60, elapsed_seconds % 60
    logging.info(f"Total elapsed time: {hours:02d}:{minutes:02d}:{seconds:02d}")
