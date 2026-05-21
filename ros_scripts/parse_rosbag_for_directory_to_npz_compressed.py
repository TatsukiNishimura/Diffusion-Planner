#!/usr/bin/env python3
"""
Usage:
    python3 ./ros_scripts/parse_rosbag_for_directory_to_npz_compressed.py <target_dir_list> --save_root <save_root> [options]

Example:
    python3 ./ros_scripts/parse_rosbag_for_directory_to_npz_compressed.py \
        /path/to/data_dir1 /path/to/data_dir2 \
        --save_root /path/to/output_datasets \
        --step 1 \
        --num_workers 8

Description:
    This script performs bulk conversion of ROSBAGs into compressed .npz files.
    It automatically discovers ROSBAGs in the specified directories, locates the corresponding
    lanelet2 maps, and uses a C++ binary for high-speed feature extraction.
    The final output is compressed using np.savez_compressed to minimize storage usage.
"""

import argparse
import logging
import subprocess
import time
from multiprocessing import Pool, cpu_count
from pathlib import Path
import numpy as np
from tqdm import tqdm

# Reuse the binary reading logic
from convert_cpp_bin_to_python_npz import TrainingDataReader

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CPP_BINARY = (
    PROJECT_ROOT / "cpp_tools" / "build" / "autoware_diffusion_planner_tools" / "data_converter"
)

def parse_args():
    parser = argparse.ArgumentParser(description="Bulk ROSBAG to compressed NPZ conversion")
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
    return parser.parse_args()

def convert_bin_to_compressed_npz(bin_file, save_dir):
    """Convert a single .bin file to compressed .npz"""
    try:
        reader = TrainingDataReader()
        data = reader.read_binary_file(str(bin_file))
        output_file = save_dir / f"{bin_file.stem}.npz"
        np.savez_compressed(str(output_file), **data)
        bin_file.unlink() # Clean up intermediate bin
        return True
    except Exception as e:
        logging.error(f"Error converting {bin_file}: {e}")
        return False

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
    ) = args_tuple

    date = bag_path.parent.name
    time_str = bag_path.name

    # Map discovery logic matching the original script
    map_dir = bag_path.parent.parent.parent / "map" / date
    vector_map_path = map_dir / "lanelet2_map.osm"
    if (map_dir / time_str).is_dir():
        vector_map_path = map_dir / time_str / "lanelet2_map.osm"

    (save_root / date).mkdir(parents=True, exist_ok=True)
    save_dir = (save_root / date / time_str).resolve()

    if save_dir.is_dir() and list(save_dir.glob("*.npz")):
        return f"Skipped (already exists): {save_dir}"

    save_dir.mkdir(parents=True, exist_ok=True)

    try:
        # 1. Run C++ extraction
        cpp_command = [
            str(cpp_binary_path),
            str(bag_path),
            str(vector_map_path),
            str(save_dir),
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

        # 2. Compressed NPZ conversion for this bag
        bin_files = list(save_dir.glob("*.bin"))
        for bin_file in bin_files:
            convert_bin_to_compressed_npz(bin_file, save_dir)
            
        return f"Completed: {save_dir} ({len(bin_files)} frames)"
    except Exception as e:
        return f"Error processing {bag_path}: {str(e)}"

if __name__ == "__main__":
    start_time = time.perf_counter()
    args = parse_args()
    
    args.save_root.mkdir(parents=True, exist_ok=True)

    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
        handlers=[
            logging.FileHandler(args.save_root / "bulk_conversion.log", mode="w"),
            logging.StreamHandler()
        ],
    )

    # Search for "metadata.yaml" to identify ROSBAGs
    metadata_list = []
    for target_dir in args.target_dir_list:
        metadata_list.extend(list(target_dir.glob("**/metadata.yaml")))
    
    bag_dir_list = sorted(list(set(m.parent for m in metadata_list if m.is_file())))

    logging.info(f"Found {len(bag_dir_list)} bag directories to process")
    logging.info(f"Using {args.num_workers} parallel workers")

    # Prepare arguments for parallel bag processing
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
        )
        for bag_path in bag_dir_list
    ]

    # Process bags in parallel
    with Pool(processes=args.num_workers) as pool:
        # Using imap_unordered to see progress as it happens
        for result in tqdm(pool.imap_unordered(process_single_bag, process_args), total=len(bag_dir_list), desc="Bags processed"):
            logging.info(result)

    elapsed_seconds = int(time.perf_counter() - start_time)
    hours, minutes, seconds = elapsed_seconds // 3600, (elapsed_seconds % 3600) // 60, elapsed_seconds % 60
    logging.info(f"Total elapsed time: {hours:02d}:{minutes:02d}:{seconds:02d}")
