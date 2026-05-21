#!/usr/bin/env python3
"""
Usage:
    python3 diffusion_planner/util_scripts/generate_wds_list.py <target_dir> --output <output_json>

Example:
    python3 diffusion_planner/util_scripts/generate_wds_list.py \
        processed_dataset/test_webdataset/ \
        --output train_wds.json
"""

import argparse
import json
from pathlib import Path

def parse_args():
    parser = argparse.ArgumentParser(description="Generate JSON list of WebDataset shards")
    parser.add_argument("target_dir", type=Path, help="Directory containing .tar.gz shards")
    parser.add_argument("--output", type=Path, required=True, help="Output JSON path")
    return parser.parse_args()

def main():
    args = parse_args()
    
    # Search for all .tar.gz files recursively
    shard_paths = sorted(list(args.target_dir.glob("**/*.tar.gz")))
    
    # Convert to absolute strings for the JSON
    shard_list = [str(p.resolve()) for p in shard_paths]
    
    print(f"Found {len(shard_list)} shards in {args.target_dir}")
    
    with open(args.output, 'w') as f:
        json.dump(shard_list, f, indent=4)
        
    print(f"Saved list to {args.output}")

if __name__ == "__main__":
    main()
