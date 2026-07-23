#!/usr/bin/env python3
"""Conservatively remove tiny isolated occupied components from a ROS map."""

import argparse
import json
import shutil
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import yaml


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-yaml", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--max-component-cells", type=int, default=3)
    parser.add_argument("--unknown-protection-radius", type=int, default=2)
    return parser.parse_args()


def main():
    args = parse_args()
    input_yaml = Path(args.input_yaml).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    metadata = yaml.safe_load(input_yaml.read_text())
    image_path = (input_yaml.parent / metadata["image"]).resolve()
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"cannot read map image: {image_path}")

    occupied = image <= 50
    unknown = (image > 50) & (image < 250)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        occupied.astype(np.uint8), connectivity=8
    )

    radius = max(0, args.unknown_protection_radius)
    if radius:
        kernel_size = 2 * radius + 1
        near_unknown = cv2.dilate(
            unknown.astype(np.uint8),
            np.ones((kernel_size, kernel_size), dtype=np.uint8),
        ).astype(bool)
    else:
        near_unknown = unknown

    cleaned = image.copy()
    removed_components = 0
    removed_cells = 0
    protected_components = 0
    protected_cells = 0
    removed_sizes = []

    for label in range(1, count):
        size = int(stats[label, cv2.CC_STAT_AREA])
        if size > args.max_component_cells:
            continue
        component = labels == label
        if np.any(near_unknown & component):
            protected_components += 1
            protected_cells += size
            continue
        cleaned[component] = 254
        removed_components += 1
        removed_cells += size
        removed_sizes.append(size)

    output_image = output_dir / "map.pgm"
    output_yaml = output_dir / "map.yaml"
    output_json = output_dir / "map.json"
    if not cv2.imwrite(str(output_image), cleaned):
        raise RuntimeError(f"cannot write map image: {output_image}")

    output_metadata = dict(metadata)
    output_metadata["image"] = "map.pgm"
    output_yaml.write_text(
        yaml.safe_dump(output_metadata, sort_keys=False, default_flow_style=None)
    )

    source_json = input_yaml.parent / "map.json"
    report = {
        "source_yaml": str(input_yaml),
        "source_image": str(image_path),
        "filter": {
            "max_component_cells": args.max_component_cells,
            "unknown_protection_radius_cells": radius,
            "connectivity": 8,
        },
        "result": {
            "occupied_cells_before": int(np.count_nonzero(occupied)),
            "occupied_cells_after": int(np.count_nonzero(cleaned <= 50)),
            "removed_components": removed_components,
            "removed_cells": removed_cells,
            "removed_component_sizes": removed_sizes,
            "protected_tiny_components_near_unknown": protected_components,
            "protected_tiny_cells_near_unknown": protected_cells,
        },
    }
    if source_json.exists():
        try:
            report["source_metadata"] = json.loads(source_json.read_text())
        except json.JSONDecodeError:
            shutil.copy2(source_json, output_dir / "source_map.json")
    output_json.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")

    print(json.dumps(report["result"], ensure_ascii=False))


if __name__ == "__main__":
    main()
