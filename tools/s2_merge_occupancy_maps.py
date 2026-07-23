#!/usr/bin/env python3
"""Merge already aligned ROS trinary occupancy maps without losing obstacles."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_map(yaml_path: Path):
    metadata = yaml.safe_load(yaml_path.read_text())
    image_path = yaml_path.parent / metadata["image"]
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Cannot read map image: {image_path}")
    return image, metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("maps", nargs="+", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--output-prefix", default="map")
    parser.add_argument(
        "--second-transform",
        nargs=3,
        type=float,
        metavar=("DEGREES", "DX_M", "DY_M"),
        help="Rigid world transform applied to the second map.",
    )
    args = parser.parse_args()
    loaded = [load_map(path.resolve()) for path in args.maps]
    resolutions = [float(meta["resolution"]) for _, meta in loaded]
    resolution = resolutions[0]
    if any(abs(value - resolution) > 1e-6 for value in resolutions[1:]):
        raise RuntimeError(f"Map resolutions differ: {resolutions}")
    if any(abs(float(meta["origin"][2])) > 1e-6 for _, meta in loaded):
        raise RuntimeError("Non-zero map origin yaw requires image resampling")

    transforms = [(0.0, 0.0, 0.0) for _ in loaded]
    if args.second_transform:
        if len(loaded) != 2:
            raise RuntimeError("--second-transform requires exactly two maps")
        transforms[1] = (
            math.radians(args.second_transform[0]),
            args.second_transform[1],
            args.second_transform[2],
        )

    bounds = []
    for (image, meta), (theta, dx, dy) in zip(loaded, transforms):
        origin_x, origin_y = map(float, meta["origin"][:2])
        corners = np.asarray(
            [
                [origin_x, origin_y],
                [origin_x + image.shape[1] * resolution, origin_y],
                [origin_x, origin_y + image.shape[0] * resolution],
                [origin_x + image.shape[1] * resolution, origin_y + image.shape[0] * resolution],
            ]
        )
        cosine, sine = math.cos(theta), math.sin(theta)
        rotation = np.asarray([[cosine, sine], [-sine, cosine]])
        transformed = corners @ rotation + np.asarray([dx, dy])
        bounds.append(transformed)
    all_corners = np.vstack(bounds)
    min_x, min_y = all_corners.min(axis=0)
    max_x, max_y = all_corners.max(axis=0)
    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))
    merged = np.full((height, width), 205, dtype=np.uint8)

    for (image, meta), (theta, dx, dy) in zip(loaded, transforms):
        origin_x, origin_y = map(float, meta["origin"][:2])

        def source_pixel_to_output(u: float, v: float) -> tuple[float, float]:
            x = origin_x + (u + 0.5) * resolution
            y = origin_y + (image.shape[0] - v - 0.5) * resolution
            cosine, sine = math.cos(theta), math.sin(theta)
            transformed_x = cosine * x - sine * y + dx
            transformed_y = sine * x + cosine * y + dy
            output_u = (transformed_x - min_x) / resolution - 0.5
            output_v = height - (transformed_y - min_y) / resolution - 0.5
            return output_u, output_v

        source_triangle = np.float32([[0, 0], [1, 0], [0, 1]])
        destination_triangle = np.float32(
            [source_pixel_to_output(*point) for point in source_triangle]
        )
        affine = cv2.getAffineTransform(source_triangle, destination_triangle)
        warped = cv2.warpAffine(
            image,
            affine,
            (width, height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=205,
        )
        free = warped >= 250
        occupied = warped <= 50
        merged[free & (merged == 205)] = 254
        # Conservative conflict policy: occupied always wins over free.
        merged[occupied] = 0

    args.output_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = args.output_dir / f"{args.output_prefix}.pgm"
    yaml_path = args.output_dir / f"{args.output_prefix}.yaml"
    json_path = args.output_dir / f"{args.output_prefix}.json"
    if not cv2.imwrite(str(pgm_path), merged):
        raise RuntimeError(f"Failed to write {pgm_path}")
    output_yaml = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": resolution,
        "origin": [float(min_x), float(min_y), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    yaml_path.write_text(yaml.safe_dump(output_yaml, sort_keys=False))
    stats = {
        "ok": True,
        "source_maps": [str(path.resolve()) for path in args.maps],
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "origin": output_yaml["origin"],
        "occupied_cells": int(np.count_nonzero(merged == 0)),
        "free_cells": int(np.count_nonzero(merged == 254)),
        "unknown_cells": int(np.count_nonzero(merged == 205)),
        "conflict_policy": "occupied_wins",
        "map_transforms": [
            {
                "rotation_deg": math.degrees(theta),
                "translation_m": [dx, dy],
            }
            for theta, dx, dy in transforms
        ],
    }
    json_path.write_text(json.dumps(stats, indent=2) + "\n")
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    main()
