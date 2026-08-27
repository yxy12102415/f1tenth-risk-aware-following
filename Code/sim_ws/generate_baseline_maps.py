#!/usr/bin/env python3
"""Generate deterministic static-obstacle maps from Melbourne_map."""

import json
import math
from pathlib import Path

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parent
MAPS = ROOT / 'src' / 'f1tenth_gym_ros' / 'maps'
BASE_NAME = 'Melbourne_map'
RESOLUTION = 0.09009
ORIGIN = (-81.75675943118459, -49.98378618463543)
OBSTACLE_RADIUS_M = 0.32

# A point on the initial straight, far enough ahead to make trial startup repeatable.
CENTER = (-18.40, 17.38)
TRACK_HEADING = -0.785
LATERAL_OFFSET_M = 0.70


def world_to_pixel(x: float, y: float, height: int) -> tuple[int, int]:
    px = round((x - ORIGIN[0]) / RESOLUTION)
    py = height - 1 - round((y - ORIGIN[1]) / RESOLUTION)
    return px, py


def main() -> None:
    source_image = MAPS / f'{BASE_NAME}.png'
    source_yaml = MAPS / f'{BASE_NAME}.yaml'
    yaml_template = source_yaml.read_text()
    radius_px = max(1, round(OBSTACLE_RADIUS_M / RESOLUTION))
    left_normal = (-math.sin(TRACK_HEADING), math.cos(TRACK_HEADING))
    centers = {
        'center': CENTER,
        'left': (
            CENTER[0] + LATERAL_OFFSET_M * left_normal[0],
            CENTER[1] + LATERAL_OFFSET_M * left_normal[1],
        ),
        'right': (
            CENTER[0] - LATERAL_OFFSET_M * left_normal[0],
            CENTER[1] - LATERAL_OFFSET_M * left_normal[1],
        ),
    }

    manifest = {
        'source_map': BASE_NAME,
        'obstacle_radius_m': OBSTACLE_RADIUS_M,
        'lateral_offset_m': LATERAL_OFFSET_M,
        'scenarios': {},
    }
    for position, (x, y) in centers.items():
        name = f'{BASE_NAME}_obstacle_{position}'
        image = Image.open(source_image).convert('L')
        draw = ImageDraw.Draw(image)
        px, py = world_to_pixel(x, y, image.height)
        draw.ellipse(
            (px - radius_px, py - radius_px, px + radius_px, py + radius_px),
            fill=0,
        )
        image.save(MAPS / f'{name}.png')
        (MAPS / f'{name}.yaml').write_text(
            yaml_template.replace(f'image: {BASE_NAME}.png', f'image: {name}.png')
        )
        manifest['scenarios'][position] = {
            'map_name': name,
            'center_world_m': [x, y],
            'center_pixel': [px, py],
        }

    manifest_path = MAPS / 'baseline_obstacles.json'
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    print(f'Generated 3 maps and {manifest_path}')


if __name__ == '__main__':
    main()
