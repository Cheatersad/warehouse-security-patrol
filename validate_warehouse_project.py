#!/usr/bin/env python3
"""Offline consistency checks for the warehouse world, map and patrol route."""
from __future__ import annotations

from pathlib import Path
import py_compile
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image
import yaml


ROOT = Path(__file__).resolve().parent
PACKAGE = ROOT / 'src' / 'warehouse_patrol'
INFLATION_RADIUS_M = 0.70


def map_pixel(x: float, y: float, origin_x: float, origin_y: float, resolution: float, height: int) -> tuple[int, int]:
    col = int(round((x - origin_x) / resolution))
    row_from_bottom = int(round((y - origin_y) / resolution))
    return height - 1 - row_from_bottom, col


def main() -> None:
    errors: list[str] = []

    for xml_file in [
        PACKAGE / 'package.xml',
        ROOT / 'src' / 'security_perception' / 'package.xml',
        PACKAGE / 'worlds' / 'warehouse_zones.world',
    ]:
        try:
            ET.parse(xml_file)
        except Exception as exc:  # noqa: BLE001
            errors.append(f'XML error in {xml_file}: {exc}')

    for python_file in list((PACKAGE / 'warehouse_patrol').glob('*.py')) + list((PACKAGE / 'launch').glob('*.py')) + list((ROOT / 'src' / 'security_perception' / 'security_perception').glob('*.py')):
        try:
            py_compile.compile(str(python_file), doraise=True)
        except Exception as exc:  # noqa: BLE001
            errors.append(f'Python syntax error in {python_file}: {exc}')

    map_meta = yaml.safe_load((PACKAGE / 'maps' / 'warehouse_map.yaml').read_text())
    patrol = yaml.safe_load((PACKAGE / 'config' / 'patrol_waypoints.yaml').read_text())
    image = np.asarray(Image.open(PACKAGE / 'maps' / map_meta['image']).convert('L'))
    resolution = float(map_meta['resolution'])
    origin_x, origin_y, _ = [float(v) for v in map_meta['origin']]

    free_binary = np.where(image >= 250, 255, 0).astype(np.uint8)
    distance_pixels = cv2.distanceTransform(free_binary, cv2.DIST_L2, 5)
    safe = (distance_pixels * resolution >= INFLATION_RADIUS_M).astype(np.uint8)
    _, components = cv2.connectedComponents(safe, connectivity=8)

    component_ids: set[int] = set()
    waypoint_count = 0
    minimum_clearance = float('inf')
    for zone in patrol['zones']:
        for waypoint in zone['waypoints']:
            waypoint_count += 1
            row, col = map_pixel(
                float(waypoint['x']),
                float(waypoint['y']),
                origin_x,
                origin_y,
                resolution,
                image.shape[0],
            )
            if not (0 <= row < image.shape[0] and 0 <= col < image.shape[1]):
                errors.append(f'Waypoint outside map: {zone["name"]} {waypoint}')
                continue
            clearance = float(distance_pixels[row, col] * resolution)
            minimum_clearance = min(minimum_clearance, clearance)
            component = int(components[row, col])
            if component == 0:
                errors.append(
                    f'Waypoint has less than {INFLATION_RADIUS_M:.2f} m clearance: '
                    f'{zone["name"]} {waypoint}, clearance={clearance:.2f} m'
                )
            else:
                component_ids.add(component)

    if len(component_ids) != 1:
        errors.append(f'Patrol waypoints are split across components: {component_ids}')

    model_path = ROOT / 'yolov8n.pt'
    if not model_path.is_file() or model_path.stat().st_size < 1_000_000:
        errors.append(f'YOLO model missing or invalid: {model_path}')

    print('Warehouse project validation')
    print(f'  Map size: {image.shape[1]} x {image.shape[0]} pixels')
    print(f'  Resolution: {resolution:.2f} m/pixel')
    print(f'  Zones: {len(patrol["zones"])}')
    print(f'  Patrol waypoints: {waypoint_count}')
    print(f'  Minimum waypoint clearance: {minimum_clearance:.2f} m')
    print(f'  Safe-map connected components used: {sorted(component_ids)}')

    if errors:
        print('\nFAILED:')
        for error in errors:
            print(f'  - {error}')
        raise SystemExit(1)

    print('\nPASS: world, map, code, model and ordered patrol route are consistent.')


if __name__ == '__main__':
    main()
