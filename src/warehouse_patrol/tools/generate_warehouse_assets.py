#!/usr/bin/env python3
"""Generate a matching Gazebo Classic warehouse world and Nav2 occupancy map.

The world and map are created from the same box definitions so the static map
matches the Gazebo collision geometry exactly.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import math

from PIL import Image, ImageDraw


@dataclass(frozen=True)
class Box:
    name: str
    x: float
    y: float
    sx: float
    sy: float
    height: float = 1.2
    color: str = "rack"


BOUNDS = (-15.0, 15.0, -11.0, 11.0)
MAP_MARGIN = 0.5
RESOLUTION = 0.05

COLORS = {
    "wall": (0.72, 0.74, 0.78, 1.0),
    "rack": (0.22, 0.32, 0.42, 1.0),
    "pallet": (0.52, 0.31, 0.15, 1.0),
    "table": (0.18, 0.50, 0.58, 1.0),
    "cage": (0.78, 0.18, 0.15, 1.0),
    "conveyor": (0.25, 0.25, 0.28, 1.0),
    "office": (0.55, 0.58, 0.62, 1.0),
    "column": (0.85, 0.72, 0.18, 1.0),
}

ZONE_TILES = [
    ("Zone_A_Receiving", -11.1, -5.6, 7.2, 10.0, (0.20, 0.48, 0.82, 0.24)),
    ("Zone_B_Central_Dispatch", -0.2, -4.0, 12.8, 13.0, (0.18, 0.68, 0.45, 0.20)),
    ("Zone_C_West_Storage", -7.3, 7.0, 14.4, 7.4, (0.74, 0.52, 0.12, 0.22)),
    ("Zone_D_East_Storage", 7.3, 7.0, 14.4, 7.4, (0.46, 0.32, 0.78, 0.22)),
    ("Zone_E_Restricted_Inventory", 11.25, 1.6, 5.7, 3.4, (0.90, 0.12, 0.12, 0.30)),
    ("Zone_F_Packing", 10.7, -5.7, 7.7, 9.6, (0.18, 0.66, 0.70, 0.22)),
]


def boxes() -> list[Box]:
    b: list[Box] = []
    # Outer boundary walls.
    b += [
        Box("wall_north", 0.0, 10.9, 30.0, 0.2, 2.2, "wall"),
        Box("wall_south", 0.0, -10.9, 30.0, 0.2, 2.2, "wall"),
        Box("wall_west", -14.9, 0.0, 0.2, 22.0, 2.2, "wall"),
        Box("wall_east", 14.9, 0.0, 0.2, 22.0, 2.2, "wall"),
    ]

    # High-bay racks. Their split layout creates a full-width cross aisle.
    rack_xs = [-12.0, -8.5, -5.0, -1.5, 2.0, 5.5, 9.0, 12.5]
    for idx, x in enumerate(rack_xs, 1):
        b.append(Box(f"rack_{idx}_south", x, 5.15, 0.9, 2.1, 1.8, "rack"))
        b.append(Box(f"rack_{idx}_north", x, 9.25, 0.9, 2.1, 1.8, "rack"))

    # Receiving pallets and inspection stations (south-west).
    for idx, (x, y) in enumerate([
        (-13.0, -8.8), (-10.5, -8.8),
        (-13.0, -5.7), (-10.5, -5.7),
    ], 1):
        b.append(Box(f"receiving_pallet_{idx}", x, y, 1.2, 1.0, 0.75, "pallet"))
    b += [
        Box("inspection_table_1", -12.5, -2.2, 1.7, 0.8, 0.9, "table"),
        Box("inspection_table_2", -9.8, -2.2, 1.7, 0.8, 0.9, "table"),
        Box("west_admin_block", -11.7, 1.15, 4.7, 2.1, 2.0, "office"),
    ]

    # Central dispatch conveyors and charging islands.
    b += [
        Box("conveyor_west", -3.2, -3.0, 3.8, 0.8, 0.85, "conveyor"),
        Box("conveyor_east", 3.2, -3.0, 3.8, 0.8, 0.85, "conveyor"),
        Box("charging_island_1", -4.4, -8.0, 1.1, 1.1, 0.55, "column"),
        Box("charging_island_2", 0.0, -8.0, 1.1, 1.1, 0.55, "column"),
        Box("charging_island_3", 4.4, -8.0, 1.1, 1.1, 0.55, "column"),
        Box("sorter_column_1", -5.6, 0.8, 0.8, 0.8, 1.4, "column"),
        Box("sorter_column_2", -2.1, 0.8, 0.8, 0.8, 1.4, "column"),
        Box("sorter_column_3", 1.4, 0.8, 0.8, 0.8, 1.4, "column"),
        Box("sorter_column_4", 4.9, 0.8, 0.8, 0.8, 1.4, "column"),
    ]

    # Packing tables and outbound pallets (south-east).
    for idx, (x, y) in enumerate([
        (8.4, -8.3), (11.3, -8.3), (13.5, -8.3),
        (8.4, -5.2), (11.3, -5.2), (13.5, -5.2),
    ], 1):
        b.append(Box(f"packing_table_{idx}", x, y, 1.25, 0.9, 0.9, "table"))
    b += [
        Box("outbound_pallet_1", 8.5, -2.4, 1.0, 1.0, 0.7, "pallet"),
        Box("outbound_pallet_2", 11.0, -2.4, 1.0, 1.0, 0.7, "pallet"),
        Box("outbound_pallet_3", 13.5, -2.4, 1.0, 1.0, 0.7, "pallet"),
    ]

    # Restricted inventory cage with a 2 m west-side entry.
    b += [
        Box("cage_north", 11.25, 3.4, 5.9, 0.16, 1.5, "cage"),
        Box("cage_south", 11.25, -0.2, 5.9, 0.16, 1.5, "cage"),
        Box("cage_east", 14.2, 1.6, 0.16, 3.6, 1.5, "cage"),
        Box("cage_west_lower", 8.3, 0.2, 0.16, 0.8, 1.5, "cage"),
        Box("cage_west_upper", 8.3, 3.0, 0.16, 0.8, 1.5, "cage"),
        Box("secure_cabinet_1", 10.25, 0.55, 1.0, 0.65, 1.25, "cage"),
        Box("secure_cabinet_2", 12.35, 2.65, 1.0, 0.65, 1.25, "cage"),
    ]

    return b


def sdf_box_model(box: Box) -> str:
    rgba = COLORS[box.color]
    z = box.height / 2.0
    return f"""
    <model name='{box.name}'>
      <static>true</static>
      <pose>{box.x:.3f} {box.y:.3f} {z:.3f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{box.sx:.3f} {box.sy:.3f} {box.height:.3f}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{box.sx:.3f} {box.sy:.3f} {box.height:.3f}</size></box></geometry>
          <material>
            <ambient>{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}</ambient>
            <diffuse>{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def sdf_tile(name: str, x: float, y: float, sx: float, sy: float, rgba: tuple[float, float, float, float]) -> str:
    return f"""
    <model name='{name}_floor'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.008 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><box><size>{sx:.3f} {sy:.3f} 0.012</size></box></geometry>
          <material>
            <ambient>{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}</ambient>
            <diffuse>{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}</diffuse>
          </material>
          <transparency>{1.0-rgba[3]:.3f}</transparency>
        </visual>
      </link>
    </model>"""


def sdf_zone_sign(name: str, x: float, y: float, rgba: tuple[float, float, float, float]) -> str:
    # A simple overhead coloured marker; labels are visible in RViz through zone markers.
    return f"""
    <model name='{name}_sign'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 2.35 0 0 0</pose>
      <link name='link'>
        <visual name='post'>
          <pose>0 0 -1.15 0 0 0</pose>
          <geometry><cylinder><radius>0.035</radius><length>2.3</length></cylinder></geometry>
          <material><diffuse>0.2 0.2 0.2 1</diffuse></material>
        </visual>
        <visual name='panel'>
          <geometry><box><size>1.5 0.12 0.55</size></box></geometry>
          <material><diffuse>{rgba[0]} {rgba[1]} {rgba[2]} 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def generate_world(path: Path, obstacle_boxes: Iterable[Box]) -> None:
    models = "\n".join(sdf_box_model(b) for b in obstacle_boxes)
    tiles = "\n".join(sdf_tile(*z) for z in ZONE_TILES)
    signs = "\n".join(
        sdf_zone_sign(z[0], z[1], min(10.1, z[2] + z[4] / 2.0 - 0.5), z[5])
        for z in ZONE_TILES
    )
    world = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <world name='warehouse_security_world'>
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <physics name='ode_physics' type='ode'>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <scene>
      <ambient>0.55 0.55 0.58 1</ambient>
      <background>0.82 0.86 0.90 1</background>
      <shadows>true</shadows>
    </scene>

    <gui fullscreen='0'>
      <camera name='warehouse_camera'>
        <pose>0 -27 27 0 0.68 1.57</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    <model name='warehouse_floor'>
      <static>true</static>
      <pose>0 0 0.002 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><box><size>30 22 0.01</size></box></geometry>
          <material><diffuse>0.70 0.70 0.68 1</diffuse></material>
        </visual>
      </link>
    </model>

{tiles}
{models}
{signs}

    <!-- A self-contained mannequin anomaly in the restricted cage. It has
         collision geometry, so LiDAR sees it even when YOLO does not classify
         the synthetic shape. Deterministic demo alerts remain disabled by default. -->
    <model name='restricted_zone_intruder'>
      <static>true</static>
      <pose>13.15 0.55 0 0 0 3.14159</pose>
      <link name='body'>
        <collision name='full_body_collision'>
          <pose>0 0 0.78 0 0 0</pose>
          <geometry><cylinder><radius>0.22</radius><length>1.50</length></cylinder></geometry>
        </collision>
        <visual name='torso_visual'>
          <pose>0 0 1.12 0 0 0</pose>
          <geometry><cylinder><radius>0.22</radius><length>0.72</length></cylinder></geometry>
          <material><diffuse>0.85 0.12 0.10 1</diffuse></material>
        </visual>
        <visual name='head_visual'>
          <pose>0 0 1.65 0 0 0</pose>
          <geometry><sphere><radius>0.18</radius></sphere></geometry>
          <material><diffuse>0.72 0.50 0.32 1</diffuse></material>
        </visual>
        <visual name='leg_left'>
          <pose>0 0.11 0.48 0 0 0</pose>
          <geometry><cylinder><radius>0.075</radius><length>0.85</length></cylinder></geometry>
          <material><diffuse>0.10 0.12 0.18 1</diffuse></material>
        </visual>
        <visual name='leg_right'>
          <pose>0 -0.11 0.48 0 0 0</pose>
          <geometry><cylinder><radius>0.075</radius><length>0.85</length></cylinder></geometry>
          <material><diffuse>0.10 0.12 0.18 1</diffuse></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""
    path.write_text(world)


def world_to_pixel(x: float, y: float, origin_x: float, origin_y: float, height: int) -> tuple[int, int]:
    px = int(round((x - origin_x) / RESOLUTION))
    py_from_bottom = int(round((y - origin_y) / RESOLUTION))
    py = height - 1 - py_from_bottom
    return px, py


def generate_map(pgm_path: Path, yaml_path: Path, obstacle_boxes: Iterable[Box]) -> None:
    min_x, max_x, min_y, max_y = BOUNDS
    origin_x = min_x - MAP_MARGIN
    origin_y = min_y - MAP_MARGIN
    width = int(round((max_x - min_x + 2 * MAP_MARGIN) / RESOLUTION))
    height = int(round((max_y - min_y + 2 * MAP_MARGIN) / RESOLUTION))
    image = Image.new("L", (width, height), 254)
    draw = ImageDraw.Draw(image)

    for b in obstacle_boxes:
        left = b.x - b.sx / 2.0
        right = b.x + b.sx / 2.0
        bottom = b.y - b.sy / 2.0
        top = b.y + b.sy / 2.0
        p1 = world_to_pixel(left, top, origin_x, origin_y, height)
        p2 = world_to_pixel(right, bottom, origin_x, origin_y, height)
        draw.rectangle([p1, p2], fill=0)

    image.save(pgm_path)
    yaml_path.write_text(
        f"image: {pgm_path.name}\n"
        "mode: trinary\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )


def main() -> None:
    base = Path(__file__).resolve().parents[1]
    world_path = base / "worlds" / "warehouse_zones.world"
    pgm_path = base / "maps" / "warehouse_map.pgm"
    yaml_path = base / "maps" / "warehouse_map.yaml"
    obstacle_boxes = boxes()
    generate_world(world_path, obstacle_boxes)
    generate_map(pgm_path, yaml_path, obstacle_boxes)
    print(f"Generated {world_path}")
    print(f"Generated {pgm_path}")
    print(f"Generated {yaml_path}")


if __name__ == "__main__":
    main()
