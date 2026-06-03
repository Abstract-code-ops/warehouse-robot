#!/usr/bin/env python3
"""
build_warehouse_sdf.py
─────────────────────────────────────────────────────────────────────────────
Generates the complete tugbot_style_ai_warehouse.sdf from Python so that
product models and ArUco textures are kept in sync with warehouse_constants.py.

Run from the repo root:
    python3 scripts/build_warehouse_sdf.py

Output: src/linorobot2/linorobot2_gazebo/worlds/tugbot_style_ai_warehouse.sdf
"""

import os
import sys
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from cognitive_amr.warehouse_constants import (
  SHELVES, PRODUCTS, SLOT_X_OFFSETS, PRODUCT_Z, get_approach_pose,
  LANDMARK_TAGS,
)

TEXTURE_BASE = (
    '/ros2_ws/src/linorobot2/linorobot2_gazebo/materials/textures'
)

# ── Coordinate conversion ─────────────────────────────────────────────────
# warehouse_constants uses MAP frame (robot spawn = map origin 0,0).
# The Gazebo world uses a different origin: map(0,0) = gz(-9.0, -7.5).
# All SDF poses must be in GZ frame, so convert before embedding.
MAP_OFFSET_X = 9.0
MAP_OFFSET_Y = 7.5

def to_gz(map_x: float, map_y: float) -> tuple:
    """Convert map-frame coordinates to Gazebo world frame."""
    return map_x - MAP_OFFSET_X, map_y - MAP_OFFSET_Y


OUT_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    '..', '..', '..', 'src',
    'linorobot2', 'linorobot2_gazebo',
    'worlds', 'tugbot_style_ai_warehouse.sdf'
))


# ── Helpers ──────────────────────────────────────────────────────────────

def aruco_texture_uri(aruco_id: int) -> str:
    return f'file://{TEXTURE_BASE}/aruco_{aruco_id:04d}.png'


def product_model(sku: str, info: dict) -> str:
    """Return SDF XML for a single product box with an ArUco face."""
    sx = info['placed_shelf']
    slot = info['placed_slot']
    s = SHELVES[sx]
    gz_cx, gz_cy = to_gz(s['cx'], s['cy'])
    wx = gz_cx + SLOT_X_OFFSETS[slot]
    # Offset 5 cm toward the aisle so product front face is flush with shelf face
    y_nudge = -0.05 if s['row'] == 'A' else 0.05
    wy = gz_cy + y_nudge
    wz = PRODUCT_Z
    aid = info['aruco_id']
    uri = aruco_texture_uri(aid)
    # face_y: 3 mm proud of product front face to avoid z-fighting with shelf front
    face_y = -0.228 if s['row'] == 'A' else 0.228

    return f"""
    <!-- Product {sku} (ArUco {aid}) at {sx}-slot{slot} -->
    <model name="product_{sku.replace('-', '_')}">
      <static>true</static>
      <pose>{wx:.3f} {wy:.3f} {wz:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="box_col">
          <geometry><box><size>0.65 0.45 0.45</size></box></geometry>
        </collision>
        <visual name="box_body">
          <geometry><box><size>0.65 0.45 0.45</size></box></geometry>
          <material><diffuse>0.55 0.32 0.16 1</diffuse></material>
        </visual>
        <visual name="aruco_face">
          <pose>0 {face_y:.4f} 0 0 0 0</pose>
          <geometry><box><size>0.28 0.001 0.28</size></box></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <pbr><metal>
              <albedo_map>{uri}</albedo_map>
              <metalness>0.0</metalness>
              <roughness>0.9</roughness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""


def shelf_row_band(shelf_name: str, row: str) -> str:
    """Return a small coloured strip at the shelf end-cap indicating the row."""
    colour = '0.15 0.45 0.90 1' if row == 'A' else '0.95 0.50 0.10 1'
    return f"""
        <visual name="row_band">
          <pose>-2.3 -0.26 0.35 0 0 0</pose>
          <geometry><box><size>0.04 0.05 1.2</size></box></geometry>
          <material><diffuse>{colour}</diffuse></material>
        </visual>"""


def aisle_entry_tile(shelf_id: str, colour: str) -> str:
    """Return SDF for a small coloured floor tile at the aisle scan start."""
    s = SHELVES[shelf_id]
    gz_cx, gz_cy = to_gz(s['cx'], s['cy'])
    sx = gz_cx - 2.5    # west end of shelf in GZ frame
    _, map_ay, _ = get_approach_pose(shelf_id)
    ay = map_ay - MAP_OFFSET_Y   # convert approach Y to GZ frame
    return f"""
    <model name="aisle_entry_{shelf_id}">
      <static>true</static>
      <pose>{sx:.2f} {ay:.3f} 0.008 0 0 0</pose>
      <link name="link">
        <visual name="tile">
          <geometry><box><size>0.4 0.4 0.01</size></box></geometry>
          <material><diffuse>{colour}</diffuse></material>
        </visual>
      </link>
    </model>"""


def landmark_tag_model(tag_id: int, info: dict) -> str:
    """Return a fixed landmark board with an ArUco texture in world coordinates."""
    # LANDMARK_TAGS are stored in MAP frame — convert to GZ world frame
    x, y = to_gz(float(info['x']), float(info['y']))
    z = float(info.get('z', 1.1))
    yaw = float(info.get('yaw', 0.0))
    uri = aruco_texture_uri(tag_id)
    return f"""
    <model name="landmark_tag_{tag_id}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>0.32 0.02 0.32</size></box></geometry>
        </collision>
        <visual name="board">
          <geometry><box><size>0.32 0.02 0.32</size></box></geometry>
          <material><diffuse>0.10 0.10 0.10 1</diffuse></material>
        </visual>
        <visual name="tag_face">
          <pose>0 -0.011 0 0 0 0</pose>
          <geometry><box><size>0.28 0.001 0.28</size></box></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <pbr><metal>
              <albedo_map>{uri}</albedo_map>
              <metalness>0.0</metalness>
              <roughness>0.9</roughness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""




# ── SDF assembly ─────────────────────────────────────────────────────────

def build_sdf() -> str:
    shelf_block = ''
    for sh, d in SHELVES.items():
        row = d['row']
        gz_cx, gz_cy = to_gz(d['cx'], d['cy'])
        row_colour  = '0.15 0.45 0.90 1' if row == 'A' else '0.95 0.50 0.10 1'
        steel       = '0.20 0.22 0.28 1'
        deck        = '0.32 0.32 0.35 1'
        # For row A the aisle is on the south face (local -Y); row B on the north (+Y).
        # front_y: local Y of the shelf face that faces the aisle
        front_y     = -0.275 if row == 'A' else 0.275
        label_y     = -0.295 if row == 'A' else 0.295   # 2 cm proud of front face

        # ── Open industrial pallet rack (Approach 1) ──────────────────────
        # Model origin at (gz_cx, gz_cy, 0.80).  All poses are relative to that.
        #
        #   Top rail   z=+0.77  (world 1.57 m)
        #   ──────────────────────────────────
        #   [open front — products visible here]
        #   Shelf deck z=-0.075 (world 0.725 m)  ← products sit on top (PRODUCT_Z=0.95)
        #   Floor base z=-0.77  (world 0.03 m)
        #   ──────────────────────────────────
        #   Back panel y=±0.245 (solid rear face)
        #   4 corner uprights at (±2.47, ±0.245)
        shelf_block += f"""
    <model name="storage_shelf_{sh}">
      <static>true</static>
      <pose>{gz_cx:.3f} {gz_cy:.3f} 0.80 0 0 0</pose>
      <link name="link">
        <!-- Full bounding-box collision (keeps Nav2 costmap correct) -->
        <collision name="collision">
          <geometry><box><size>5.0 0.55 1.6</size></box></geometry>
        </collision>

        <!-- ── Structural steel (posts, rails, back panel) ── -->
        <!-- Aisle-side posts (row A = south face y=-0.275; row B = north face y=+0.275) -->
        <visual name="post_W_aisle"><pose>-2.47 {front_y:.3f} 0 0 0 0</pose><geometry><box><size>0.06 0.06 1.6</size></box></geometry><material><diffuse>{steel}</diffuse></material></visual>
        <visual name="post_E_aisle"><pose> 2.47 {front_y:.3f} 0 0 0 0</pose><geometry><box><size>0.06 0.06 1.6</size></box></geometry><material><diffuse>{steel}</diffuse></material></visual>
        <!-- Rear-side posts -->
        <visual name="post_W_rear"> <pose>-2.47 {-front_y:.3f} 0 0 0 0</pose><geometry><box><size>0.06 0.06 1.6</size></box></geometry><material><diffuse>{steel}</diffuse></material></visual>
        <visual name="post_E_rear"> <pose> 2.47 {-front_y:.3f} 0 0 0 0</pose><geometry><box><size>0.06 0.06 1.6</size></box></geometry><material><diffuse>{steel}</diffuse></material></visual>
        <!-- Back panel (solid rear face, away from aisle, full height) -->
        <visual name="back_panel"><pose>0 {-front_y:.3f} 0 0 0 0</pose><geometry><box><size>4.94 0.04 1.6</size></box></geometry><material><diffuse>{steel}</diffuse></material></visual>
        <!-- Top rail -->
        <visual name="top_rail"><pose>0 0 0.77 0 0 0</pose><geometry><box><size>4.94 0.55 0.05</size></box></geometry><material><diffuse>{steel}</diffuse></material></visual>
        <!-- Floor base board -->
        <visual name="floor_board"><pose>0 0 -0.77 0 0 0</pose><geometry><box><size>4.94 0.55 0.04</size></box></geometry><material><diffuse>{deck}</diffuse></material></visual>
        <!-- Shelf deck — products rest on top of this (world Z = 0.725) -->
        <visual name="shelf_deck"><pose>0 0 -0.075 0 0 0</pose><geometry><box><size>4.94 0.55 0.04</size></box></geometry><material><diffuse>{deck}</diffuse></material></visual>

        <!-- ── Identification label (BELOW products, no overlap with ArUco) ── -->
        <visual name="front_label"><pose>0 {label_y:.3f} -0.60 0 0 0</pose><geometry><box><size>4.60 0.02 0.10</size></box></geometry><material><diffuse>0.95 0.70 0.18 1</diffuse></material></visual>
        <!-- Row-colour end band (small, at west end) -->
        <visual name="row_band"><pose>-2.47 {label_y:.3f} -0.60 0 0 0</pose><geometry><box><size>0.06 0.02 0.10</size></box></geometry><material><diffuse>{row_colour}</diffuse></material></visual>
      </link>
    </model>"""

    product_block = ''
    for sku, info in PRODUCTS.items():
        product_block += product_model(sku, info)

    landmark_block = ''
    for tag_id_str, info in LANDMARK_TAGS.items():
      landmark_block += landmark_tag_model(int(tag_id_str), info)

    aisle_tile_block = ''
    for sh, d in SHELVES.items():
        colour = '0.15 0.45 0.90 0.7' if d['row'] == 'A' else '0.95 0.50 0.10 0.7'
        aisle_tile_block += aisle_entry_tile(sh, colour)


    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="tugbot_style_ai_warehouse">

    <!-- Gazebo Sim systems -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <gravity>0 0 -9.81</gravity>
    <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    <atmosphere type="adiabatic"/>

    <scene>
      <ambient>0.72 0.72 0.70 1</ambient>
      <background>0.82 0.86 0.90 1</background>
      <shadows>true</shadows>
      <grid>true</grid>
    </scene>

    <light name="warehouse_key_light" type="directional">
      <pose>-8 -8 11 0 0.55 0.78</pose>
      <diffuse>1.0 0.98 0.95 1</diffuse>
      <specular>0.32 0.32 0.32 1</specular>
      <direction>0.45 0.35 -1</direction>
    </light>
    <light name="warehouse_fill_light" type="directional">
      <pose>8 8 10 0 -0.60 -0.80</pose>
      <diffuse>0.72 0.76 0.84 1</diffuse>
      <specular>0.08 0.08 0.08 1</specular>
      <direction>-0.45 -0.40 -1</direction>
    </light>
    <light name="warehouse_rim_light" type="directional">
      <pose>0 -10 9 0 0.45 0</pose>
      <diffuse>0.82 0.86 0.90 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>0 0.65 -1</direction>
    </light>

    <!-- FLOOR -->
    <model name="warehouse_floor">
      <static>true</static>
      <pose>0 0 -0.05 0 0 0</pose>
      <link name="floor_link">
        <collision name="collision">
          <geometry><box><size>24 18 0.1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>24 18 0.1</size></box></geometry>
          <material>
            <ambient>0.42 0.42 0.42 1</ambient>
            <diffuse>0.56 0.56 0.56 1</diffuse>
            <specular>0.08 0.08 0.08 1</specular>
          </material>
        </visual>
        <visual name="expansion_joint_1">
          <pose>0 -3.0 0.051 0 0 0</pose>
          <geometry><box><size>24.0 0.04 0.002</size></box></geometry>
          <material><diffuse>0.32 0.32 0.32 1</diffuse></material>
        </visual>
        <visual name="expansion_joint_2">
          <pose>0 3.0 0.051 0 0 0</pose>
          <geometry><box><size>24.0 0.04 0.002</size></box></geometry>
          <material><diffuse>0.32 0.32 0.32 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- OUTER WALLS -->
    <model name="outer_walls">
      <static>true</static>
      <link name="walls">
        <collision name="north_col"><pose>0 9 0.75 0 0 0</pose><geometry><box><size>24 0.2 1.5</size></box></geometry></collision>
        <visual name="north_vis"><pose>0 9 0.75 0 0 0</pose><geometry><box><size>24 0.2 1.5</size></box></geometry><material><diffuse>0.82 0.82 0.80 1</diffuse></material></visual>
        <collision name="south_left_col"><pose>-7 -9 0.75 0 0 0</pose><geometry><box><size>10 0.2 1.5</size></box></geometry></collision>
        <visual name="south_left_vis"><pose>-7 -9 0.75 0 0 0</pose><geometry><box><size>10 0.2 1.5</size></box></geometry><material><diffuse>0.82 0.82 0.80 1</diffuse></material></visual>
        <collision name="south_right_col"><pose>7 -9 0.75 0 0 0</pose><geometry><box><size>10 0.2 1.5</size></box></geometry></collision>
        <visual name="south_right_vis"><pose>7 -9 0.75 0 0 0</pose><geometry><box><size>10 0.2 1.5</size></box></geometry><material><diffuse>0.82 0.82 0.80 1</diffuse></material></visual>
        <collision name="west_col"><pose>-12 0 0.75 0 0 0</pose><geometry><box><size>0.2 18 1.5</size></box></geometry></collision>
        <visual name="west_vis"><pose>-12 0 0.75 0 0 0</pose><geometry><box><size>0.2 18 1.5</size></box></geometry><material><diffuse>0.82 0.82 0.80 1</diffuse></material></visual>
        <collision name="east_col"><pose>12 0 0.75 0 0 0</pose><geometry><box><size>0.2 18 1.5</size></box></geometry></collision>
        <visual name="east_vis"><pose>12 0 0.75 0 0 0</pose><geometry><box><size>0.2 18 1.5</size></box></geometry><material><diffuse>0.82 0.82 0.80 1</diffuse></material></visual>
      </link>
    </model>

    <!-- ═══ STORAGE SHELVES (rack frames only — products are separate models) ═══ -->
    {shelf_block}

    <!-- ═══ INDEPENDENT PRODUCT MODELS (ArUco-tagged) ═══ -->
    {product_block}

    <!-- ═══ FIXED LANDMARK TAG MODELS (for drift correction) ═══ -->
    {landmark_block}

    <!-- ═══ AISLE ENTRY FLOOR TILES (A=blue, B=orange) ═══ -->
    {aisle_tile_block}

    <!-- PICKING / PACKING STATIONS -->
    <model name="picking_station_1">
      <static>true</static>
      <pose>-9.0 5.7 0.45 0 0 0</pose>
      <link name="link">
        <collision name="table_col"><geometry><box><size>2.4 1.0 0.9</size></box></geometry></collision>
        <visual name="table_vis"><geometry><box><size>2.4 1.0 0.9</size></box></geometry><material><diffuse>0.10 0.42 0.62 1</diffuse></material></visual>
        <visual name="screen"><pose>0 0.55 0.55 0 0 0</pose><geometry><box><size>0.9 0.05 0.55</size></box></geometry><material><diffuse>0.02 0.02 0.02 1</diffuse></material></visual>
      </link>
    </model>

    <model name="picking_station_2">
      <static>true</static>
      <pose>-9.0 2.7 0.45 0 0 0</pose>
      <link name="link">
        <collision name="table_col"><geometry><box><size>2.4 1.0 0.9</size></box></geometry></collision>
        <visual name="table_vis"><geometry><box><size>2.4 1.0 0.9</size></box></geometry><material><diffuse>0.10 0.42 0.62 1</diffuse></material></visual>
        <visual name="screen"><pose>0 0.55 0.55 0 0 0</pose><geometry><box><size>0.9 0.05 0.55</size></box></geometry><material><diffuse>0.02 0.02 0.02 1</diffuse></material></visual>
      </link>
    </model>

    <!-- DISPATCH ZONE 1 (primary outbound) -->
    <model name="dispatch_zone_1">
      <static>true</static>
      <pose>8.6 -6.4 0.15 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>4.6 2.6 0.3</size></box></geometry></collision>
        <visual name="platform"><geometry><box><size>4.6 2.6 0.3</size></box></geometry><material><diffuse>0.20 0.50 0.22 1</diffuse></material></visual>
        <visual name="dock_line"><pose>0 1.34 0.08 0 0 0</pose><geometry><box><size>4.4 0.06 0.04</size></box></geometry><material><diffuse>1 1 1 1</diffuse></material></visual>
        <visual name="label_D1"><pose>0 0 0.22 0 0 0</pose><geometry><box><size>1.0 0.04 0.16</size></box></geometry><material><diffuse>0.20 0.80 0.20 1</diffuse></material></visual>
      </link>
    </model>

    <!-- DISPATCH ZONE PRIORITY (express / priority-flagged orders — red) -->
    <model name="dispatch_zone_priority">
      <static>true</static>
      <pose>8.6 5.9 0.15 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>1.6 1.1 0.3</size></box></geometry></collision>
        <visual name="platform"><geometry><box><size>1.6 1.1 0.3</size></box></geometry><material><diffuse>0.85 0.08 0.08 1</diffuse></material></visual>
        <visual name="priority_stripes"><pose>0 0 0.17 0 0 0</pose><geometry><box><size>1.4 0.12 0.04</size></box></geometry><material><diffuse>1 1 0 1</diffuse></material></visual>
        <visual name="label_PRI"><pose>0 0 0.25 0 0 0</pose><geometry><box><size>0.8 0.03 0.12</size></box></geometry><material><diffuse>1.0 1.0 1.0 1</diffuse></material></visual>
      </link>
    </model>

    <!-- CHARGING AREA -->
    <model name="charging_dock_1">
      <static>true</static>
      <pose>-9.3 -6.5 0.25 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>1.4 1.1 0.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.4 1.1 0.5</size></box></geometry><material><diffuse>0.95 0.75 0.05 1</diffuse></material></visual>
        <visual name="black_face"><pose>0 0.57 0 0 0 0</pose><geometry><box><size>1.1 0.04 0.32</size></box></geometry><material><diffuse>0.02 0.02 0.02 1</diffuse></material></visual>
      </link>
    </model>

    <model name="charging_dock_2">
      <static>true</static>
      <pose>-9.3 -4.8 0.25 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>1.4 1.1 0.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.4 1.1 0.5</size></box></geometry><material><diffuse>0.95 0.75 0.05 1</diffuse></material></visual>
        <visual name="black_face"><pose>0 0.57 0 0 0 0</pose><geometry><box><size>1.1 0.04 0.32</size></box></geometry><material><diffuse>0.02 0.02 0.02 1</diffuse></material></visual>
      </link>
    </model>

    <!-- FLOOR MARKINGS: lanes + zone indicators -->
    <model name="floor_markings">
      <static>true</static>
      <pose>0 0 0.012 0 0 0</pose>
      <link name="link">
        <visual name="main_lane"><pose>0 -7.5 0 0 0 0</pose><geometry><box><size>21.0 0.08 0.02</size></box></geometry><material><diffuse>1 0.9 0.05 1</diffuse></material></visual>
        <visual name="cross_lane_1"><pose>-6.4 0 0 0 0 0</pose><geometry><box><size>0.08 13.0 0.02</size></box></geometry><material><diffuse>1 0.9 0.05 1</diffuse></material></visual>
        <visual name="cross_lane_2"><pose>0 0 0 0 0 0</pose><geometry><box><size>0.08 13.0 0.02</size></box></geometry><material><diffuse>1 0.9 0.05 1</diffuse></material></visual>
        <visual name="cross_lane_3"><pose>6.4 0 0 0 0 0</pose><geometry><box><size>0.08 13.0 0.02</size></box></geometry><material><diffuse>1 0.9 0.05 1</diffuse></material></visual>
        <visual name="pickup_zone"><pose>-7.2 4.2 0 0 0 0</pose><geometry><box><size>2.0 4.4 0.02</size></box></geometry><material><diffuse>0.05 0.35 1 0.55</diffuse></material></visual>
        <visual name="dispatch_zone_1_marker"><pose>8.6 -6.4 0.02 0 0 0</pose><geometry><box><size>5.0 3.0 0.02</size></box></geometry><material><diffuse>0.05 1 0.18 0.45</diffuse></material></visual>
        <visual name="dispatch_priority_marker"><pose>8.6 5.9 0.02 0 0 0</pose><geometry><box><size>2.0 1.5 0.02</size></box></geometry><material><diffuse>1 0.10 0.10 0.55</diffuse></material></visual>
        <visual name="charge_zone_marker"><pose>-9.3 -5.65 0.02 0 0 0</pose><geometry><box><size>2.0 3.2 0.02</size></box></geometry><material><diffuse>1 0.85 0.05 0.45</diffuse></material></visual>
      </link>
    </model>

    <!-- SIMULATED AISLE BLOCKAGE — forces outer cross-lane routing, good for demo -->
    <model name="temporary_pallet_obstacle">
      <static>true</static>
      <pose>0 -2.5 0.35 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>1.2 0.8 0.7</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.2 0.8 0.7</size></box></geometry><material><diffuse>0.50 0.28 0.10 1</diffuse></material></visual>
        <visual name="hazard_stripe"><pose>0 0 0.38 0 0 0</pose><geometry><box><size>1.2 0.12 0.04</size></box></geometry><material><diffuse>1 0.80 0 1</diffuse></material></visual>
      </link>
    </model>

    <!--
      Robot spawn points:
        robot_1: -9.0 -7.5 0.1
      Task waypoints:
        Charging docks : (-9.3,-6.5), (-9.3,-4.8)
        Dispatch 1     : (8.6,-6.4)
        Dispatch 2     : (-7.2,-7.0)
        Dispatch Priority: (8.6,5.9)
    -->

  </world>
</sdf>
"""


def main():
    sdf = build_sdf()
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, 'w') as f:
        f.write(sdf)
    print(f"SDF written to: {OUT_PATH}")
    print(f"  Shelves   : {len(SHELVES)}")
    print(f"  Products  : {len(PRODUCTS)}")


if __name__ == '__main__':
    main()
