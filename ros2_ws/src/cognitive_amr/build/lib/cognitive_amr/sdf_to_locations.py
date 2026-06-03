"""
sdf_to_locations.py
────────────────────────────────────────────────────────────────────────────
Utility: parse a Gazebo SDF world file and convert every named model's pose
into Nav2 map-frame coordinates, then print a ready-to-paste locations.yaml.

Coordinate transform
────────────────────
  SLAM was started with the robot at Gazebo position (SPAWN_X, SPAWN_Y).
  SLAM always assigns that physical point as (0, 0) in the map frame.

  Therefore:
      map_x = gazebo_x - SPAWN_X
      map_y = gazebo_y - SPAWN_Y

  Adjust SPAWN_X / SPAWN_Y below if you ever re-run SLAM from a different
  spawn position.

Usage
─────
  python3 sdf_to_locations.py
"""

import xml.etree.ElementTree as ET

# ── Configure these two values ────────────────────────────────────────────
SDF_PATH   = '/ros2_ws/src/cognitive_amr_gazebo/worlds/tugbot_style_ai_warehouse.sdf'

# Gazebo spawn position used when SLAM was recorded.
# SLAM assigns this Gazebo point as map-frame (0, 0).
SPAWN_X = -9.0
SPAWN_Y = -7.5

# Models to skip (infrastructure, not navigation targets)
SKIP_MODELS = {
    'warehouse_floor',
    'outer_walls',
    'floor_markings',
    'sun',
}
# ──────────────────────────────────────────────────────────────────────────


def gazebo_to_map(gz_x: float, gz_y: float):
    """Convert Gazebo world coordinates to Nav2 map-frame coordinates."""
    return round(gz_x - SPAWN_X, 3), round(gz_y - SPAWN_Y, 3)


def parse_sdf(path: str):
    """Return list of (model_name, gz_x, gz_y) for every <model> in the SDF."""
    tree = ET.parse(path)
    root = tree.getroot()

    # SDF root can be <sdf> → <world> → <model>, or just <world> → <model>
    world = root.find('world') or root
    results = []

    for model in world.findall('model'):
        name = model.get('name', 'unnamed')
        if name in SKIP_MODELS:
            continue

        pose_el = model.find('pose')
        if pose_el is None or not pose_el.text:
            continue

        # SDF pose format: "x y z roll pitch yaw"
        parts = pose_el.text.strip().split()
        gz_x, gz_y = float(parts[0]), float(parts[1])
        results.append((name, gz_x, gz_y))

    return results


def suggest_orientation(name: str):
    """
    Heuristic: suggest a facing direction based on the model name.
    Shelves face the aisle (toward x=0), picking/charging stations face right.
    Returns (z_orient, w_orient).
    """
    n = name.lower()
    if 'shelf_a' in n:
        return 0.707, 0.707   # face up (+Y), approaching from south aisle
    if 'shelf_b' in n:
        return -0.707, 0.707  # face down (-Y), approaching from north aisle
    if 'charging' in n:
        return 0.0, 1.0       # face right (+X) toward the charging dock
    if 'picking' in n:
        return 0.0, 1.0       # face right
    if 'dispatch' in n:
        return 1.0, 0.0       # face left (-X)
    if 'pallet' in n or 'bin' in n:
        return 0.0, 1.0
    return 0.0, 1.0           # default: face right


def main():
    models = parse_sdf(SDF_PATH)

    print("# ── Auto-generated locations ─────────────────────────────────────")
    print(f"# Source SDF : {SDF_PATH}")
    print(f"# SLAM spawn : Gazebo ({SPAWN_X}, {SPAWN_Y})  →  map (0.0, 0.0)")
    print("#")
    print("# IMPORTANT: The robot navigates to the AISLE IN FRONT of each shelf,")
    print("# not into the shelf itself. Adjust x/y offsets as needed after testing.")
    print()
    print("metadata:")
    print('  frame_id: "map"')
    print('  description: "Tugbot AI Warehouse — auto-generated from SDF"')
    print()
    print("locations:")

    for name, gz_x, gz_y in models:
        map_x, map_y = gazebo_to_map(gz_x, gz_y)
        z_o, w_o = suggest_orientation(name)
        print()
        print(f"  {name}:")
        print(f"    x: {map_x}")
        print(f"    y: {map_y}")
        print(f"    z_orient: {z_o}")
        print(f"    w_orient: {w_o}")
        print(f"    description: \"Gazebo pose: ({gz_x}, {gz_y})\"")


if __name__ == '__main__':
    main()
