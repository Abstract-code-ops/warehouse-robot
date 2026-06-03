"""
warehouse_constants.py
────────────────────────────────────────────────────────────────────────────
Single source of truth for all warehouse geometry.
Derived from tugbot_style_ai_warehouse.sdf actual shelf poses.
"""

import math

# ── Shelf geometry ───────────────────────────────────────────────────────
SHELF_HALF_DEPTH = 0.275   # 0.55m / 2
APPROACH_STANDOFF = 0.8    # metres from shelf face to robot camera
PICK_APPROACH_STANDOFF = 1.10  # extra room for turning/spin near shelves
PRODUCT_Z = 0.95           # shelf_center_z(0.80) + 0.15

# x offset from shelf centre for each slot
SLOT_X_OFFSETS = {1: -1.4, 2: 0.2, 3: 1.6}

# ── Shelf centre positions (from SDF) ────────────────────────────────────
    # map frame — spawn offset applied: map_x = gz_x + 9.0, map_y = gz_y + 7.5
SHELVES = {
    'A1': {'cx':  5.5, 'cy':  2.0, 'row': 'A'},
    'A2': {'cx':  5.5, 'cy':  4.5, 'row': 'A'},
    'A3': {'cx':  5.5, 'cy':  7.0, 'row': 'A'},
    'A4': {'cx':  5.5, 'cy':  9.5, 'row': 'A'},
    'A5': {'cx':  5.5, 'cy': 12.0, 'row': 'A'},
    'A6': {'cx':  5.5, 'cy': 14.5, 'row': 'A'},
    'B1': {'cx': 12.5, 'cy':  2.0, 'row': 'B'},
    'B2': {'cx': 12.5, 'cy':  4.5, 'row': 'B'},
    'B3': {'cx': 12.5, 'cy':  7.0, 'row': 'B'},
    'B4': {'cx': 12.5, 'cy':  9.5, 'row': 'B'},
    'B5': {'cx': 12.5, 'cy': 12.0, 'row': 'B'},
    'B6': {'cx': 12.5, 'cy': 14.5, 'row': 'B'},
}


def get_product_world_pos(shelf_id: str, slot: int) -> tuple:
    """Return (x, y, z) world position for a product box centre."""
    s = SHELVES[shelf_id]
    x = s['cx'] + SLOT_X_OFFSETS[slot]
    y = s['cy']
    return x, y, PRODUCT_Z


def get_approach_pose(shelf_id: str, slot: int | None = None,
                      standoff: float | None = None) -> tuple:
    """Return (x, y, yaw) approach pose for the robot to face a shelf.

    If slot is given the x is aligned with that slot.
    If slot is None the x is the shelf centre x (scan-start at x-2.5).
    """
    s = SHELVES[shelf_id]
    use_standoff = APPROACH_STANDOFF if standoff is None else float(standoff)

    if s['row'] == 'A':
        # robot approaches from south (-y), faces north (+y)
        ay = s['cy'] - SHELF_HALF_DEPTH - use_standoff
        yaw = math.pi / 2
    else:
        # B-row: robot approaches from north (+y), faces south (-y)
        ay = s['cy'] + SHELF_HALF_DEPTH + use_standoff
        yaw = -math.pi / 2
    ax = s['cx'] if slot is None else s['cx'] + SLOT_X_OFFSETS[slot]
    return ax, ay, yaw


def get_scan_start_pos(shelf_id: str) -> tuple:
    """Return (x, y, yaw) scan start: western end of aisle segment."""
    s = SHELVES[shelf_id]
    _, ay, yaw = get_approach_pose(shelf_id)
    sx = s['cx'] - 2.5   # 2.5m west of shelf centre = start of 5m shelf
    return sx, ay, yaw


# ── Dispatch zones ───────────────────────────────────────────────────────
# Bottom-left dual dispatch bays.
DISPATCH_ZONE_BL_1       = (1.7, 1.1)
DISPATCH_ZONE_BL_2       = (1.7, 2.8)
DISPATCH_ZONES_BOTTOM_LEFT = (DISPATCH_ZONE_BL_1, DISPATCH_ZONE_BL_2)

# Legacy aliases kept for compatibility with existing imports.
DISPATCH_ZONE_1          = DISPATCH_ZONE_BL_1
DISPATCH_ZONE_PRIORITY   = DISPATCH_ZONE_BL_2

# ── Fixed landmark tags for drift correction ─────────────────────────────
# Keys are ArUco IDs as strings to match ROS parameter dictionary behavior.
LANDMARK_TAGS = {
    '45': {'x':  -1.8, 'y':  -0.3, 'z': 1.1},
    '46': {'x':  -1.8, 'y':  15.3, 'z': 1.1},
    '47': {'x':  19.8, 'y':  -0.3, 'z': 1.1},
    '48': {'x':  19.8, 'y':  15.3, 'z': 1.1},
}

# ── Product catalogue ────────────────────────────────────────────────────
# aruco_id must be 0-49 (DICT_4X4_50)
# placed_shelf / placed_slot = PHYSICAL location in SDF
# For mismatch demos, some SKUs are intentionally placed in a different aisle
# than what DB_INITIAL_INVENTORY records.
PRODUCTS = {
    'SKU-0001': {'name': 'Drive Belt Set',       'aruco_id': 1,  'placed_shelf': 'A1', 'placed_slot': 1},
    'SKU-0002': {'name': 'Pneumatic Cylinder',    'aruco_id': 2,  'placed_shelf': 'A1', 'placed_slot': 2},
    'SKU-0003': {'name': 'Flow Control Valve',    'aruco_id': 3,  'placed_shelf': 'A1', 'placed_slot': 3},
    'SKU-0004': {'name': 'Ball Bearing 6205',     'aruco_id': 4,  'placed_shelf': 'B1', 'placed_slot': 1},
    'SKU-0005': {'name': 'Shaft Coupling 20mm',   'aruco_id': 5,  'placed_shelf': 'B1', 'placed_slot': 2},
    'SKU-0006': {'name': 'Timing Pulley T5',      'aruco_id': 6,  'placed_shelf': 'B1', 'placed_slot': 3},
    'SKU-0007': {'name': 'O-Ring Assortment',     'aruco_id': 7,  'placed_shelf': 'A2', 'placed_slot': 1},
    'SKU-0008': {'name': 'Limit Switch SPDT',     'aruco_id': 8,  'placed_shelf': 'A2', 'placed_slot': 2},
    'SKU-0009': {'name': 'Linear Guide Rail',     'aruco_id': 9,  'placed_shelf': 'A2', 'placed_slot': 3},
    # B2 slot-1: correctly placed SKU-0017
    'SKU-0017': {'name': 'Bearing Assembly',      'aruco_id': 17, 'placed_shelf': 'B2', 'placed_slot': 1},
    'SKU-0010': {'name': 'Proximity Sensor NPN',  'aruco_id': 10, 'placed_shelf': 'B2', 'placed_slot': 2},
    'SKU-0011': {'name': 'Solenoid Valve 24V',    'aruco_id': 11, 'placed_shelf': 'B2', 'placed_slot': 3},
    # A3: slot-1 has SKU-0012, slot-2 EMPTY (DB says SKU-0042), slot-3 has SKU-0014
    'SKU-0012': {'name': 'Encoder 1000PPR',       'aruco_id': 12, 'placed_shelf': 'A3', 'placed_slot': 1},
    'SKU-0042': {'name': 'Hydraulic Valve Set',   'aruco_id': 42, 'placed_shelf': 'B3', 'placed_slot': 2},
    'SKU-0013': {'name': 'Pressure Sensor',       'aruco_id': 13, 'placed_shelf': 'B3', 'placed_slot': 1},
    'SKU-0014': {'name': 'Motor Driver PCB',      'aruco_id': 14, 'placed_shelf': 'A3', 'placed_slot': 3},
    'SKU-0015': {'name': 'Cable Harness 3m',      'aruco_id': 15, 'placed_shelf': 'B3', 'placed_slot': 3},
    'SKU-0016': {'name': 'Servo Motor 24V',       'aruco_id': 16, 'placed_shelf': 'A4', 'placed_slot': 1},
    'SKU-0018': {'name': 'Power Supply 48V',      'aruco_id': 18, 'placed_shelf': 'A4', 'placed_slot': 2},
    'SKU-0019': {'name': 'Safety Relay Module',   'aruco_id': 19, 'placed_shelf': 'A4', 'placed_slot': 3},
    'SKU-0020': {'name': 'Stepper Driver DM542',  'aruco_id': 20, 'placed_shelf': 'B4', 'placed_slot': 1},
    'SKU-0021': {'name': 'HMI Panel 7inch',       'aruco_id': 21, 'placed_shelf': 'B4', 'placed_slot': 2},
    'SKU-0022': {'name': 'Ethernet Switch 8P',    'aruco_id': 22, 'placed_shelf': 'B4', 'placed_slot': 3},
    'SKU-0023': {'name': 'Pneumatic Tubing 6mm',  'aruco_id': 23, 'placed_shelf': 'A5', 'placed_slot': 1},
    'SKU-0024': {'name': 'Actuator Rod 300mm',    'aruco_id': 24, 'placed_shelf': 'A5', 'placed_slot': 2},
    'SKU-0025': {'name': 'Gripper Finger Set',    'aruco_id': 25, 'placed_shelf': 'A5', 'placed_slot': 3},
    'SKU-0026': {'name': 'Vision Camera 5MP',     'aruco_id': 26, 'placed_shelf': 'B5', 'placed_slot': 1},
    'SKU-0027': {'name': 'Force Sensor 50N',      'aruco_id': 27, 'placed_shelf': 'B5', 'placed_slot': 2},
    'SKU-0028': {'name': 'PLC Module CPU',        'aruco_id': 28, 'placed_shelf': 'B5', 'placed_slot': 3},
    'SKU-0029': {'name': 'Conveyor Belt 1m',      'aruco_id': 29, 'placed_shelf': 'A6', 'placed_slot': 1},
    'SKU-0030': {'name': 'Vacuum Cup 50mm',       'aruco_id': 30, 'placed_shelf': 'A6', 'placed_slot': 2},
    'SKU-0031': {'name': 'Laser Distance Sensor', 'aruco_id': 31, 'placed_shelf': 'A6', 'placed_slot': 3},
    'SKU-0032': {'name': 'Rotary Encoder 600P',   'aruco_id': 32, 'placed_shelf': 'B6', 'placed_slot': 1},
    'SKU-0033': {'name': 'Magnetic Clamp 80N',    'aruco_id': 33, 'placed_shelf': 'B6', 'placed_slot': 2},
    'SKU-0034': {'name': 'IO Terminal Block',     'aruco_id': 34, 'placed_shelf': 'B6', 'placed_slot': 3},
}

# DB initial records: what the WMS *believes* (SKU-0042 wrongly recorded at A3-slot-2)
DB_INITIAL_INVENTORY = {
    'SKU-0001': {'shelf': 'A1', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0002': {'shelf': 'A1', 'slot': 2, 'confidence': 'confirmed'},
    'SKU-0003': {'shelf': 'A1', 'slot': 3, 'confidence': 'confirmed'},
    'SKU-0004': {'shelf': 'B1', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0005': {'shelf': 'B1', 'slot': 2, 'confidence': 'confirmed'},
    'SKU-0006': {'shelf': 'B1', 'slot': 3, 'confidence': 'confirmed'},
    'SKU-0007': {'shelf': 'A2', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0008': {'shelf': 'A2', 'slot': 2, 'confidence': 'confirmed'},
    'SKU-0009': {'shelf': 'A2', 'slot': 3, 'confidence': 'confirmed'},
    'SKU-0017': {'shelf': 'B2', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0010': {'shelf': 'B2', 'slot': 2, 'confidence': 'confirmed'},
    'SKU-0011': {'shelf': 'B2', 'slot': 3, 'confidence': 'confirmed'},
    'SKU-0012': {'shelf': 'A3', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0042': {'shelf': 'A3', 'slot': 2, 'confidence': 'unverified'},  # ← WRONG SLOT
    'SKU-0013': {'shelf': 'B3', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0014': {'shelf': 'B3', 'slot': 2, 'confidence': 'confirmed'},
    'SKU-0015': {'shelf': 'B3', 'slot': 3, 'confidence': 'confirmed'},
    'SKU-0016': {'shelf': 'A4', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0018': {'shelf': 'A4', 'slot': 2, 'confidence': 'confirmed'},
    'SKU-0019': {'shelf': 'A4', 'slot': 3, 'confidence': 'confirmed'},
    'SKU-0020': {'shelf': 'B4', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0021': {'shelf': 'B4', 'slot': 2, 'confidence': 'confirmed'},
    'SKU-0022': {'shelf': 'B4', 'slot': 3, 'confidence': 'confirmed'},
    'SKU-0023': {'shelf': 'A5', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0024': {'shelf': 'A5', 'slot': 2, 'confidence': 'confirmed'},
    'SKU-0025': {'shelf': 'A5', 'slot': 3, 'confidence': 'confirmed'},
    'SKU-0026': {'shelf': 'B5', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0027': {'shelf': 'B5', 'slot': 2, 'confidence': 'confirmed'},
    'SKU-0028': {'shelf': 'B5', 'slot': 3, 'confidence': 'confirmed'},
    'SKU-0029': {'shelf': 'A6', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0030': {'shelf': 'A6', 'slot': 2, 'confidence': 'confirmed'},
    'SKU-0031': {'shelf': 'A6', 'slot': 3, 'confidence': 'confirmed'},
    'SKU-0032': {'shelf': 'B6', 'slot': 1, 'confidence': 'confirmed'},
    'SKU-0033': {'shelf': 'B6', 'slot': 2, 'confidence': 'confirmed'},
    'SKU-0034': {'shelf': 'B6', 'slot': 3, 'confidence': 'confirmed'},
}

# Quick reverse lookup: aruco_id → product_id
ARUCO_TO_SKU = {v['aruco_id']: k for k, v in PRODUCTS.items()}
