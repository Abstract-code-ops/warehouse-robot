#!/usr/bin/env python3
"""
generate_aruco_markers.py
─────────────────────────────────────────────────────────────────────────────
Generates ArUco marker PNG files for every product in the warehouse.
Run once before launching Gazebo:

    python3 scripts/generate_aruco_markers.py

Output directory: src/linorobot2/linorobot2_gazebo/materials/textures/
"""

import sys
import os

# Allow running from repo root or from scripts/ directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    import cv2.aruco as aruco
    import numpy as np
except ImportError:
    sys.exit("ERROR: opencv-contrib-python is required.\n"
             "  pip install opencv-contrib-python")

from cognitive_amr.warehouse_constants import PRODUCTS, LANDMARK_TAGS

TEXTURE_DIR = os.path.join(
    os.path.dirname(__file__),
    '..', '..', '..', 'src',
    'linorobot2', 'linorobot2_gazebo',
    'materials', 'textures'
)
TEXTURE_DIR = os.path.normpath(TEXTURE_DIR)

def main():
    os.makedirs(TEXTURE_DIR, exist_ok=True)

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    generated = []

    marker_jobs = []
    for sku, info in PRODUCTS.items():
        marker_jobs.append((f'PRODUCT {sku}', int(info['aruco_id'])))
    for tag_id in LANDMARK_TAGS.keys():
        marker_jobs.append((f'LANDMARK {tag_id}', int(tag_id)))

    for label, marker_id in marker_jobs:
        if marker_id > 49:
            print(f"  SKIP {label}: aruco_id={marker_id} exceeds DICT_4X4_50 range (0-49)")
            continue

        # 512×512 grey marker image → pad with white border → save as colour PNG
        img = aruco.generateImageMarker(dictionary, marker_id, 400)
        # Add 56-pixel white border to match a 512×512 output with clear margins
        img_bordered = cv2.copyMakeBorder(img, 56, 56, 56, 56,
                                          cv2.BORDER_CONSTANT, value=255)
        img_colour = cv2.cvtColor(img_bordered, cv2.COLOR_GRAY2BGR)

        filename = os.path.join(TEXTURE_DIR, f'aruco_{marker_id:04d}.png')
        cv2.imwrite(filename, img_colour)
        generated.append(filename)
        print(f"  [{label}] ArUco ID {marker_id:2d} -> {os.path.basename(filename)}")

    print(f"\nGenerated {len(generated)} marker images in:\n  {TEXTURE_DIR}")


if __name__ == '__main__':
    main()
