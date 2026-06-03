#!/usr/bin/env python3
"""
init_database.py
─────────────────────────────────────────────────────────────────────────────
Creates and seeds the warehouse inventory SQLite database.
Run once before starting the cognitive_amr stack:

    python3 scripts/init_database.py [--db /path/to/inventory.db]

Default path: ~/ros2_ws/src/cognitive_amr/config/inventory.db
"""

import sys
import os
import sqlite3
import argparse
import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from cognitive_amr.warehouse_constants import (
    PRODUCTS, DB_INITIAL_INVENTORY, SHELVES,
    get_product_world_pos, get_approach_pose
)

DEFAULT_DB = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'inventory.db'
)
DEFAULT_DB = os.path.normpath(DEFAULT_DB)


def create_schema(conn: sqlite3.Connection) -> None:
    conn.executescript("""
        CREATE TABLE IF NOT EXISTS products (
            product_id   TEXT PRIMARY KEY,
            product_name TEXT NOT NULL,
            aruco_id     INTEGER NOT NULL UNIQUE
        );

        CREATE TABLE IF NOT EXISTS inventory (
            product_id    TEXT REFERENCES products(product_id),
            shelf_id      TEXT NOT NULL,
            slot_index    INTEGER NOT NULL,
            expected_x    REAL NOT NULL,
            expected_y    REAL NOT NULL,
            expected_yaw  REAL NOT NULL,
            last_verified TEXT,
            confidence    TEXT NOT NULL DEFAULT 'unverified',
            PRIMARY KEY (product_id)
        );

        CREATE TABLE IF NOT EXISTS audit_log (
            event_id       INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp      TEXT NOT NULL,
            product_id     TEXT,
            shelf_scanned  TEXT,
            slot_scanned   INTEGER,
            aruco_detected INTEGER,
            outcome        TEXT NOT NULL,
            robot_pose_x   REAL,
            robot_pose_y   REAL,
            operator_action TEXT
        );
    """)


def seed_products(conn: sqlite3.Connection) -> None:
    for sku, info in PRODUCTS.items():
        conn.execute(
            "INSERT OR REPLACE INTO products VALUES (?, ?, ?)",
            (sku, info['name'], info['aruco_id'])
        )


def seed_inventory(conn: sqlite3.Connection) -> None:
    now = datetime.datetime.now(datetime.timezone.utc).isoformat()
    for sku, rec in DB_INITIAL_INVENTORY.items():
        shelf_id   = rec['shelf']
        slot_index = rec['slot']
        confidence = rec.get('confidence', 'unverified')
        ax, ay, ayaw = get_approach_pose(shelf_id, slot_index)
        conn.execute(
            """INSERT OR REPLACE INTO inventory
               (product_id, shelf_id, slot_index,
                expected_x, expected_y, expected_yaw,
                last_verified, confidence)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
            (sku, shelf_id, slot_index, ax, ay, ayaw, now, confidence)
        )


def main():
    parser = argparse.ArgumentParser(description='Initialise warehouse inventory DB')
    parser.add_argument('--db', default=DEFAULT_DB,
                        help=f'Path to SQLite file (default: {DEFAULT_DB})')
    parser.add_argument('--reset', action='store_true',
                        help='Drop existing tables and recreate from scratch')
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.db), exist_ok=True)

    conn = sqlite3.connect(args.db)
    conn.row_factory = sqlite3.Row

    if args.reset:
        conn.executescript("""
            DROP TABLE IF EXISTS audit_log;
            DROP TABLE IF EXISTS inventory;
            DROP TABLE IF EXISTS products;
        """)
        print("Existing tables dropped.")

    create_schema(conn)
    seed_products(conn)
    seed_inventory(conn)
    conn.commit()

    # Summary
    n_products  = conn.execute("SELECT COUNT(*) FROM products").fetchone()[0]
    n_inventory = conn.execute("SELECT COUNT(*) FROM inventory").fetchone()[0]
    print(f"Database ready: {args.db}")
    print(f"  products : {n_products}")
    print(f"  inventory: {n_inventory}")

    # Show the demo-critical record
    row = conn.execute(
        "SELECT * FROM inventory WHERE product_id='SKU-0042'"
    ).fetchone()
    if row:
        print(f"\nDemo record: SKU-0042 → shelf={row['shelf_id']} "
              f"slot={row['slot_index']} confidence={row['confidence']}")
        print("  (Physical location is B3-slot-2; DB says A3-slot-2 — cross-aisle mismatch demo ready)")

    conn.close()


if __name__ == '__main__':
    main()
