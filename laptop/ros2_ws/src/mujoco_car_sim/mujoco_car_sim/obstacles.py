"""Procedurally generated box obstacles, added to a MuJoCo model through MjSpec.

Placement is a cluster process:
  * `cluster_count` cluster centres are scattered uniformly over the arena.
  * Each obstacle picks a random centre and lands at a Gaussian offset of
    std-dev `cluster_spread` (metres) around it.
  * `cluster_count = 0` turns clustering off: obstacles are placed uniformly.
Fewer clusters and a smaller spread give tight, dense groups; more clusters or a
larger spread approach a uniform scatter. `count` controls how many there are.
"""
from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass
class ObstacleConfig:
    count: int = 60                  # how many obstacles
    cluster_count: int = 4           # 0 = uniform placement, no clustering
    cluster_spread: float = 0.6      # std-dev (m) of obstacles around a cluster centre
    arena_half_size: float = 6.0     # floor is a square of side 2 * this (m)
    min_size: float = 0.05           # half-width range of each box (m)
    max_size: float = 0.25
    min_height: float = 0.05         # half-height range (m)
    max_height: float = 0.30
    spawn_clearance: float = 1.0     # keep this radius (m) around the car's start empty
    min_gap: float = 0.0             # extra spacing between obstacles (m); 0 allows overlap
    seed: int = -1                   # -1 = new layout every launch


def generate_layout(cfg: ObstacleConfig, spawn_xy=(0.0, 0.0)):
    """Return (seed_used, list of (x, y, yaw, half_x, half_y, half_z))."""
    seed = int(cfg.seed) if cfg.seed >= 0 else int(np.random.SeedSequence().entropy % 2**31)
    rng = np.random.default_rng(seed)
    limit = cfg.arena_half_size - cfg.max_size  # keep boxes fully on the floor
    spawn = np.asarray(spawn_xy, dtype=float)

    centres = rng.uniform(-limit, limit, size=(cfg.cluster_count, 2)) if cfg.cluster_count > 0 else None

    boxes, placed_xy, placed_r = [], [], []
    attempts = 0
    while len(boxes) < cfg.count and attempts < cfg.count * 200:
        attempts += 1
        if centres is not None:
            xy = centres[rng.integers(len(centres))] + rng.normal(0.0, cfg.cluster_spread, 2)
        else:
            xy = rng.uniform(-limit, limit, 2)
        if np.any(np.abs(xy) > limit):
            continue

        hx, hy = rng.uniform(cfg.min_size, cfg.max_size, 2)
        hz = rng.uniform(cfg.min_height, cfg.max_height)
        radius = float(np.hypot(hx, hy))  # bounding circle of the box
        if np.linalg.norm(xy - spawn) < cfg.spawn_clearance + radius:
            continue
        if cfg.min_gap > 0 and placed_xy:
            d = np.linalg.norm(np.asarray(placed_xy) - xy, axis=1)
            if np.any(d < np.asarray(placed_r) + radius + cfg.min_gap):
                continue

        yaw = rng.uniform(0.0, np.pi)
        boxes.append((float(xy[0]), float(xy[1]), float(yaw), float(hx), float(hy), float(hz)))
        placed_xy.append(xy)
        placed_r.append(radius)
    return seed, boxes


def build_model(model_path, cfg: ObstacleConfig, car_body='chassis'):
    """Load the MJCF, resize the floor, add obstacles and compile. Returns (model, seed, n)."""
    spec = mujoco.MjSpec.from_file(model_path)

    for geom in spec.worldbody.geoms:  # resize the floor plane to match the arena
        if geom.type == mujoco.mjtGeom.mjGEOM_PLANE:
            geom.size = [cfg.arena_half_size, cfg.arena_half_size, geom.size[2]]

    car = spec.body(car_body)
    spawn_xy = (car.pos[0], car.pos[1]) if car is not None else (0.0, 0.0)
    seed, boxes = generate_layout(cfg, spawn_xy)

    for i, (x, y, yaw, hx, hy, hz) in enumerate(boxes):
        g = spec.worldbody.add_geom()  # attached to the world, so obstacles are static
        g.name = f'obstacle_{i}'
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = [hx, hy, hz]
        g.pos = [x, y, hz]  # rest on the floor
        g.quat = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        g.rgba = [0.85, 0.45, 0.2, 1.0]
    return spec.compile(), seed, len(boxes)
