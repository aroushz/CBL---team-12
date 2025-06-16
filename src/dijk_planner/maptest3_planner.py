#!/usr/bin/env python3
# maptest3_planner.py
#
# A minimal Dijkstra path‑planner for the map stored in
#   <repo root>/maps/maptest3.yaml / .pgm
#
# Usage (inside the ros2‑dev container):
#   python3 maptest3_planner.py  x0  y0  x1  y1

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

# ───────────────────────── locate repo & map dir ──────────────────────────
HERE       = Path(__file__).resolve()
REPO_ROOT  = next(p for p in HERE.parents if (p / "maps").is_dir())
MAP_DIR    = REPO_ROOT / "maps"                 # …/team12/maps
CACHE_FILE = REPO_ROOT / ".planner_cache.pkl"   # shared cache

META_YAML  = MAP_DIR / "maptest3.yaml"
SCALE      = 4                                  # px per grid cell

# ───────────────────────── load map & build graph ─────────────────────────
meta = yaml.safe_load(META_YAML.read_text())
img_path = MAP_DIR / Path(meta["image"]).name    # image may be abs. in YAML
img      = Image.open(img_path).convert("L")

grid = (np.asarray(img) >
        meta.get("occupied_thresh", .65) * 255).astype(np.uint8)
H, W = grid.shape

graph = {}
for y in range(H):
    for x in range(W):
        if not grid[y, x]:
            continue
        nbrs = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and grid[ny, nx]:
                nbrs.append(((nx, ny), 1))
        graph[(x, y)] = nbrs

# ──────────────────────────── tiny cache layer ────────────────────────────
try:
    memo = pickle.loads(CACHE_FILE.read_bytes())
except FileNotFoundError:
    memo = {}

def save_cache() -> None:
    CACHE_FILE.write_bytes(pickle.dumps(memo))

# ───────────────────────────── Dijkstra import ────────────────────────────
# re‑use your existing implementation
from dijk import dijk

def shortest(start, goal):
    key = (start, goal)
    if key not in memo:
        res = dijk(start, graph)[goal]
        memo[key] = res["Distance"], res["Path"]
        save_cache()
    return memo[key]

# ────────────────────────────── rendering help ────────────────────────────
def draw_path(path, start, goal) -> Path:
    img = Image.new("RGB", (W * SCALE, H * SCALE), "white")
    drw = ImageDraw.Draw(img)

    # obstacles
    ys, xs = np.where(grid == 0)
    for y, x in zip(ys, xs):
        drw.rectangle([(x*SCALE, y*SCALE),
                       ((x+1)*SCALE-1, (y+1)*SCALE-1)], fill="black")

    # the path
    for (x0, y0), (x1, y1) in path:
        drw.line([(x0*SCALE+SCALE//2, y0*SCALE+SCALE//2),
                  (x1*SCALE+SCALE//2, y1*SCALE+SCALE//2)],
                 fill="red", width=SCALE//2)

    # start / goal markers
    for (x, y), colour in ((start, "green"), (goal, "blue")):
        drw.ellipse([(x*SCALE+1, y*SCALE+1),
                     ((x+1)*SCALE-2, (y+1)*SCALE-2)],
                    fill=colour)

    fn = REPO_ROOT / f"maptest3_{start[0]}_{start[1]}_{goal[0]}_{goal[1]}.png"
    img.save(fn)
    return fn

# ─────────────────────────────── CLI entry ────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("x0", type=int)
    ap.add_argument("y0", type=int)
    ap.add_argument("x1", type=int)
    ap.add_argument("y1", type=int)
    a  = ap.parse_args()

    start, goal = (a.x0, a.y0), (a.x1, a.y1)
    if start not in graph or goal not in graph:
        sys.exit("start or goal lies on an obstacle ‑ pick free cells")

    dist, path = shortest(start, goal)
    png = draw_path(path, start, goal)
    print(f"distance {dist}  steps {len(path)}  -> {png.name}")

if __name__ == "__main__":
    main()
