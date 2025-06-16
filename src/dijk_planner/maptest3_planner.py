#!/usr/bin/env python3
#
# Quick path‑planner demo for the maptest3 map.
# Call it in a docker shell like:
#   python3 maptest3_planner.py
# …then enter start / goal grid‑coordinates in the prompt.

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

# -----------------------------------------------------------------------------
# files & constants
# -----------------------------------------------------------------------------
HERE   = Path(__file__).parent
MAPDIR = HERE / "maps"                    # maptest3.{yaml,pgm} live here
CACHE  = HERE / "planner_cache.pkl"       # memoised dijkstra results
SCALE  = 4                                # pixels per cell in render

yaml_meta = yaml.safe_load((MAPDIR / "maptest3.yaml").read_text())
img_path = Path(meta["image"])
if not img_path.is_absolute():
    img_path = MAP_DIR / img_path
img = Image.open(img_path).convert("L")

occ_thr = yaml_meta.get("occupied_thresh", .65) * 255
grid    = (np.asarray(pgm_img) > occ_thr).astype(np.uint8)   # 1 = free
H, W    = grid.shape

# -----------------------------------------------------------------------------
# build 4‑connected graph of free cells
# -----------------------------------------------------------------------------
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

# -----------------------------------------------------------------------------
# small dijkstra helper
# -----------------------------------------------------------------------------
def dijkstra(src, g):
    dist = {v: float("inf") for v in g}
    path = {v: [] for v in g}
    dist[src] = 0

    to_visit = set(g)
    while to_visit:
        cur = min(to_visit, key=lambda v: dist[v])
        to_visit.remove(cur)

        for nxt, w in g[cur]:
            alt = dist[cur] + w
            if alt < dist[nxt]:
                dist[nxt] = alt
                path[nxt] = path[cur] + [(cur, nxt)]
    return {v: {"dist": dist[v], "path": path[v]} for v in g}

# -----------------------------------------------------------------------------
# memoised solve()
# -----------------------------------------------------------------------------
try:
    memo = pickle.loads(CACHE.read_bytes())
except FileNotFoundError:
    memo = {}


def solve(a, b):
    key = (a, b)
    if key not in memo:
        memo[key] = dijkstra(a, graph)[b]["dist"], dijkstra(a, graph)[b]["path"]
        CACHE.write_bytes(pickle.dumps(memo))
    return memo[key]


# -----------------------------------------------------------------------------
# pretty render
# -----------------------------------------------------------------------------
def render(path_edges, start, goal):
    out = Image.new("RGB", (W * SCALE, H * SCALE), "white")
    drw = ImageDraw.Draw(out)

    # draw walls
    ys, xs = np.where(grid == 0)
    for y, x in zip(ys, xs):
        drw.rectangle(
            [(x * SCALE, y * SCALE), ((x + 1) * SCALE - 1, (y + 1) * SCALE - 1)],
            fill="black",
        )

    # draw the path
    for (x0, y0), (x1, y1) in path_edges:
        drw.line(
            [
                (x0 * SCALE + SCALE // 2, y0 * SCALE + SCALE // 2),
                (x1 * SCALE + SCALE // 2, y1 * SCALE + SCALE // 2),
            ],
            fill="red",
            width=SCALE // 2,
        )

    # start / goal dots
    for (x, y), col in ((start, "green"), (goal, "blue")):
        drw.ellipse(
            [(x * SCALE + 1, y * SCALE + 1), ((x + 1) * SCALE - 2, (y + 1) * SCALE - 2)],
            fill=col,
        )

    name = f"maptest3_{start[0]}_{start[1]}_{goal[0]}_{goal[1]}.png"
    out.save(name)
    return name


# -----------------------------------------------------------------------------
# mini‑REPL
# -----------------------------------------------------------------------------
def main():
    print("maptest3 planner – give four ints: x0 y0  x1 y1   (blank to quit)")

    while True:
        try:
            line = input("> ").strip()
        except EOFError:
            break
        if not line or line.lower() in {"q", "quit", "exit"}:
            break

        try:
            x0, y0, x1, y1 = map(int, line.split())
        except ValueError:
            print("need exactly 4 integers")
            continue

        start, goal = (x0, y0), (x1, y1)
        if start not in graph or goal not in graph:
            print("start/goal not on a free cell – try again")
            continue

        dist, edges = solve(start, goal)
        png = render(edges, start, goal)
        print(f"distance {dist}  steps {len(edges)}  -> {png}")


if __name__ == "__main__":
    main()
