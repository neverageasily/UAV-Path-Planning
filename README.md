# UAV Path Planning in MATLAB

## Overview

This project implements and compares two path-planning algorithms in MATLAB:

- A* Search
- Rapidly-exploring Random Tree (RRT)

Both algorithms are tested on three 25 × 25 grid-based obstacle maps with the same start and goal positions.

## What I Implemented

- A* search with Manhattan-distance heuristic
- RRT with random sampling and nearest-node expansion
- Obstacle and boundary checking
- Parent-based path reconstruction
- Diagonal corner-collision checking for RRT
- Performance evaluation using planning time, path steps, and travelling distance

## Results

### A*

A* consistently found the shortest path in all three cases:

- 48 steps
- 48 m travelling distance

### RRT

RRT was evaluated over 10 runs for each case.

| Case | Avg. Planning Time (s) | Avg. Steps | Avg. Distance (m) |
|---|---:|---:|---:|
| Case 1 | 0.009243 | 33.3 | 43.78 |
| Case 2 | 0.004355 | 35.4 | 46.58 |
| Case 3 | 0.003376 | 34.9 | 45.79 |

RRT produces different paths across runs because of random sampling.

> Note: A* uses four-directional movement, while RRT allows diagonal movement, so their path lengths should not be compared directly as shortest-path performance.

## Project Structure

```text
UAV-Path-Planning/
├── Astar_main.m
├── RRT_main.m
├── map_cases.m
└── results/
```

## How to Run

Open MATLAB and run:

```matlab id="sxkekv"
Astar_main
```

or

```matlab id="lu0ye2"
RRT_main
```

Then select obstacle Case 1, 2, or 3 when prompted.

## Environment

MATLAB
