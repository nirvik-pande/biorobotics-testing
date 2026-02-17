# Biorobotics Lab – Testing

Personal repo for implementing and testing pathfinding algorithms for the CMU Biorobotics Lab.

## Structure

- **AStar/** – A* search implementation
- **CBS/** – Conflict-Based Search (multi-agent pathfinding)
  - `cbs.h` / `low_level.h` – CBS solver and low-level planner
  - `astar.h` – A* used as the low-level search
  - `graph.h` / `grid.h` – Graph and grid representations
  - `common.h` – Shared types and utilities
  - `main.cpp` – Entry point
  - `stress_test.cpp` – Stress testing CBS performance
  - `CMakeLists.txt` – Build config

## Build (CBS)

```bash
cd CBS/build
cmake ..
make
```

## Run

```bash
./cbs          # normal run
./stress_test  # stress test
```

## Notes

- Personal testing workspace, not official lab code
- C++ with CMake build system
