# Biorobotics Lab – Testing

Personal repo for implementing and testing pathfinding algorithms for the CMU Biorobotics Lab.

## Structure

- **AStar/** – A* search implementation
- **CBS/** – Conflict-Based Search (multi-agent pathfinding)

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
