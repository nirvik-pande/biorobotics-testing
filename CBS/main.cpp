#include "cbs.h"
#include <chrono>

/*
 * TEST CBS Demo
 *
 * Runs CBS on a few example MAPF instances and prints the resulting paths.
 * Grid legend: # = obstacle, . = free, letters = agent starts
 */

void printSolution(const std::vector<Agent>& agents, const std::vector<Path>& paths,
                   const Grid& grid) {
    int max_t = 0;
    for (auto& p : paths) max_t = std::max(max_t, (int)p.size());

    // Print paths as text
    for (auto& a : agents) {
        auto& p = paths[a.id];
        std::cout << "  Agent " << a.id << ": ";
        for (int t = 0; t < (int)p.size(); t++) {
            if (t > 0) std::cout << " -> ";
            std::cout << "(" << p[t].x << "," << p[t].y << ")";
        }
        std::cout << "\n";
    }

    // Print timestep-by-timestep grid
    std::cout << "\n  Timestep view:\n";
    for (int t = 0; t < max_t; t++) {
        std::cout << "  t=" << t << ":\n";
        for (int y = 0; y < grid.height; y++) {
            std::cout << "    ";
            for (int x = 0; x < grid.width; x++) {
                if (grid.obstacles[y * grid.width + x]) {
                    std::cout << '#';
                    continue;
                }
                char c = '.';
                for (auto& a : agents) {
                    Pos p = (t < (int)paths[a.id].size()) ? paths[a.id][t] : paths[a.id].back();
                    if (p.x == x && p.y == y) {
                        c = '0' + a.id;
                        break;
                    }
                }
                std::cout << c;
            }
            std::cout << '\n';
        }
    }
}

// ---- Test Scenarios ----

void testSwap() {
    std::cout << "=== Test 1: Two agents swap positions ===\n";
    // 5x1 corridor: agents must pass each other
    //   A . . . B    ->    B . . . A
    Grid grid(5, 3);
    // Add walls
    for (int x = 0; x < 5; x++) {
        grid.setObstacle(x, 0);
        grid.setObstacle(x, 2);
    }
    // Open the corridor
    for (int x = 0; x < 5; x++) {
        grid.obstacles[1 * 5 + x] = false;
    }
    // Make a bypass cell
    grid.obstacles[0 * 5 + 2] = false;

    std::vector<Agent> agents = {
        {0, {0, 1}, {4, 1}},
        {1, {4, 1}, {0, 1}},
    };

    CBS cbs(grid, agents);
    auto t0 = std::chrono::high_resolution_clock::now();
    bool solved = cbs.solve();
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (solved) {
        std::cout << "  Solved! Cost=" << cbs.getSolutionCost()
                  << " Expanded=" << cbs.getNodesExpanded()
                  << " Generated=" << cbs.getNodesGenerated()
                  << " Time=" << ms << "ms\n";
        printSolution(agents, cbs.getSolution(), grid);
    } else {
        std::cout << "  No solution found.\n";
    }
    std::cout << "\n";
}

void testCross() {
    std::cout << "=== Test 2: Two agents cross paths on a 5x5 grid ===\n";
    Grid grid(5, 5);

    std::vector<Agent> agents = {
        {0, {0, 2}, {4, 2}},  
        {1, {2, 0}, {2, 4}},  
    };

    CBS cbs(grid, agents);
    auto t0 = std::chrono::high_resolution_clock::now();
    bool solved = cbs.solve();
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (solved) {
        std::cout << "  Solved! Cost=" << cbs.getSolutionCost()
                  << " Expanded=" << cbs.getNodesExpanded()
                  << " Generated=" << cbs.getNodesGenerated()
                  << " Time=" << ms << "ms\n";
        printSolution(agents, cbs.getSolution(), grid);
    } else {
        std::cout << "  No solution found.\n";
    }
    std::cout << "\n";
}

void testMultiAgent() {
    std::cout << "=== Test 3: Four agents on 8x8 grid with obstacles ===\n";
    Grid grid(8, 8);

    grid.setObstacle(2, 1); grid.setObstacle(2, 2); grid.setObstacle(2, 3);
    grid.setObstacle(5, 4); grid.setObstacle(5, 5); grid.setObstacle(5, 6);
    grid.setObstacle(3, 5);

    std::vector<Agent> agents = {
        {0, {0, 0}, {7, 7}},
        {1, {7, 0}, {0, 7}},
        {2, {0, 7}, {7, 0}},
        {3, {7, 7}, {0, 0}},
    };

    CBS cbs(grid, agents);
    auto t0 = std::chrono::high_resolution_clock::now();
    bool solved = cbs.solve();
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (solved) {
        std::cout << "  Solved! Cost=" << cbs.getSolutionCost()
                  << " Expanded=" << cbs.getNodesExpanded()
                  << " Generated=" << cbs.getNodesGenerated()
                  << " Time=" << ms << "ms\n";

        for (auto& a : agents) {
            auto& p = cbs.getSolution()[a.id];
            std::cout << "  Agent " << a.id << " (" << p.size() - 1 << " steps): ";
            for (int t = 0; t < (int)p.size(); t++) {
                if (t > 0) std::cout << "->";
                std::cout << "(" << p[t].x << "," << p[t].y << ")";
            }
            std::cout << "\n";
        }
    } else {
        std::cout << "  No solution found.\n";
    }
    std::cout << "\n";
}

int main() {
    std::cout << "Simple CBS (Conflict-Based Search) for MAPF\n";
    std::cout << "=============================================\n\n";

    testSwap();
    testCross();
    testMultiAgent();

    return 0;
}