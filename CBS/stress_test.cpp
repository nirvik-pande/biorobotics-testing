#include "cbs.h"
#include <chrono>
#include <random>
#include <iomanip>

/*
 * CBS Stress Test — Success Rate vs Number of Agents
 *
 * Replicates the style of experiments from Sharon et al. (2015).
 */

bool generateInstance(const Grid& grid, int k, std::vector<Agent>& agents, std::mt19937& rng) {
    std::vector<Pos> free_cells;
    for (int y = 0; y < grid.height; y++)
        for (int x = 0; x < grid.width; x++)
            if (grid.isFree({x, y}))
                free_cells.push_back({x, y});

    if ((int)free_cells.size() < 2 * k) return false;

    std::shuffle(free_cells.begin(), free_cells.end(), rng);

    agents.clear();
    for (int i = 0; i < k; i++)
        agents.push_back({i, free_cells[i], free_cells[k + i]});
    return true;
}

int main() {
    const int GRID_SIZE     = 8;
    const int OBSTACLE_PCT  = 0;        
    const int INSTANCES     = 25;       
    const int NODE_LIMIT    = 5000;     
    const double TIME_LIMIT = 10.0;     
    const int K_MIN         = 2;
    const int K_MAX         = 20;

    std::mt19937 rng(12345);

    Grid grid(GRID_SIZE, GRID_SIZE);
    if (OBSTACLE_PCT > 0) {
        for (int y = 0; y < GRID_SIZE; y++)
            for (int x = 0; x < GRID_SIZE; x++)
                if ((int)(rng() % 100) < OBSTACLE_PCT)
                    grid.setObstacle(x, y);
    }

    int free_count = 0;
    for (int i = 0; i < GRID_SIZE * GRID_SIZE; i++)
        if (!grid.obstacles[i]) free_count++;

    std::cout << "CBS Stress Test — Success Rate vs Agent Count\n";
    std::cout << "Grid: " << GRID_SIZE << "x" << GRID_SIZE
              << " | Free: " << free_count
              << " | Instances/k: " << INSTANCES
              << " | Node limit: " << NODE_LIMIT
              << " | Time limit: " << TIME_LIMIT << "s\n";
    std::cout << std::string(76, '=') << "\n";
    std::cout << std::setw(4) << "k"
              << std::setw(10) << "solved"
              << std::setw(10) << "rate%"
              << std::setw(12) << "avg_ms"
              << std::setw(12) << "avg_exp"
              << std::setw(12) << "avg_gen"
              << std::setw(10) << "avg_cost" << "\n";
    std::cout << std::string(76, '-') << "\n";

    std::vector<std::pair<int,double>> chart_data;

    for (int k = K_MIN; k <= K_MAX; k++) {
        int solved = 0;
        double total_time = 0, total_exp = 0, total_gen = 0, total_cost = 0;

        for (int inst = 0; inst < INSTANCES; inst++) {
            std::vector<Agent> agents;
            if (!generateInstance(grid, k, agents, rng))
                continue;

            CBS cbs(grid, agents);

            auto t0 = std::chrono::high_resolution_clock::now();
            bool ok = cbs.solve(NODE_LIMIT);
            auto t1 = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

            if (ms > TIME_LIMIT * 1000) ok = false;

            if (ok) {
                solved++;
                total_time += ms;
                total_exp += cbs.getNodesExpanded();
                total_gen += cbs.getNodesGenerated();
                total_cost += cbs.getSolutionCost();
            }
        }

        double rate = 100.0 * solved / INSTANCES;
        double avg_ms  = solved > 0 ? total_time / solved : 0;
        double avg_exp = solved > 0 ? total_exp / solved : 0;
        double avg_gen = solved > 0 ? total_gen / solved : 0;
        double avg_cost = solved > 0 ? total_cost / solved : 0;

        chart_data.push_back({k, rate});

        std::cout << std::setw(4) << k
                  << std::setw(7) << solved << "/" << std::setw(2) << INSTANCES
                  << std::setw(9) << std::fixed << std::setprecision(0) << rate << "%"
                  << std::setw(12) << std::setprecision(1) << avg_ms
                  << std::setw(12) << std::setprecision(0) << avg_exp
                  << std::setw(12) << avg_gen
                  << std::setw(10) << std::setprecision(1) << avg_cost << "\n";

        if (solved == 0 && k > K_MIN + 2) {
            std::cout << "[Stopped: 0% success rate]\n";
            for (int kk = k + 1; kk <= K_MAX; kk++)
                chart_data.push_back({kk, 0});
            break;
        }
    }

    std::cout << "\n" << std::string(76, '=') << "\n";
    std::cout << "Success Rate vs k  (each block = 2%)\n";
    std::cout << std::string(76, '-') << "\n";

    for (auto& [k, rate] : chart_data) {
        int bars = (int)(rate / 2.0 + 0.5);
        std::cout << "k=" << std::setw(2) << k << " |";
        for (int i = 0; i < bars; i++) std::cout << "#";
        for (int i = bars; i < 50; i++) std::cout << " ";
        std::cout << "| " << std::setw(3) << (int)rate << "%\n";
    }

    std::cout << "      " << std::string(50, '-') << "\n";
    std::cout << "      0%       20%       40%       60%       80%      100%\n";

    return 0;
}