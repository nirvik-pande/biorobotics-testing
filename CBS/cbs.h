#pragma once
#include "common.h"
#include "grid.h"
#include "low_level.h"
#include <queue>
#include <list>
#include <memory>

/*
 * Conflict-Based Search (CBS) — High Level
 *
 * Two-level algorithm:
 *
 *   HIGH LEVEL: Best-first search over a Constraint Tree (CT).
 *     - Each CT node stores: constraints, a solution (path per agent), and cost (sum of path lengths).
 *     - Root: plan each agent independently with A*. No constraints.
 *     - Expand: detect first conflict -> branch into 2 children, each adding one constraint.
 *     - Solution: CT node with zero conflicts.
 *
 *   LOW LEVEL: Space-Time A* (low_level.h).
 *     - Given constraints for one agent, find shortest path obeying them.
 *
 *  This is standard CBS (Sharon et al., 2015). No CBSH (Jiaoyang Li) improvements.
 */

struct CTNode {
    std::vector<Path> paths;            
    std::vector<Constraint> constraints;
    int cost;                           

    bool operator>(const CTNode& o) const { return cost > o.cost; }

    void computeCost() {
        cost = 0;
        for (auto& p : paths)
            cost += (int)p.size() - 1;
    }
};

class CBS {
public:
    CBS(const Grid& grid, const std::vector<Agent>& agents)
        : grid_(grid), agents_(agents) {}

    bool solve(int max_nodes = 100000) {
        nodes_expanded_ = 0;
        nodes_generated_ = 0;

        auto root = std::make_shared<CTNode>();
        root->paths.resize(agents_.size());
        for (auto& a : agents_) {
            root->paths[a.id] = SpaceTimeAStar::findPath(grid_, a, {});
            if (root->paths[a.id].empty()) {
                std::cout << "No path exists for agent " << a.id << "\n";
                return false;
            }
        }
        root->computeCost();
        nodes_generated_++;

        auto cmp = [](const std::shared_ptr<CTNode>& a, const std::shared_ptr<CTNode>& b) {
            return a->cost > b->cost;
        };
        std::priority_queue<std::shared_ptr<CTNode>,
                            std::vector<std::shared_ptr<CTNode>>,
                            decltype(cmp)> open(cmp);
        open.push(root);

        while (!open.empty() && nodes_expanded_ < max_nodes) {
            auto curr = open.top(); open.pop();
            nodes_expanded_++;

            Conflict conflict;
            if (!findFirstConflict(curr->paths, conflict)) {
                solution_ = curr->paths;
                solution_cost_ = curr->cost;
                return true;
            }

            for (int i = 0; i < 2; i++) {
                auto child = std::make_shared<CTNode>();
                child->constraints = curr->constraints;

                Constraint new_c;
                new_c.agent = (i == 0) ? conflict.a1 : conflict.a2;
                new_c.timestep = conflict.timestep;
                new_c.is_edge = conflict.is_edge;

                if (!conflict.is_edge) {
                    new_c.loc = conflict.loc;
                    new_c.loc2 = {-1, -1};
                } else {
                    if (i == 0) {
                        new_c.loc = conflict.loc;
                        new_c.loc2 = conflict.loc2;
                    } else {
                        new_c.loc = conflict.loc2;
                        new_c.loc2 = conflict.loc;
                    }
                }

                child->constraints.push_back(new_c);

                child->paths = curr->paths;
                int ag = new_c.agent;
                Path new_path = SpaceTimeAStar::findPath(grid_, agents_[ag],
                                                         child->constraints);
                if (new_path.empty())
                    continue; 

                child->paths[ag] = new_path;
                child->computeCost();
                nodes_generated_++;
                open.push(child);
            }
        }

        return false; 
    }

    const std::vector<Path>& getSolution() const { return solution_; }
    int getSolutionCost() const { return solution_cost_; }
    int getNodesExpanded() const { return nodes_expanded_; }
    int getNodesGenerated() const { return nodes_generated_; }

private:
    const Grid& grid_;
    const std::vector<Agent>& agents_;
    std::vector<Path> solution_;
    int solution_cost_ = -1;
    int nodes_expanded_ = 0;
    int nodes_generated_ = 0;

    static Pos getPos(const Path& path, int t) {
        if (t < (int)path.size()) return path[t];
        return path.back();
    }

    bool findFirstConflict(const std::vector<Path>& paths, Conflict& conflict) {
        int num_agents = (int)paths.size();
        int max_t = 0;
        for (auto& p : paths) max_t = std::max(max_t, (int)p.size());

        for (int a1 = 0; a1 < num_agents; a1++) {
            for (int a2 = a1 + 1; a2 < num_agents; a2++) {
                for (int t = 0; t < max_t; t++) {
                    Pos p1 = getPos(paths[a1], t);
                    Pos p2 = getPos(paths[a2], t);

                    if (p1 == p2) {
                        conflict = {a1, a2, p1, p1, t, false};
                        return true;
                    }

                    if (t + 1 < max_t) {
                        Pos p1_next = getPos(paths[a1], t + 1);
                        Pos p2_next = getPos(paths[a2], t + 1);
                        if (p1 == p2_next && p2 == p1_next) {
                            conflict = {a1, a2, p1, p2, t + 1, true};
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }
};