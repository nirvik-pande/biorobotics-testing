#pragma once
#include "common.h"
#include "grid.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>

/*
 * Space-Time A*
 *
 * Plans a single agent's path on a grid, respecting a set of constraints.
 * Search state = (position, timestep). Heuristic = Manhattan distance.
 *
 * Vertex constraint: agent can't be at loc at timestep.
 * Edge constraint: agent can't move from loc to loc2 at timestep.
 */

struct STNode {
    Pos pos;
    int t;
    int g, f;
    bool operator>(const STNode& o) const {
        return f > o.f || (f == o.f && g < o.g); 
    }
};

struct StateHash {
    size_t operator()(const std::pair<Pos,int>& s) const {
        auto h = PosHash()(s.first);
        return h ^ (std::hash<int>()(s.second) << 1);
    }
};
struct StateEq {
    bool operator()(const std::pair<Pos,int>& a, const std::pair<Pos,int>& b) const {
        return a.first == b.first && a.second == b.second;
    }
};

using State = std::pair<Pos,int>;

class SpaceTimeAStar {
public:
    static Path findPath(const Grid& grid, const Agent& agent,
                         const std::vector<Constraint>& constraints,
                         int max_time = 200)
    {
        std::unordered_set<State, StateHash, StateEq> vertex_cons;
        std::unordered_set<long long> edge_cons;

        for (auto& c : constraints) {
            if (c.agent != agent.id) continue;
            if (!c.is_edge) {
                vertex_cons.insert({c.loc, c.timestep});
            } else {
                edge_cons.insert(encodeEdge(c.loc, c.loc2, c.timestep));
            }
        }

        // A* search
        std::priority_queue<STNode, std::vector<STNode>, std::greater<STNode>> open;
        std::unordered_map<State, int, StateHash, StateEq> best_g;
        std::unordered_map<State, State, StateHash, StateEq> came_from;

        State start_state = {agent.start, 0};
        open.push({agent.start, 0, 0, manhattan(agent.start, agent.goal)});
        best_g[start_state] = 0;

        while (!open.empty()) {
            auto curr = open.top(); open.pop();
            State curr_state = {curr.pos, curr.t};

            if (curr.pos == agent.goal) {
                return reconstructPath(came_from, curr_state);
            }

            if (curr.t >= max_time) continue;

            if (best_g.count(curr_state) && curr.g > best_g[curr_state])
                continue;

            for (auto& next_pos : grid.getNeighbors(curr.pos)) {
                int next_t = curr.t + 1;
                State next_state = {next_pos, next_t};

                if (vertex_cons.count(next_state)) continue;

                if (edge_cons.count(encodeEdge(curr.pos, next_pos, next_t))) continue;

                int next_g = curr.g + 1;
                if (!best_g.count(next_state) || next_g < best_g[next_state]) {
                    best_g[next_state] = next_g;
                    int h = manhattan(next_pos, agent.goal);
                    open.push({next_pos, next_t, next_g, next_g + h});
                    came_from[next_state] = curr_state;
                }
            }
        }

        return {};
    }

private:
    static long long encodeEdge(Pos from, Pos to, int t) {
  
        long long key = (long long)from.x;
        key = key * 1000 + from.y;
        key = key * 1000 + to.x;
        key = key * 1000 + to.y;
        key = key * 100000 + t;
        return key;
    }

    static Path reconstructPath(
        const std::unordered_map<State, State, StateHash, StateEq>& came_from,
        State goal_state)
    {
        Path path;
        State s = goal_state;
        while (came_from.count(s)) {
            path.push_back(s.first);
            s = came_from.at(s);
        }
        path.push_back(s.first); 
        std::reverse(path.begin(), path.end());
        return path;
    }
};