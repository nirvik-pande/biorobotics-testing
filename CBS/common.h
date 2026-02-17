#pragma once
#include <vector>
#include <tuple>
#include <iostream>
#include <algorithm>
#include <climits>

struct Pos {
    int x, y;
    bool operator==(const Pos& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Pos& o) const { return !(*this == o); }
};

struct PosHash {
    size_t operator()(const Pos& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 16);
    }
};


using Path = std::vector<Pos>;

struct Agent {
    int id;
    Pos start;
    Pos goal;
};


struct Constraint {
    int agent;
    Pos loc;
    Pos loc2;       
    int timestep;
    bool is_edge;   
};

struct Conflict {
    int a1, a2;     
    Pos loc;        
    Pos loc2;       
    int timestep;
    bool is_edge;
};

inline int manhattan(Pos a, Pos b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}