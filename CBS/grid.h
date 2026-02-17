#pragma once
#include "common.h"
#include <unordered_set>
#include <string>
#include <fstream>
#include <sstream>

class Grid {
public:
    int width, height;
    std::vector<bool> obstacles; 
    Grid(int w, int h) : width(w), height(h), obstacles(w * h, false) {}

    void setObstacle(int x, int y) { obstacles[y * width + x] = true; }

    bool inBounds(Pos p) const {
        return p.x >= 0 && p.x < width && p.y >= 0 && p.y < height;
    }

    bool isFree(Pos p) const {
        return inBounds(p) && !obstacles[p.y * width + p.x];
    }

    std::vector<Pos> getNeighbors(Pos p) const {
        std::vector<Pos> result;
        Pos dirs[] = {{1,0},{-1,0},{0,1},{0,-1},{0,0}};
        for (auto& d : dirs) {
            Pos next = {p.x + d.x, p.y + d.y};
            if (isFree(next))
                result.push_back(next);
        }
        return result;
    }

    void print(const std::vector<Agent>& agents) const {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (obstacles[y * width + x]) {
                    std::cout << '#';
                } else {
                    bool found = false;
                    for (auto& a : agents) {
                        if (a.start.x == x && a.start.y == y) {
                            std::cout << (char)('A' + a.id);
                            found = true; break;
                        }
                    }
                    if (!found) std::cout << '.';
                }
            }
            std::cout << '\n';
        }
    }
};