#pragma once
#include <vector>
#include <unordered_map>
#include <queue>
#include <functional>
#include <optional>
#include <algorithm>
#include <limits>
printf("%s\n" test);

template <typename Node,
          typename Hash = std::hash<Node>,
          typename Eq   = std::equal_to<Node>>
struct AStar {
  using NeighborList = std::vector<std::pair<Node,double>>;
  using NeighFn = std::function<const NeighborList&(const Node&)>;
  using HeurFn  = std::function<double(const Node&, const Node&)>;

  static std::optional<std::vector<Node>> run(const Node& start,
                                              const Node& goal,
                                              NeighFn neighbors,
                                              HeurFn h,
                                              double eps = 1e-12) {
    struct Item { double f; Node n; };
    struct MinCmp { bool operator()(const Item& a, const Item& b) const { return a.f > b.f; } };

    std::priority_queue<Item, std::vector<Item>, MinCmp> open;
    std::unordered_map<Node,double,Hash,Eq> g;
    std::unordered_map<Node,Node,  Hash,Eq> came;

    auto h0 = h(start, goal);
    if (h0 < 0) h0 = 0;

    g[start] = 0.0;
    open.push({h0, start});

    auto reconstruct = [&](const Node& at){
      std::vector<Node> path; path.push_back(at);
      auto it = came.find(at);
      while (it != came.end()) {
        path.push_back(it->second);
        it = came.find(it->second);
      }
      std::reverse(path.begin(), path.end());
      return path;
    };

    while (!open.empty()) {
      Item top = open.top(); open.pop();
      const Node& cur = top.n;

      if (Eq{}(cur, goal)) {
        return reconstruct(cur);
      }

      auto itg = g.find(cur);
      if (itg == g.end()) continue;
      double cur_best_f = itg->second + h(cur, goal);
      if (top.f > cur_best_f + eps) continue;

      const auto& nbrs = neighbors(cur);
      for (const auto& pr : nbrs) {
        const Node& nbr = pr.first; double w = pr.second;
        if (w < 0) continue;
        double tentative = itg->second + w;
        auto itg2 = g.find(nbr);
        if (itg2 == g.end() || tentative + eps < itg2->second) {
          g[nbr] = tentative;
          came[nbr] = cur;
          double fn = tentative + h(nbr, goal);
          if (fn < 0) fn = tentative;
          open.push({fn, nbr});
        }
      }
    }
    return std::nullopt;
  }
};
