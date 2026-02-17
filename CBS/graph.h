#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <sstream>
#include <optional>

struct Graph {
  using Node = std::string;
  using Adj  = std::vector<std::pair<Node,double>>;
  std::unordered_map<Node, Adj> adj;

  const Adj& neighbors(const Node& n) const {
    static const Adj kEmpty;
    auto it = adj.find(n);
    return (it == adj.end()) ? kEmpty : it->second;
  }
};

inline void add_edge(Graph& G, const std::string& u, const std::string& v, double w, bool undirected) {
  G.adj[u].emplace_back(v, w);
  if (undirected) G.adj[v].emplace_back(u, w);
}

inline std::optional<Graph> load_graph(const std::string& path, bool undirected=false) {
  std::ifstream in(path);
  if (!in) return std::nullopt;
  Graph G;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0]=='#') continue;
    std::istringstream iss(line);
    std::string u, v; double w;
    if (!(iss >> u >> v >> w)) continue;
    add_edge(G, u, v, w, undirected);
  }
  return G;
}
