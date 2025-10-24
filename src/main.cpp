#include <iostream>
#include <string>
#include <cmath>
#include <optional>
#include "astar.hpp"
#include "graph.hpp"

using Node = std::string;

static double zero_heuristic(const Node&, const Node&) { return 0.0; }

static bool parse_xy(const std::string& s, int& x, int& y) {
  auto pos = s.find(',');
  if (pos == std::string::npos) return false;
  try {
    x = std::stoi(s.substr(0, pos));
    y = std::stoi(s.substr(pos + 1));
  } catch (...) { return false; }
  return true;
}
static double manhattan(const Node& a, const Node& b) {
  int ax, ay, bx, by; if (!parse_xy(a, ax, ay) || !parse_xy(b, bx, by)) return 0.0;
  return std::abs(ax - bx) + std::abs(ay - by);
}
static double euclidean(const Node& a, const Node& b) {
  int ax, ay, bx, by; if (!parse_xy(a, ax, ay) || !parse_xy(b, bx, by)) return 0.0;
  double dx = double(ax - bx), dy = double(ay - by);
  return std::sqrt(dx*dx + dy*dy);
}

int main(int argc, char** argv) {
  std::string graph_path; Node src, dst; bool undirected=false; std::string heur="manhattan";
  for (int i=1; i<argc; ++i) {
    std::string arg = argv[i];
    auto need = [&](const char* name){ if (i+1>=argc) { std::cerr << "missing value for " << name << "\n"; std::exit(1);} return std::string(argv[++i]); };
    if (arg == "--graph") graph_path = need("--graph");
    else if (arg == "--src") src = need("--src");
    else if (arg == "--dst") dst = need("--dst");
    else if (arg == "--heuristic") heur = need("--heuristic");
    else if (arg == "--undirected") undirected = true;
    else { std::cerr << "Unknown arg: " << arg << "\n"; return 1; }
  }
  if (graph_path.empty() || src.empty() || dst.empty()) {
    std::cerr << "Usage: astar --graph <file> --src <id> --dst <id> [--undirected] [--heuristic none|manhattan|euclidean]\n";
    return 1;
  }

  auto gopt = load_graph(graph_path, undirected);
  if (!gopt) { std::cerr << "Failed to read graph: " << graph_path << "\n"; return 1; }
  const Graph G = std::move(*gopt);

  AStar<Node>::NeighFn neigh = [&](const Node& n) -> const AStar<Node>::NeighborList& {
    return G.neighbors(n);
  };

  AStar<Node>::HeurFn h = zero_heuristic;
  if (heur == "none") h = zero_heuristic;
  else if (heur == "manhattan") h = manhattan;
  else if (heur == "euclidean") h = euclidean;
  else { std::cerr << "Unknown heuristic: " << heur << ". Falling back to none.\n"; }

  auto path = AStar<Node>::run(src, dst, neigh, h);
  if (!path) { std::cout << "NO_PATH\n"; return 0; }

  double cost = 0.0;
  for (size_t i=1; i<path->size(); ++i) {
    const auto& adj = G.neighbors((*path)[i-1]);
    bool found=false; for (const auto& pr : adj) if (pr.first == (*path)[i]) { cost += pr.second; found=true; break; }
    if (!found) { std::cerr << "Warning: missing edge in path while summing cost\n"; }
  }

  std::cout << "COST " << cost << "\n";
  std::cout << "PATH ";
  for (size_t i=0; i<path->size(); ++i) {
    if (i) std::cout << " ";
    std::cout << (*path)[i];
  }
  std::cout << "\n";
  return 0;
}
