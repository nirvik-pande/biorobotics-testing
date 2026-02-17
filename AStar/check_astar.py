import subprocess, sys
import networkx as nx

# usage: python check_astar.py <graph.tsv> <src> <dst> [undirected]

def load_graph(path, undirected=False):
    G = nx.DiGraph()
    with open(path) as f:
        for line in f:
            if not line.strip() or line.startswith('#'): continue
            u,v,w = line.split()
            w = float(w)
            G.add_edge(u,v,weight=w)
            if undirected:
                G.add_edge(v,u,weight=w)
    return G

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("usage: check_astar.py <graph.tsv> <src> <dst> [undirected]")
        sys.exit(1)
    path, src, dst = sys.argv[1:4]
    undirected = (len(sys.argv) > 4)

    G = load_graph(path, undirected)

    sp = nx.shortest_path(G, src, dst, weight='weight')
    sp_cost = nx.path_weight(G, sp, weight='weight')
    def manhattan(a,b):
        try:
            ax, ay = map(int, a.split(','))
            bx, by = map(int, b.split(','))
            return abs(ax-bx) + abs(ay-by)
        except:
            return 0.0

    ap = nx.astar_path(G, src, dst, heuristic=manhattan, weight='weight')
    ap_cost = nx.path_weight(G, ap, weight='weight')

    def manhattan(a,b):
        try:
            ax, ay = map(int, a.split(','))
            bx, by = map(int, b.split(','))
            return abs(ax-bx) + abs(ay-by)
        except:
            return 0.0

    ap = nx.astar_path(G, src, dst, heuristic=manhattan, weight='weight')
    ap_cost = nx.path_weight(G, ap, weight='weight')

    cmd = ["build/astar", "--graph", path, "--src", src, "--dst", dst, "--heuristic", "manhattan"]
    if undirected: cmd.append("--undirected")
    out = subprocess.check_output(cmd, text=True)
    cost_line = [l for l in out.splitlines() if l.startswith("COST ")][0]
    cxx_cost = float(cost_line.split()[1])

    print(f"NetworkX Dijkstra cost = {sp_cost}")
    print(f"NetworkX A* cost       = {ap_cost}")
    print(f"C++ A* cost            = {cxx_cost}")
    print("MATCHES OPTIMAL?", abs(cxx_cost - sp_cost) < 1e-9)