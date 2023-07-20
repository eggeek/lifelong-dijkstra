#include <numeric>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include "RoadNetworkLoader.h"
#include "dijkstra.h"
#include "lifelong_dij.h"

using namespace std;

void get_path(int s, int t, Dijkstra& dij, AdjGraph& g, vector<int>& path) {
  dij.run(s);
  path.clear();
  long long d = dij.get_dist()[t]; 
  int cur = t;
  while (d > 0) {
    path.push_back(cur);
    int pre = -1;
    for (auto a: g.out(cur)) if (dij.get_dist()[a.target] + a.weight == d) {
      pre = a.target;
      d -= a.weight;
      break;
    }
    assert(pre != -1);
    cur = pre;
  }
  path.push_back(s);
  reverse(path.begin(), path.end());
}

void validate_dist(const vector<long long>& dist, 
  const vector<long long>& dist_ll, const AdjGraph& g) {
  for (int i=0; i<g.node_count(); i++) {
    assert(dist[i] == dist_ll[i]);
  }
}

void validate_pa(
  const vector<int>& pa, const vector<long long>& dist,
  const vector<int>& pa_ll, const vector<long long>& dist_ll,
  const AdjGraph& g) {
  int n = pa.size();
  assert(dist.size() == n);
  assert(pa_ll.size() == n);
  assert(dist_ll.size() == n);

  for (int i=0; i<n; i++) {
    for (auto& a: g.out(i)) 
    if (pa[a.target] == i) {
      assert(dist[i] + a.weight == dist[a.target]);
    }
    for (auto& a: g.out(i)) 
    if (pa_ll[a.target] == i) {
      assert(dist_ll[i] + a.weight == dist_ll[a.target]);
      assert(dist_ll[i] + a.weight == dist_ll[a.target]);
    }
  }
}

int main() {
  vector<tuple<string, string, vector<int>>> argvs = 
  {
    {"./maps/NY/NY.gr", "./maps/NY/NY.co", {2, 3}},
    // {"./maps/NY/NY.gr", "./maps/NY/NY.co", {150, 146, 140, 138, 139, 141}},
    // {"./maps/NY/NY.gr", "./maps/NY/NY.co", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}},
    {"./maps/NY/NY.gr", "./maps/NY/NY.co", {124116,77419}},
    {"./maps/NY/NY.gr", "./maps/NY/NY.co", {224518,107452}},
    // {"./maps/COL/COL.gr", "./maps/COL/COL.co", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}},
    {"./maps/COL/COL.gr", "./maps/COL/COL.co", {1, 254}},
    {"./maps/COL/COL.gr", "./maps/COL/COL.co", {124116,77419}},
    // {"./maps/US/US-d.gr", "./maps/US/US-d.co", {1,254}},
    // {"./maps/US/US-d.gr", "./maps/US/US-d.co", {124116,77419}},
  };
  for (auto& argv: argvs) {
    string& gr = get<0>(argv);
    vector<int> nodes = get<2>(argv);
    vector<int> pa;
    vector<xyLoc> coord;
    vector<long long> dist, lldist;


    ListGraph listg = RoadNetwork::Load(gr);
    coord.resize(listg.node_count());
    pa.resize(listg.node_count());
    Mapper mapper(listg, coord);
    AdjGraph& g = mapper.g;
    Dijkstra dij(g, mapper);
    LifeLongDijkstra lldij(g, mapper);

    // If there are only two nodes (s and t), the sequence of nodes is all vertices along the shortest path from s to t
    if (nodes.size() == 2) {
      get_path(nodes[0], nodes[1], dij, g, nodes);
    }

    const long long INF = 10000000000000;
    double lldij_cost = 0, dij_cost = 0, avg_prop = 0, avg_expan = 0;
    dij.run(nodes[0]);
    dist = vector<long long>(dij.get_dist());
    lldist = vector<long long>(g.node_count(), 0);
    pa = vector<int>(g.node_count(), -1);
    lldij.set_pa(pa);

    for (int i=0; i<(int)nodes.size(); i++) {

      auto stime = std::chrono::steady_clock::now();
      dij.run(nodes[i]);
      auto etime = std::chrono::steady_clock::now();
      dij_cost += std::chrono::duration_cast<std::chrono::nanoseconds>(etime - stime).count();

      stime = std::chrono::steady_clock::now();
      // reuse the previously computed information
      if (i > 0) {
        long long delta = lldij.get_dist()[nodes[i]];
        lldij.reuse(nodes[i], delta);
      }
      // reset lldij for the first computation 
      else lldij.reset();
      auto res = lldij.run(nodes[i], lldist, true);
      lldist = vector<long long>(lldij.get_dist());
      etime = std::chrono::steady_clock::now();
      avg_prop += res.first;
      avg_expan += res.second;

      lldij_cost += std::chrono::duration_cast<std::chrono::nanoseconds>(etime - stime).count();

      // copy distance table for verification
      dist = vector<long long>(dij.get_dist());

      // build parent for normal dijkstra
      // set parent of source to -1
      pa[nodes[i]] = -1;
      build_pa(pa, dist, g);
      validate_dist(dist, lldist, g);

      // verify:
      // 1. `parent` of lldij must be same as `parent` of normal dijkstra;
      // 2. distance tables are also same;
      // meaning that they have same shortest path tree
      // (assuming no symmetires)
      validate_pa(pa, dist, lldij.get_pa(), lldist, g);
    }
    cout << "dij tcost: " << dij_cost << ", lldij tcost: " << lldij_cost 
         << ", speed up: " << dij_cost / lldij_cost
         << ", avg propagate: " << avg_prop / (double)nodes.size()
         << ", v: " << g.node_count()
         << ", avg expan: " << avg_expan / (double)nodes.size() << endl;
    cout << "==============" << endl;
  }
  return 0;
}
