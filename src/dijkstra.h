#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <algorithm>
#include <vector>
#include <limits>

#include "adj_graph.h"
#include "heap.h"
#include "mapper.h"

using namespace std;

class Dijkstra{
public:
  Dijkstra(const AdjGraph&g, const Mapper& mapper):
    g(g), q(g.node_count()), dist(g.node_count()), mapper(mapper) {}

  void run(int source_node){
    std::fill(dist.begin(), dist.end(), std::numeric_limits<long long>::max());

    dist[source_node] = 0;

    auto reach = [&](const Arc& a, long long d){
      int v = a.target;
      if(d < dist[v]){
        q.push_or_decrease_key(v, d);
        dist[v] = d;
      }
    };

    assert(q.empty());

    for(int i=0; i<g.out_deg(source_node); ++i){
      auto a = g.out(source_node, i);
      reach(a, a.weight);
    }

    while(!q.empty()){
      int x = q.pop();
      for(auto a:g.out(x)) {
        reach(a, dist[x] + a.weight);
      }
    }

    #ifndef NDEBUG
    for(int u=0; u<g.node_count(); ++u)
      for(auto uv : g.out(u)){
        int v = uv.target;
        assert(dist[u] >= dist[v] - uv.weight);
      }
    #endif
  }

  long long distance(int target)const{
    return dist[target];
  }

  const vector<long long>& get_dist() const {
    return dist;
  }

private:
  const AdjGraph&g;
  min_id_heap<long long >q;
  std::vector<long long>dist;
  const Mapper& mapper;
};

#endif
