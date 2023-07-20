#ifndef ADJ_GRAPH_H 
#define ADJ_GRAPH_H

#include "list_graph.h"
#include "range.h"
#include <iostream>

using namespace std;

class AdjGraph{
public:
  int edge_cnt = 0, num;
  AdjGraph(){}

  AdjGraph(ListGraph g){
    g.sort();
    out_arcs = vector<Arc>(g.arc);
    edge_cnt = g.arc.size();
    begin.resize(g.node_count());
    degree.resize(g.node_count());
    num = g.node_count();
    build_adj();
  }

  void build_adj() {
    fill(begin.begin(), begin.end(), -1);
    fill(degree.begin(), degree.end(), 0);

    for (int i=0; i<edge_cnt; i++) {
      int j = i;
      begin[out_arcs[i].source] = i;
      while (j+1<edge_cnt && out_arcs[j+1].source == out_arcs[i].source) j++;
      degree[out_arcs[i].source] = j - i + 1;
      i = j;
    }
  }

  AdjGraph&operator=(const ListGraph&o){
    return *this = AdjGraph(o);
  }

  inline int node_count()const{
    return begin.size();
  }

  const inline Range<std::vector<Arc>::const_iterator>out(int v)const{
    return make_range(out_arcs.begin() + begin[v], out_arcs.begin() + begin[v] + degree[v]);
  }

  const inline Arc& out(int v, int i)const{
    return out_arcs[begin[v] + i];
  }

  const inline Arc& arc(int idx) const {
    return out_arcs[idx];
  }

  const inline int ArcIndex(int v, int i) const { return begin[v] + i; }

  int out_deg(int v)const{
    return degree[v];
  }

  int max_degree() {
    int res = degree[0];
    for (int i=1; i<node_count(); i++) res = max(res, degree[i]);
    return res;
  }

  int max_degree_vert() {
    int maxd = max_degree();
    for (int i=0; i<(int)degree.size(); i++) {
      if ((int)degree[i] == maxd) return i;
    }
    return -1;
  }

private:
  vector<Arc> out_arcs;
  vector<int> begin, degree;
};

inline void build_pa(vector<int>& pa, const vector<long long>& dist, const AdjGraph& g) {
  for (int i=0; i<g.edge_cnt; i++) {
    const Arc& a = g.arc(i);
    if (dist[a.source] + a.weight == dist[a.target])
      pa[a.target] = a.source;
  }
}
#endif


