#pragma once
#include <algorithm>
#include <fstream>
#include <limits>
#include <string>
#include <iostream>
#include <vector>

#include "adj_graph.h"
#include "heap.h"
#include "mapper.h"

using namespace std;

class LifeLongDijkstra {
public:


  long long INF = numeric_limits<long long>::max();
  struct node {
    long long dist;
    int pid;
  };

  struct Edge {
    int from, to;
    long long d;
  };

  LifeLongDijkstra(const AdjGraph& g, const Mapper& mapper): g(g), mapper(mapper) {
    q = min_id_heap<long long>(g.node_count());
    dist.resize(g.node_count());
    pa.resize(g.node_count());
    nodes.resize(g.node_count());
    propa_q.resize(g.node_count());
    to_repair.resize(g.edge_cnt);
  }

  inline void reach(int from, int to, long long d) {
    node& c = nodes[to];
    c.dist = d, c.pid = from;
    dist[to] = d;
    q.push_or_decrease_key(to, c.dist);
  }

  inline long long old_dist(int v, const vector<long long>& dist0) {
    return this->delta + dist0[v];
  }

  void repair_dist(const vector<long long>& dist0) {
    for (int i=0; i<repair_cnt; i++) {
      const Edge& e = to_repair[i];
      if (dist[e.to] > e.d) reach(e.from, e.to, e.d);
    }
  }

  void propagate_dist(int v,  const vector<long long>& dist0) {
    //cerr << "propagate v: " << v << ", dist: " << dist[v] << endl;
    int front = 0, tail = 0;
    propa_q[tail++] = v;
    repair_cnt = 0;
    while (front < tail) {
      int cur = propa_q[front++];
      for (int i=0; i< g.out_deg(cur); i++) {
        const Arc& a = g.out(cur, i);
        if (pa[a.target] == cur) {  
           if (dist[cur] + a.weight < dist[a.target]) {
            dist[a.target] = dist[cur] + a.weight;
            propa_q[tail++] = a.target;
          }
        } else if (dist[a.target] > dist[cur] + a.weight) {
          to_repair[repair_cnt++] = {cur, a.target, dist[cur] + a.weight};
        }
      }
    }
    propa_q_idx = tail;
  }

  void set_pa(const vector<int>& pa0) {
    pa = vector<int>(pa0.begin(), pa0.end());
  }

  void reset() {
    fill(pa.begin(), pa.end(), -1);
    this->delta = 0;
    tot_prop = 0, pop_num = 0, valid_pop = 0;
    fill(dist.begin(), dist.end(), INF);
  }

  // void reuse(const vector<long long>& dist0, int newS, long long delta) {
  //   tot_prop = 0, pop_num = 0, valid_pop = 0;
  //   this->delta = delta;
  //   for (int i=0; i<g.node_count(); i++) {
  //     dist[i] = dist0[i] + this->delta;
  //   }
  // }

  void reuse(int newS, long long delta) {
    // reset statistic
    tot_prop = 0, pop_num = 0, valid_pop = 0;

    // delta = distance from old source to new source
    this->delta = delta;
    // update previous distance table by delta
    for (int i=0; i<g.node_count(); i++) {
      dist[i] += this->delta;
    }
  }

  pair<int, int> run(int s, const vector<long long>& dist0, bool verbose=false) {
    this->source = s;
    nodes[s] = {0, -1};
    dist[s] = 0;
    q.push_or_decrease_key(s, 0);

    while (!q.empty()) {
      int cid = q.pop();
      node& c = nodes[cid];
      pop_num++;
      assert(nodes[cid].dist >= dist[cid]);
      if (nodes[cid].dist != dist[cid]) continue;
      valid_pop++;
      // edge <c.pid, cid> is on the current shortest path tree,
      // so we update parent
      pa[cid] = c.pid;

      // we can control the propagation by setting up variable `f`
      // larger `f` results in more normal expansion at the beginning
      // we can also set an upper bound to avoid propagation in deep level.
      const long long f = 2;
      if (delta > 0 && dist[cid] > this->delta * f) {
        propa_q_idx = 0;
        propagate_dist(cid, dist0);
        tot_prop += propa_q_idx;
        if (repair_cnt) repair_dist(dist0);
      }
      else {
        for (int i=0; i<g.out_deg(cid); i++) {
          const Arc& a = g.out(cid, i);
          if (dist[a.target] > dist[cid] + a.weight)
              reach(cid, a.target, dist[cid] + a.weight);
        } 
      }
    }
    if (verbose) {
      cerr << "#propagation: " << tot_prop << ", #size: " << g.node_count() <<
        ", dist(s0, s): " << dist0[s] <<
        ", dist(s, s0): " << delta <<
        ", #pop: " << pop_num << ", #valid pop: " << valid_pop << endl;
    }
    return {tot_prop, pop_num};
  }

  vector<long long>& get_dist() { return dist; }
  const vector<int>& get_pa() { return pa; }

private:
  const AdjGraph& g;
  const Mapper& mapper;
  min_id_heap<long long> q;
  vector<long long> dist;
  vector<int> pa;
  vector<Edge> to_repair;
  int repair_cnt;

  vector<int> propa_q;
  int propa_q_idx;

  vector<node> nodes;
  long long delta;
  int source;
  long long tot_prop, pop_num, valid_pop;
};
