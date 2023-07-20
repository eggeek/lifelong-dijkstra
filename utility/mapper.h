#pragma once
#include <vector>
#include <cstdint>
#include <iostream>
#include <map>
#include "list_graph.h"
#include "adj_graph.h"
#include "coord.h"
using namespace std;

class Mapper{
public:
  AdjGraph g;
  Mapper(){}
  Mapper(const ListGraph& listg, const vector<xyLoc>& coord):
    node_count_(listg.node_count()) {
      this->coord = coord;
      id2pos.clear();
      for (int i=0; i<node_count_; i++) {
        pos_to_node_[this->coord[i]] = i;
        id2pos[i] = i;
      }
      this->g = AdjGraph(listg);
  }

  int node_count()const{
    return node_count_;
  }

  xyLoc operator()(int x)const{ return coord[x]; }
  int operator()(xyLoc p)const{
    auto it = pos_to_node_.find(p);
    if (it == pos_to_node_.end()) return -1;
    else return it->second;
  }

  int getPos(int id) {
    assert(id2pos.find(id) != id2pos.end());
    return id2pos[id];
  }

private:
  int node_count_;
  map<xyLoc, int> pos_to_node_;
  vector<xyLoc> coord;
  map<int, int> id2pos;
};
