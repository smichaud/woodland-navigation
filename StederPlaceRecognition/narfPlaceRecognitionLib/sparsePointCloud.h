#ifndef SPARSE_MAP_H
#define SPARSE_MAP_H

#include <map>
#include <set>

struct SparsePointCloud {
  SparsePointCloud(float cellSize=0.1f) : cellSize(cellSize) {}
  float cellSize;
  void add(float x, float y, float z) {
    points[lrintf(x/cellSize)][lrintf(y/cellSize)].insert(lrintf(z/cellSize));
  }
  void clear() {
    points.clear();
  }
  
  std::map<int, std::map<int, std::set<int> > > points;
};

#endif
