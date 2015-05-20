/* \author Bastian Steder - steder@informatik.uni-freiburg.de */

#include "farthestPoint.h"
#include <kdtree++/kdtree.hpp>
#include <iostream>
#include <queue>
#include <set>

//namespace Tmp {
  //inline double getTime() {
    //struct timeval ts; 
    //gettimeofday(&ts,0);
    //return ts.tv_sec + ts.tv_usec*1e-6;
  //}
//}

template <typename Real>
struct KdTreePoint {
  KdTreePoint(const Real* point) : point(point) {}
  const Real* point;
  typedef Real value_type;
  inline value_type operator[](size_t const N) const { return point[N]; }
};
template <typename Real>
inline bool operator==(const KdTreePoint<Real>& p1, const KdTreePoint<Real>& p2) {
  return p1.point==p2.point;
}

template <typename Real>
struct NotSame {
  const KdTreePoint<Real> point;
  NotSame(const KdTreePoint<Real> point) : point(point) {}
  bool operator()(const KdTreePoint<Real>& point2) const { return point2.point != point.point; }
};

template <typename Real>
struct PointWithNeighbor {
  PointWithNeighbor(const Real* point, const Real* neighbor, Real distance) : point(point), neighbor(neighbor), distance(distance) {}
  const Real* point;
  const Real* neighbor;
  Real distance;
};

template <typename Real>
struct LowerDistanceComparator {
  bool operator()(const PointWithNeighbor<Real>& p1, const PointWithNeighbor<Real>& p2) { return p1.distance<p2.distance; }
};
template <typename Real>
struct LowerNeighborIdxComparator {
  bool operator()(const PointWithNeighbor<Real>& p1, const PointWithNeighbor<Real>& p2) { return p1.neighbor<p2.neighbor; }
};


//found = kdtree.find_nearest_if( ... , not_me(target.id) );

template <typename Real, size_t Dim>
void FarthestPoint<Real,Dim>::subsample(const std::vector<Real>& points, size_t remainingNoOfPoints,
                                        Real minDistance, std::vector<size_t>& indices)
{
  //double time1 = Tmp::getTime();

  indices.clear();
  size_t noOfPoints = points.size()/Dim;
  if (noOfPoints < remainingNoOfPoints && minDistance<=0) {
    for(size_t pointIdx=0; pointIdx<noOfPoints; ++pointIdx)
      indices.push_back(pointIdx);
    return;
  }
  
  typedef KDTree::KDTree<Dim, KdTreePoint<Real> > TreeType;
  TreeType kdTree;
  for(size_t idx=0; idx<points.size(); idx+=Dim)
    kdTree.insert(KdTreePoint<Real>(&points[idx]));
  kdTree.optimize();
  
  size_t lastOptimizedKdTreeSize = kdTree.size();
  
  //double time2 = Tmp::getTime();
  
  typedef std::multiset<PointWithNeighbor<Real>,LowerDistanceComparator<Real> > DistanceSet;
  typedef typename DistanceSet::iterator DistanceIterator;
  DistanceSet pointsSortedByDistance;
  typedef std::multiset<PointWithNeighbor<Real>,LowerNeighborIdxComparator<Real> > NeighborSet;
  typedef typename NeighborSet::iterator NeighborIterator;
  NeighborSet pointsSortedByNeighbor;
  
  std::vector<PointWithNeighbor<Real> > tmp;
  //tmp.reserve(noOfPoints);
  for(size_t idx=0; idx<points.size(); idx+=Dim) {
    KdTreePoint<Real> point = KdTreePoint<Real>(&points[idx]);
    std::pair<typename TreeType::const_iterator, typename TreeType::distance_type> neighbor =
        kdTree.find_nearest_if(point, 1e10, NotSame<Real>(point));
    PointWithNeighbor<Real> pointWithNeighbor(point.point, neighbor.first->point, neighbor.second);
    tmp.push_back(pointWithNeighbor);
  }
  
  //double time3 = Tmp::getTime();
  
  std::sort(tmp.begin(), tmp.end(), LowerDistanceComparator<Real>());
  
  Real minDistanceSquared = minDistance*minDistance;
  //while (tmp.front().distance < minDistanceSquared && !tmp.empty())
    //tmp.pop_front();
  //std::cout << "Enforcing minDistance reduced no of keypoints from "<<noOfPoints<<" to "<<tmp.size()<<".\n";
  //noOfPoints = tmp.size();
  
  for(size_t pointIdx=0; pointIdx<noOfPoints; ++pointIdx)
    pointsSortedByDistance.insert(pointsSortedByDistance.end(), tmp[pointIdx]);
  
  std::sort(tmp.begin(), tmp.end(), LowerNeighborIdxComparator<Real>());
  for(size_t pointIdx=0; pointIdx<noOfPoints; ++pointIdx)
    pointsSortedByNeighbor.insert(pointsSortedByNeighbor.end(), tmp[pointIdx]);
  
  //double time4 = Tmp::getTime();
  
  // TODO: Could improve runtime by removing more than one in one step. Every point whose neighbor was not removed yet can also be removed as long as the number of points to be recalculated is lower than the number of still to be removed points

  std::vector<PointWithNeighbor<Real> > recalculatedPoints;
  while (pointsSortedByDistance.size()>remainingNoOfPoints ||
         (!pointsSortedByDistance.empty()&&pointsSortedByDistance.begin()->distance<minDistanceSquared))
  {
    recalculatedPoints.clear();
    
    PointWithNeighbor<Real> pointToRemove = *pointsSortedByDistance.begin();
    // TODO: Need to decide if better to delete neighbor
    pointsSortedByDistance.erase(pointsSortedByDistance.begin());
    kdTree.erase_exact(KdTreePoint<Real>(pointToRemove.point));
    //std::cout << noOfPoints<<" points. KdTree size "<<kdTree.size()<<".\n";
    
    bool doTreeOptimization = false;
    if (doTreeOptimization && kdTree.size()<lastOptimizedKdTreeSize/2) {
      kdTree.optimize();
      lastOptimizedKdTreeSize = kdTree.size();
      //std::cout << "Reoptimizing at size "<<kdTree.size()<<"\n";
    }
    //std::cout << "Erase "<<(pointToRemove.point-&points[0])/Dim
              //<< " with neighbor "<<(pointToRemove.neighbor-&points[0])/Dim
              //<< " from DistanceSet.\n";
    std::pair<NeighborIterator,NeighborIterator> range = pointsSortedByNeighbor.equal_range(pointToRemove);
    for (NeighborIterator it=range.first; it!=range.second; ++it) {
      if (it->point!=pointToRemove.point)
        continue;
      //std::cout << "Erase "<<(it->point-&points[0])/Dim
                //<< " with neighbor "<<(it->neighbor-&points[0])/Dim
                //<< " from NeighborSet.\n";
      pointsSortedByNeighbor.erase(it);
      break;
    }
    
    std::swap(pointToRemove.point, pointToRemove.neighbor);
    range = pointsSortedByNeighbor.equal_range(pointToRemove);
    for (NeighborIterator it=range.first; it!=range.second;) {
      PointWithNeighbor<Real> pointToRecalculate = *it;
      NeighborIterator tmpIt = it;
      ++it;
      pointsSortedByNeighbor.erase(tmpIt);
      //std::cout << "Need to update "<<(pointToRecalculate.point-&points[0])/Dim
                //<< " with neighbor "<<(pointToRecalculate.neighbor-&points[0])/Dim
                //<< " with distance "<<pointToRecalculate.distance<<".\n";
      std::pair<DistanceIterator,DistanceIterator> range2 = pointsSortedByDistance.equal_range(pointToRecalculate);
      for (DistanceIterator it2=range2.first; it2!=range2.second; ++it2) {
        if (it2->point!=pointToRecalculate.point)
          continue;
        //std::cout << "Erase "<<(it2->point-&points[0])/Dim << " from DistanceSet.\n";
        pointsSortedByDistance.erase(it2);
        break;
      }
      KdTreePoint<Real> kdTreePoint(pointToRecalculate.point);
      std::pair<typename TreeType::const_iterator, typename TreeType::distance_type> newNeighbor =
          kdTree.find_nearest_if(kdTreePoint, 1e10, NotSame<Real>(kdTreePoint));
      recalculatedPoints.push_back(PointWithNeighbor<Real>(pointToRecalculate.point, newNeighbor.first->point, newNeighbor.second));
    }
    
    for (size_t remainingNoOfPointsIdx=0; remainingNoOfPointsIdx<recalculatedPoints.size(); ++remainingNoOfPointsIdx) {
      const PointWithNeighbor<Real>& recalculatedPoint = recalculatedPoints[remainingNoOfPointsIdx];
      pointsSortedByDistance.insert(recalculatedPoint);
      pointsSortedByNeighbor.insert(recalculatedPoint);
      //std::cout << "Readding "<<(recalculatedPoint.point-&points[0])/Dim
                //<< " with new neighbor "<<(recalculatedPoint.neighbor-&points[0])/Dim
                //<< " and distance "<<recalculatedPoint.distance<<".\n";
    }
  }
  
  for (DistanceIterator it=pointsSortedByDistance.begin(); it!=pointsSortedByDistance.end(); ++it)
    indices.push_back((it->point-&points[0])/Dim);

  //double time5 = Tmp::getTime();
  
  //std::cout << "Times: "<<time2-time1<<"s, "<<time3-time2<<", "<<time4-time3<<", "<<time5-time4<<"s.\n";
}
