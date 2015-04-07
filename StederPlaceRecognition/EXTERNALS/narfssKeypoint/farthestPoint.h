/* \author Bastian Steder - steder@informatik.uni-freiburg.de */

#ifndef FARTHEST_POINT_H
#define FARTHEST_POINT_H

#include <vector>
#include <stdlib.h>

template <typename Real, size_t Dim>
class FarthestPoint {
  public:
    // =====CONSTRUCTOR & DESTRUCTOR=====
    
    // =====PUBLIC METHODS=====
    //! Subsample the points to the given number, trying to keep them as uniformly distributed as possible.
    static void subsample(const std::vector<Real>& points, size_t remainingNoOfPoints,
                          Real minDistance, std::vector<size_t>& indices);
    
    // =====PUBLIC MEMBER VARIABLES=====

  protected:
    // =====PROTECTED TYPEDEFS=====
    // =====PROTECTED METHODS=====
    
    // =====PROTECTED MEMBER VARIABLES=====
};

#include "farthestPoint.hpp"

#endif
