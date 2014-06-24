#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "pointmatcher/PointMatcher.h"
#include <Eigen/Core>

typedef float used_type;

typedef PointMatcher<used_type> PM;
typedef Eigen::Matrix<used_type, 3, 1> Vector3;
typedef Eigen::Matrix<unsigned long int,3,1> Vector3uli;

typedef std::pair<unsigned long int, used_type> indexAndValue;

const int xIndice = 0;
const int yIndice = 1;
const int zIndice = 2;

#endif
