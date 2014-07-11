#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "pointmatcher/PointMatcher.h"
#include <Eigen/Core>

typedef float used_type;
typedef unsigned long int uli;


typedef PointMatcher<used_type> PM;

typedef PM::Matrix Matrix;

typedef Eigen::Matrix<used_type, Eigen::Dynamic, 1> VectorX;
typedef Eigen::Matrix<used_type, 3, 1> Vector3;
typedef Eigen::Matrix<used_type, 1, 1> Scalar;

typedef Eigen::Matrix<uli,3,1> Vector3uli;

typedef std::pair<uli, used_type> indexAndValue;

const int xIndex = 0;
const int yIndex = 1;
const int zIndex = 2;

#endif
