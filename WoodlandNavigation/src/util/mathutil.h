#ifndef MATHUTIL_H
#define MATHUTIL_H

#include "definitions.h"
#include <exception>
#include <iostream>

namespace mathutil {
    Vector3uli convertToIndex(Vector3 vector);
    Vector3 floorVector(Vector3 vector);

    uli getRoundedQuantileRelatedIndex(
            std::vector<indexAndValue> indexesAndValues, float quantile);
    class empty_container: public std::exception {
        virtual const char* what() const throw();
    };
    bool compareValues(indexAndValue i,indexAndValue j);
}

#endif
