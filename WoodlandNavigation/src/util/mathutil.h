#ifndef MATHUTIL_H
#define MATHUTIL_H

#include "definitions.h"

class MathUtil
{
public:
    static Vector3uli convertToIndice(Vector3 vector);
    static Vector3 floorVector(Vector3 vector);

    // Quantile must be between [0,1]
    static unsigned long int getRoundedQuantileRelatedIndice(
            std::vector<indexAndValue> indexesAndValues, float quantile);

private:
    static bool compareValues(indexAndValue i,indexAndValue j);
};

#endif
