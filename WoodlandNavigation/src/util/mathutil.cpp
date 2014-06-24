#include "mathutil.h"
#include <math.h>

Vector3uli MathUtil::convertToIndice(Vector3 vector) {
    if(vector[0] < 0 || vector[1] < 0 || vector[2] < 0){
        std::string msg =
                "Invalid conversion : (negative float -> unsigned long int)";
        throw std::invalid_argument(msg);
    }

    return vector.cast<unsigned long int>();
}

Vector3 MathUtil::floorVector(Vector3 vector) {
    Vector3 floorVector;

    floorVector << static_cast<used_type>(floor(vector[0])),
            static_cast<used_type>(floor(vector[1])),
            static_cast<used_type>(floor(vector[2]));

    return floorVector;
}

unsigned long int MathUtil::getRoundedQuantileRelatedIndice(
        std::vector<indexAndValue> indexesAndValues, float quantile) {
    if(quantile < 0 || quantile > 1) {
        std::string msg = "Quantile must be between 0 and 1";
        throw std::invalid_argument(msg);
    }


    int quantileIndice = static_cast<int>(
                round(static_cast<double>(indexesAndValues.size()-1)
                      *quantile));

    std::nth_element (indexesAndValues.begin(),
                      indexesAndValues.begin()+quantileIndice,
                      indexesAndValues.end(), MathUtil::compareValues);

    std::vector<indexAndValue>::iterator it=
            (indexesAndValues.begin()+quantileIndice);

    return it->first;
}

bool MathUtil::compareValues(indexAndValue i,indexAndValue j) {
    return (i.second < j.second);
}


