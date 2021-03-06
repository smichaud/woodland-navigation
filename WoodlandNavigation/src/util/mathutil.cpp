#include "mathutil.h"
#include <math.h>

namespace mathutil{

Vector3uli convertToIndex(Vector3 vector) {
    if(vector[0] < 0 || vector[1] < 0 || vector[2] < 0){
        throw std::invalid_argument(
                    "Invalid conversion from negative float -> uli)");
    }

    return vector.cast<uli>();
}

Vector3 floorVector(Vector3 vector) {
    Vector3 floorVector;

    floorVector << static_cast<used_type>(floor(vector[0])),
            static_cast<used_type>(floor(vector[1])),
            static_cast<used_type>(floor(vector[2]));

    return floorVector;
}

uli getRoundedQuantileRelatedIndex(
        std::vector<indexAndValue> indexesAndValues, float quantile) {
    if(quantile < 0 || quantile > 1) {
        throw std::invalid_argument("Quantile must be between 0 and 1");
    } else if (indexesAndValues.size() == 0) {
        throw empty_container();
    }

    int quantileIndex = static_cast<int>(
                round(static_cast<double>(indexesAndValues.size()-1)
                      *quantile));

    std::nth_element (indexesAndValues.begin(),
                      indexesAndValues.begin()+quantileIndex,
                      indexesAndValues.end(), compareValues);

    std::vector<indexAndValue>::iterator it=
            (indexesAndValues.begin()+quantileIndex);

    return it->first;
}

const char *empty_container::what() const throw(){
    return "Unable to find quantile for empty container";
}

bool compareValues(indexAndValue i,indexAndValue j) {
    return (i.second < j.second);
}

}
