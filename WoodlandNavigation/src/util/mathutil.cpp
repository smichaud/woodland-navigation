#include "mathutil.h"
#include <math.h>

Vector3uli MathUtil::convertToIndice(Vector3 vector) {
    Vector3uli floorVector;

    if(vector[0] < 0 || vector[1] < 0 || vector[2] < 0){
        std::string msg =
                "Invalid conversion : (negative float -> unsigned long int)";
        throw std::invalid_argument(msg);
    }

    floorVector << static_cast<unsigned long int>(floor(vector[0])),
            static_cast<unsigned long int>(floor(vector[1])),
            static_cast<unsigned long int>(floor(vector[2]));

    return floorVector;
}

Vector3 MathUtil::floorVector(Vector3 vector) {
    Vector3 floorVector;

    floorVector << static_cast<used_type>(floor(vector[0])),
            static_cast<used_type>(floor(vector[1])),
            static_cast<used_type>(floor(vector[2]));

    return floorVector;
}
