#include "util.h"

using namespace std;

void Util::appendDescriptorRGBA(PM::DataPoints &dataPoints,
                                float red,
                                float green,
                                float blue,
                                float alpha) {
    int redIndice = 0;
    int greenIndice = 1;
    int blueIndice = 2;

    if(red < 0){
        red = 0;
    } else if(red>1){
        red = 1;
    }
    if(green < 0){
        green=0;
    } else if(green>1){
        green = 1;
    }
    if(blue < 0){
        blue = 0;
    } else if(blue > 1){
        blue = 1;
    }
    if(alpha < 0){
        alpha = 0;
    } else if(alpha > 1){
        alpha = 1;
    }

    try {
        PM::Matrix colors;

        int numberOfPoints = dataPoints.features.cols();

        colors.resize(4,numberOfPoints);
        colors.row(redIndice).fill(red);
        colors.row(greenIndice).fill(green);
        colors.row(blueIndice).fill(blue);

        dataPoints.addDescriptor("color", colors);
    } catch(exception& e){
        cerr << "An error occured : " << e.what() << endl;
    }
}
