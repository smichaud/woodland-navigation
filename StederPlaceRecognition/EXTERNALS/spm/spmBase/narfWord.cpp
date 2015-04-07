#include "narfWord.h"
// #include "data_structs/pointCloud/pointCloud.h"

void NarfWord::writeBinary(std::ostream &file) const {
    file.write((char*)&id, sizeof(id));
    feature.saveBinary(file);

    int numPoints = pointCloud.size();
    file.write((char*)&numPoints, sizeof(numPoints));
    for(int i = 0; i < numPoints; ++i){
      file.write((char*)&pointCloud.points[i].x, sizeof(pointCloud.points[i].x));
      file.write((char*)&pointCloud.points[i].y, sizeof(pointCloud.points[i].y));
      file.write((char*)&pointCloud.points[i].z, sizeof(pointCloud.points[i].z));
    }
}
 
void NarfWord::readBinary(std::istream &file){
    file.read((char*)&id, sizeof(id));
    feature.loadBinary(file);
    //compatibility to old data
    int numPoints = 0;
    file.read((char*)&numPoints, sizeof(numPoints));
    for(int i = 0; i < numPoints; ++i){
      float x,y,z;
      file.read((char*)&x, sizeof(x));
      file.read((char*)&y, sizeof(y));
      file.read((char*)&z, sizeof(z));
      pointCloud.push_back(NWPoint(x,y,z));
    }
}

