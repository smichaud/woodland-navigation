#ifndef VIS_MAP_CLOUD_H
#define VIS_MAP_CLOUD_H

#include "EXTERNALS/ais3dTools/visualization/aluGLViewer/alu_glviewer_object.h"
#include <vector>
#include "narfPlaceRecognitionLib/sparsePointCloud.h"
#include <Eigen/Geometry>

namespace Vis {

class Text3D;

class MapCloud : public Ais3dTools::ALUGLViewerObject {
  public:
    enum CoordinateSystem {
      xForward_yToLeft_zUpwards,
      xToRight_yDownwards_zForward
    };
    
    typedef Ais3dTools::ALUGLViewerObject BaseClass;
    typedef std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > PoseVector;
    MapCloud(SparsePointCloud* map, const PoseVector* trajectory);
    virtual void draw() const;
    
    bool flattenMap, showCloud, showNumbers;
    void setNodeSize(float nodeSize=-1.0f) { nodeSize_ = nodeSize; }
    //! Set the font size. -1 is automatic
    void setFontSize(float fontSize=20) { fontSize_=fontSize; }
    void setWantedNoOfShownNumbers(int wantedNoOfShownNumbers=30) { wantedNoOfShownNumbers_ = wantedNoOfShownNumbers; }
    
    void resetConnectingLines() { connectingLines_.clear(); }
    void addConnectingLine(int index1, int index2) { connectingLines_.push_back(ConnectingLine(index1, index2)); }
    
    struct ConnectingLine {
      ConnectingLine(int index1, int index2) : index1(index1), index2(index2) {}
      int index1, index2;
    };

    void setCoordinateSystem(CoordinateSystem coordinateSystem=xToRight_yDownwards_zForward) { coordinateSystem_=coordinateSystem; }
    
    
  protected:
    //void updateNodeNumbers();
    
    SparsePointCloud* map_;
    const PoseVector* trajectory_;
    float nodeSize_;
    float fontSize_;
    int wantedNoOfShownNumbers_;
    std::vector<ConnectingLine> connectingLines_;
    CoordinateSystem coordinateSystem_;
};

}  // Namespace end
#endif
