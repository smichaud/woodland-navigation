#ifndef ALU_GL_OBJECT_SPARSE_POINT_CLOUD_H
#define ALU_GL_OBJECT_SPARSE_POINT_CLOUD_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"
#include "basics/sparsePointCloud.h"
#include <Eigen/Geometry>

namespace Ais3dTools {
namespace Vis {
class SparsePointCloud : public ALUGLViewerObject
{
  public:
    SparsePointCloud(const Ais3dTools::SparsePointCloud* sparsePointCloud=NULL);
    
    virtual void draw() const;
    
    void setPointCloud(const Ais3dTools::SparsePointCloud* pointCloud) { _pointCloud = pointCloud;}
    const Ais3dTools::SparsePointCloud* getPointCloud() const { return _pointCloud;}
    
    inline void setPointSize(float pointSize=1.0f) { _pointSize=pointSize;}
    inline void setUseTransformation(bool state){ _apply_cloud_trans = state;}
    inline bool getUseTransformation(){ return _apply_cloud_trans;}

    inline void setTransformation(const Eigen::Isometry3f& trans){ _transformation = trans; _apply_cloud_trans = true;}
    inline Eigen::Affine3f& getTransformation(){ return _transformation;}

  protected:
    const Ais3dTools::SparsePointCloud* _pointCloud;
    float _pointSize;
    bool _apply_cloud_trans;
    Eigen::Affine3f  _transformation;
};

}  // Namespace end
}  // Namespace end

#endif
