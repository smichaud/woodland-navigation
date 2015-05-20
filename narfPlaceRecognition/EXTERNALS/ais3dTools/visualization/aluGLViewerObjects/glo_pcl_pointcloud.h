#ifndef ALU_GL_OBJECT_PCL_POINT_CLOUD_SHARED_H
#define ALU_GL_OBJECT_PCL_POINT_CLOUD_SHARED_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"

//#pragma GCC diagnostic ignored "-Wunused-parameter -Wunknown-pragmas"  // Do not show warnings from PCL
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
//#pragma GCC diagnostic warning "-Wunused-parameter -Wunknown-pragmas"

namespace Ais3dTools {
template <typename PointCloudType>
class PclPointCloudObjectT : public ALUGLViewerObject
{
  public:
    //typedef typename PointCloudType::iterator       iterator;
    //typedef typename PointCloudType::const_iterator const_iterator;

    PclPointCloudObjectT(const PointCloudType* pointCloud=NULL);
    ~PclPointCloudObjectT();

    virtual void draw() const;
    virtual void drawPoints() const;

    void setPointCloud(const PointCloudType* pointCloud) { _pointCloud = pointCloud;}
    const PointCloudType* getPointCloud() const { return _pointCloud;}

    inline void setPointSize(float pointSize=1.0f) { _pointSize=pointSize;}
    inline void setHasColor(bool state){ _hasColor = state;}
    inline bool getHasColor(){ return _hasColor;}

    /** enable / disable anaglyph drawing*/
    inline void setAnaglyph(bool state){ _anaglyph = state;}
    /** return anaglyph draw state */
    inline bool getAnaglyph(){ return _anaglyph;}

    /**
      false: mangenta/gree
      true:  blue / red
    */
    inline void setAnaglyphMode(bool state){ _anag_blue_red = state;}
    inline bool getAnaglyphMode(){ return _anag_blue_red;}

  protected:
    const PointCloudType* _pointCloud;
    float _pointSize;
    bool _hasColor;
    bool _anaglyph;
    bool _anag_blue_red;
};

typedef PclPointCloudObjectT< pcl::PointCloud<pcl::PointXYZ> >  PclPointCloud;
typedef PclPointCloudObjectT< pcl::PointCloud<pcl::PointXYZRGB> >  PclPointCloudRGB;

}  // Namespace end

#include "glo_pcl_pointcloud.hpp"

#endif
