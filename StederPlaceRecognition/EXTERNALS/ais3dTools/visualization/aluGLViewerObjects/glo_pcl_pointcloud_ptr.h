#ifndef ALU_GL_OBJECT_PCL_POINT_CLOUD_PTR_SHARED_H
#define ALU_GL_OBJECT_PCL_POINT_CLOUD_PTR_SHARED_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"

//#pragma GCC diagnostic ignored "-Wunused-parameter -Wunknown-pragmas"  // Do not show warnings from PCL
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
//#pragma GCC diagnostic warning "-Wunused-parameter -Wunknown-pragmas"

namespace Ais3dTools {
template <typename PointCloudType>
class PclPointCloudPtrObjectT : public ALUGLViewerObject
{
  public:
    //typedef typename PointCloudType::iterator       iterator;
    //typedef typename PointCloudType::const_iterator const_iterator;

    typedef typename PointCloudType::Ptr PointCloudPtr;
    typedef typename PointCloudType::ConstPtr PointCloudConstPtr;

    PclPointCloudPtrObjectT(PointCloudConstPtr& pointCloud);
    PclPointCloudPtrObjectT(const PointCloudPtr& pointCloud);
    ~PclPointCloudPtrObjectT();

    virtual void draw() const;
    virtual void drawPoints() const;

    void setPointCloud(PointCloudConstPtr& pointCloud) { _pointCloud = pointCloud;}
    void setPointCloud(const PointCloudPtr& pointCloud) { _pointCloud = pointCloud;};
    PointCloudConstPtr& getPointCloud() const { return _pointCloud;}

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

  public slots:
    void increasePointSize() { _pointSize +=1.0f;};
    void decreasePointSize() { _pointSize -=1.0f;};

  protected:
    PointCloudConstPtr  _pointCloud;
    float               _pointSize;
    bool                _hasColor;
    bool                _anaglyph;
    bool                _anag_blue_red;
};

typedef PclPointCloudPtrObjectT< pcl::PointCloud<pcl::PointXYZ> >  PclPointCloudPtr;
typedef PclPointCloudPtrObjectT< pcl::PointCloud<pcl::PointXYZRGB> >  PclPointCloudPtrRGB;

}  // Namespace end

#include "glo_pcl_pointcloud_ptr.hpp"

#endif
