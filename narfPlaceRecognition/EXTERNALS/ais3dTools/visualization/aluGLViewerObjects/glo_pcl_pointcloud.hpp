#include "pcl/common/transforms.h"
#include "basics/macros.h"

namespace Ais3dTools {

template <typename PointCloudType>
PclPointCloudObjectT<PointCloudType>::PclPointCloudObjectT(const PointCloudType* pointCloud) :
  ALUGLViewerObject(0.0f, 0.0f, 0.8f), _pointCloud(pointCloud), _anaglyph(false), _anag_blue_red(false)
{
  setPointSize();
}

template <typename PointCloudType>
PclPointCloudObjectT<PointCloudType>::~PclPointCloudObjectT() {
}

template <typename PointCloudType>
void PclPointCloudObjectT<PointCloudType>::drawPoints() const
{
  glBegin(GL_POINTS);
  for (size_t i = 0; i < _pointCloud->points.size (); ++i){
    if(isnan(_pointCloud->points[i].x) || isnan(_pointCloud->points[i].y) || isnan(_pointCloud->points[i].z))
      continue;
    glVertex3f(_pointCloud->points[i].x, _pointCloud->points[i].y, _pointCloud->points[i].z);
  }
  glEnd();
}

template< >
inline void PclPointCloudObjectT< pcl::PointCloud<pcl::PointXYZRGBNormal> >::drawPoints() const
{
  glBegin(GL_POINTS);
  for (size_t i = 0; i < _pointCloud->points.size (); ++i){
    if(isnan(_pointCloud->points[i].x) || isnan(_pointCloud->points[i].y) || isnan(_pointCloud->points[i].z))
      continue;
    glColor3ub(_pointCloud->points[i].r, _pointCloud->points[i].g, _pointCloud->points[i].b);
    glVertex3f(_pointCloud->points[i].x, _pointCloud->points[i].y, _pointCloud->points[i].z);
  }
  glEnd();

  double length = 0.02;
  int stepSize = 30;
  glColor3f(0.0, 0.0, 0.0);
    for (size_t i = 0; i < _pointCloud->points.size (); i+=stepSize){
      if(isnan(_pointCloud->points[i].x) || isnan(_pointCloud->points[i].y) || isnan(_pointCloud->points[i].z) || _pointCloud->points[i].normal[0] == 0.0f)
        continue;
      glPushMatrix();
        glTranslatef(_pointCloud->points[i].x, _pointCloud->points[i].y, _pointCloud->points[i].z);
        glBegin(GL_LINES);
          glVertex3f(0.0, 0.0, 0.0);
          glVertex3f(length * _pointCloud->points[i].normal[0], length * _pointCloud->points[i].normal[1], length * _pointCloud->points[i].normal[2]); //_pointCloud->points[i].x, _pointCloud->points[i].y, _pointCloud->points[i].z); //
        glEnd();
      glPopMatrix();
    }

}

template< >
inline void PclPointCloudObjectT< pcl::PointCloud<pcl::PointXYZRGB> >::drawPoints() const
{
  glBegin(GL_POINTS);
  for (size_t i = 0; i < _pointCloud->points.size (); ++i){
    if(isnan(_pointCloud->points[i].x) || isnan(_pointCloud->points[i].y) || isnan(_pointCloud->points[i].z))
      continue;
    glColor3ub(_pointCloud->points[i].r, _pointCloud->points[i].g, _pointCloud->points[i].b);
    glVertex3f(_pointCloud->points[i].x, _pointCloud->points[i].y, _pointCloud->points[i].z);
  }
  glEnd();

}

template< >
inline void PclPointCloudObjectT< pcl::PointCloud<pcl::PointXYZRGBA> >::drawPoints() const
{
  glBegin(GL_POINTS);
  for (size_t i = 0; i < _pointCloud->points.size (); ++i){
    if(isnan(_pointCloud->points[i].x) || isnan(_pointCloud->points[i].y) || isnan(_pointCloud->points[i].z))
      continue;
    glColor3ub(_pointCloud->points[i].r, _pointCloud->points[i].g, _pointCloud->points[i].b);
    glVertex3f(_pointCloud->points[i].x, _pointCloud->points[i].y, _pointCloud->points[i].z);
  }
  glEnd();

}

template< >
inline void PclPointCloudObjectT< pcl::PointCloud<pcl::PointXYZI> >::drawPoints() const
{
  glBegin(GL_POINTS);
  for (size_t i = 0; i < _pointCloud->points.size (); ++i){
    if(isnan(_pointCloud->points[i].x) || isnan(_pointCloud->points[i].y) || isnan(_pointCloud->points[i].z))
      continue;
    if(_pointCloud->points[i].intensity > 0.55){
      glColor3f(0.0f, 0.4f, 0.0f);
    } else {
      glColor3f(_pointCloud->points[i].intensity, _pointCloud->points[i].intensity, _pointCloud->points[i].intensity);
    }
    glVertex3f(_pointCloud->points[i].x, _pointCloud->points[i].y, _pointCloud->points[i].z);
  }
  glEnd();
}

template <typename PointCloudType>
void PclPointCloudObjectT<PointCloudType>::draw() const
{
  if (!_pointCloud)
    return;
  glPushMatrix();
  GLboolean hasLight = glIsEnabled(GL_LIGHTING);
  if (hasLight)
    glDisable(GL_LIGHTING);
  glColor3f(draw_color_r_, draw_color_g_, draw_color_b_);
  glPointSize(_pointSize);

    drawPoints();

  glPointSize(1.0f);
  if (hasLight)
    glEnable(GL_LIGHTING);
  glPopMatrix();
}


}  // Namespace end
