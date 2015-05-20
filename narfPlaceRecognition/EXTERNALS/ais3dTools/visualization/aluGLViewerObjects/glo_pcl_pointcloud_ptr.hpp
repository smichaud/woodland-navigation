#include "pcl/common/transforms.h"
#include "basics/macros.h"
#include "visualization/imageWidget/imageWidget.h"

namespace Ais3dTools {

template <typename PointCloudType>
PclPointCloudPtrObjectT<PointCloudType>::PclPointCloudPtrObjectT(const PointCloudPtr& pointCloud) :
 ALUGLViewerObject(0.0f, 0.0f, 0.8f), _pointCloud(pointCloud), _anaglyph(false), _anag_blue_red(false)
{
  setPointSize();
}

template <typename PointCloudType>
PclPointCloudPtrObjectT<PointCloudType>::PclPointCloudPtrObjectT(PointCloudConstPtr& pointCloud) :
  ALUGLViewerObject(), _pointCloud(pointCloud), _anaglyph(false), _anag_blue_red(false)
{
  setDrawColor(0.0f, 0.0f, 0.8f);
  setPointSize();
}

template <typename PointCloudType>
PclPointCloudPtrObjectT<PointCloudType>::~PclPointCloudPtrObjectT() {
}

template <typename PointCloudType>
void PclPointCloudPtrObjectT<PointCloudType>::drawPoints() const
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
inline void PclPointCloudPtrObjectT< pcl::PointCloud<pcl::PointXYZRGBNormal> >::drawPoints() const
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
inline void PclPointCloudPtrObjectT< pcl::PointCloud<pcl::PointXYZRGB> >::drawPoints() const
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
inline void PclPointCloudPtrObjectT< pcl::PointCloud<pcl::PointXYZRGBA> >::drawPoints() const
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
inline void PclPointCloudPtrObjectT< pcl::PointCloud<pcl::PointXYZI> >::drawPoints() const
{
  //glBegin(GL_POINTS);
  //for (size_t i = 0; i < _pointCloud->points.size (); ++i){
    //if(isnan(_pointCloud->points[i].x) || isnan(_pointCloud->points[i].y) || isnan(_pointCloud->points[i].z))
      //continue;
////     if(_pointCloud->points[i].intensity > 0.55){
////       glColor3f(0.0f, 0.4f, 0.0f);
////     } else {
      //glColor3f(_pointCloud->points[i].intensity, _pointCloud->points[i].intensity, _pointCloud->points[i].intensity);
////     }
    //glVertex3f(_pointCloud->points[i].x, _pointCloud->points[i].y, _pointCloud->points[i].z);
  //}
  //glEnd();
  
  if (_pointCloud == NULL || _pointCloud->empty())
    return;
  
  //float intensityNormalization = 1.0f;
  std::vector<float> intensities, sparseIntensities;
  for (size_t i = 0; i < _pointCloud->points.size (); ++i)
    if (isfinite(_pointCloud->points[i].x) && isfinite(_pointCloud->points[i].y) &&isfinite(_pointCloud->points[i].z))
      intensities.push_back(_pointCloud->points[i].intensity);
  float normalization = 1.0f;
  if (!intensities.empty()) {
    std::sort(intensities.begin(), intensities.end());
    //intensityNormalization = 1.0f/intensities[lrint(0.9f*intensities.size())];
    for (int percentage=0; percentage<=100; ++percentage) {
      sparseIntensities.push_back(intensities[lrint(0.01f*percentage*(int(intensities.size())-1))]);
      if (sparseIntensities.size()>=2 && sparseIntensities.back()==sparseIntensities[sparseIntensities.size()-2])
        sparseIntensities.pop_back();
      else
        std::cout << PVARC(percentage)<<sparseIntensities.back()<<"\n";
    }
    if (sparseIntensities.size()>=2)
      normalization = 1.0f/(sparseIntensities.size()-1);
  }
  //std::cout << PVARN(intensityNormalization);
  
  //float start   = intensities[0],
        //factor  = 1.0f / (intensities.back() - intensities[0]);
  float start   = intensities[lrint(0.02*intensities.size())],
        end     = intensities[lrint(0.98*intensities.size())],
        factor  = 1.0f / (end - start);
  
  bool scaleFromMinToMax = true;
  
  glBegin(GL_POINTS);
  for (size_t i = 0; i < _pointCloud->points.size (); ++i){
    Eigen::Vector3f p = _pointCloud->points[i].getVector3fMap();
    float intensity = _pointCloud->points[i].intensity;
    if (!isfinite(p.x()) || !isfinite(p.y()) ||!isfinite(p.z()))
      continue;
    float value = 0.0f;
    if (scaleFromMinToMax)
      value = std::max(0.0f, std::min(1.0f, (intensity - start)*factor));
    else {
      std::vector<float>::iterator lowerBoundIt = std::lower_bound(sparseIntensities.begin(), sparseIntensities.end(), intensity);
      if (lowerBoundIt != sparseIntensities.end())
        value = normalization*((lowerBoundIt-sparseIntensities.begin())+(intensity-*lowerBoundIt)/(*(lowerBoundIt+1)-*lowerBoundIt));
      else
        std::cout << ":-(\n";
    }
    unsigned char r, g, b;
    //std::cout << PVARC(intensity)<<PVARN(value);
    bool useGrayscale = true;
    ImageWidget::getColorForReal(value, r, g, b, useGrayscale);
    glColor3ub(r,g,b);
    glVertex3f(p.x(), p.y(), p.z());
  }
  glEnd();
}

template< >
inline void PclPointCloudPtrObjectT< pcl::PointCloud<pcl::PointXYZINormal> >::drawPoints() const
{
  glBegin(GL_POINTS);
  for (size_t i = 0; i < _pointCloud->points.size (); ++i){
    if(isnan(_pointCloud->points[i].x) || isnan(_pointCloud->points[i].y) || isnan(_pointCloud->points[i].z))
      continue;
    //     if(_pointCloud->points[i].intensity > 0.55){
      //       glColor3f(0.0f, 0.4f, 0.0f);
      //     } else {
        glColor3f(_pointCloud->points[i].intensity, _pointCloud->points[i].intensity, _pointCloud->points[i].intensity);
        //     }
        glVertex3f(_pointCloud->points[i].x, _pointCloud->points[i].y, _pointCloud->points[i].z);
  }
  glEnd();

//   double length = 0.02;
//   int stepSize = 30;
//   glColor3f(0.0, 0.0, 0.0);
//   for (size_t i = 0; i < _pointCloud->points.size (); i+=stepSize){
//     if(isnan(_pointCloud->points[i].x) || isnan(_pointCloud->points[i].y) || isnan(_pointCloud->points[i].z) || _pointCloud->points[i].normal[0] == 0.0f)
//       continue;
//     glPushMatrix();
//     glTranslatef(_pointCloud->points[i].x, _pointCloud->points[i].y, _pointCloud->points[i].z);
//     glBegin(GL_LINES);
//     glVertex3f(0.0, 0.0, 0.0);
//     glVertex3f(length * _pointCloud->points[i].normal[0], length * _pointCloud->points[i].normal[1], length * _pointCloud->points[i].normal[2]); //_pointCloud->points[i].x, _pointCloud->points[i].y, _pointCloud->points[i].z); //
//     glEnd();
//     glPopMatrix();
//   }

}

template <typename PointCloudType>
void PclPointCloudPtrObjectT<PointCloudType>::draw() const
{
  if (!_pointCloud){
    std::cerr << __PRETTY_FUNCTION__ << "no valid point cloud..." << std::endl;
    return;
  }
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
