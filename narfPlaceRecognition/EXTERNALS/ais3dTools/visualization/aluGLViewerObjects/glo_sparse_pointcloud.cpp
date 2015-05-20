#include "glo_sparse_pointcloud.h"

Ais3dTools::Vis::SparsePointCloud::SparsePointCloud(const Ais3dTools::SparsePointCloud* pointCloud) :
  ALUGLViewerObject(0.7f, 0.7f, 0.0f), _pointCloud(pointCloud), _apply_cloud_trans(false)
{
  setPointSize();
  _transformation = Eigen::Isometry3f::Identity();
}

void Ais3dTools::Vis::SparsePointCloud::draw() const
{
  if (_pointCloud==NULL)
    return;
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glDisable(GL_LIGHTING);
  glPushMatrix();
  glColor3f(draw_color_r_, draw_color_g_, draw_color_b_);
  glPointSize(_pointSize);
  if(_apply_cloud_trans) {
    glMatrixMode(GL_MODELVIEW);
    glMultMatrixf(_transformation.data());
  }
  
  glBegin(GL_POINTS);
  
  for (Ais3dTools::SparsePointCloud::const_iterator it=_pointCloud->begin(); !it.atEnd(); ++it) {
    //std::cout << it.x() <<","<<it.y()<<","<<it.z()<<"\n";
    glVertex3f(it.x(), it.y(), it.z());
  }
  glEnd();
  
  glPopMatrix();
  glPopAttrib();
}
