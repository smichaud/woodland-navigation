#include "glo_empty_box.h"

namespace Ais3dTools {

  EmptyBoxObject::EmptyBoxObject() :
  ALUGLViewerObject(), _lineWidth(1.0), _size(1.0), _pose(Eigen::Isometry3f::Identity())
  {
    setDrawColor(1.0f, 1.0f, 1.0f);
  }

  EmptyBoxObject::EmptyBoxObject(const EmptyBoxObject& other) :
  ALUGLViewerObject(), _lineWidth(other._lineWidth), _size(other._size), _pose(other._pose)
  {
    setDrawColor(1.0f, 1.0f, 1.0f);
  }

  EmptyBoxObject::EmptyBoxObject(const Eigen::Isometry3f& pose, float size) :
    ALUGLViewerObject(), _size(size), _pose(pose)
  {
    setDrawColor(1.0f, 1.0f, 1.0f);
  }

  EmptyBoxObject::~EmptyBoxObject() {
  }


  void EmptyBoxObject::draw() const
  {
    glPushMatrix();
    GLboolean hasLight = glIsEnabled(GL_LIGHTING);
    if (hasLight)
      glDisable(GL_LIGHTING);

    glColor3f(draw_color_r_, draw_color_g_, draw_color_b_);
    glMatrixMode(GL_MODELVIEW);
    glMultMatrixf(_pose.data());
    glScalef(_size, _size, _size);
    glLineWidth(_lineWidth);
    
    glBegin(GL_LINE_STRIP);
      glVertex3f(-0.5f, -0.5f, -0.5f);
      glVertex3f(0.5f, -0.5f, -0.5f);
      glVertex3f(0.5f, 0.5f, -0.5f);
      glVertex3f(-0.5f, 0.5f, -0.5f);
      glVertex3f(-0.5f, -0.5f, -0.5f);

      glVertex3f(-0.5f, -0.5f, 0.5f);
      glVertex3f(0.5f, -0.5f, 0.5f);
      glVertex3f(0.5f, 0.5f, 0.5f);
      glVertex3f(-0.5f, 0.5f, 0.5f);
      glVertex3f(-0.5f, -0.5f, 0.5f);
    glEnd();

    glBegin(GL_LINES);
      glVertex3f(0.5f, -0.5f, -0.5f);
      glVertex3f(0.5f, -0.5f, 0.5f);

      glVertex3f(0.5f, 0.5f, -0.5f);
      glVertex3f(0.5f, 0.5f, 0.5f);

      glVertex3f(-0.5f, 0.5f, -0.5f);
      glVertex3f(-0.5f, 0.5f, 0.5f);
    glEnd();

    glLineWidth(1.0f);
    if (hasLight)
      glEnable(GL_LIGHTING);
    glPopMatrix();
  }
}  // Namespace end
