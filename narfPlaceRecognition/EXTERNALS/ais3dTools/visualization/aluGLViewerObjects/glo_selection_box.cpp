#include "glo_selection_box.h"
#include "visualization/aluGLViewer/alu_glviewer.h"
#include <algorithm>
#include <vector>
#include <QMouseEvent>
#include <QKeyEvent>

namespace Ais3dTools {

  GLOSelectionBox::GLOSelectionBox() :
    ALUGLViewerObject(), width_(10.0), length_(10.0), height_(10.0)
  {
    pose_ = (Eigen::Affine3f) Eigen::Translation3f(-5.0, -5.0, -5.0);
  }

  GLOSelectionBox::GLOSelectionBox(QWidget*& widget) :
    ALUGLViewerObject(), width_(10.0), length_(10.0), height_(10.0)
  {
    (void)widget; // No warning
    pose_ = (Eigen::Affine3f) Eigen::Translation3f(-5.0, -5.0, -5.0);
  }

  GLOSelectionBox::~GLOSelectionBox(){
//     cerr << "GLOSelectionBox::~GLOSelectionBox() not yet implemented" << endl;
  }

  void GLOSelectionBox::draw() const{
    glPushMatrix();
    GLboolean hasLight = glIsEnabled(GL_LIGHTING);
    if (hasLight)
      glDisable(GL_LIGHTING);

      glTranslatef(pose_.translation()(0), pose_.translation()(1), pose_.translation()(2) );
      glScalef(width_, length_, height_);
      glColor3f(1.0, 1.0, 1.0);
      drawBox();

    if (hasLight)
      glEnable(GL_LIGHTING);
    glPopMatrix();
  }


  void GLOSelectionBox::drawBox() const{
      glBegin(GL_LINE_STRIP);
        // bottom
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(1.0, 0.0, 0.0);
        glVertex3f(1.0, 1.0, 0.0);
        glVertex3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
  
        // top
        glVertex3f(0.0, 0.0, 1.0);
        glVertex3f(1.0, 0.0, 1.0);
        glVertex3f(1.0, 1.0, 1.0);
        glVertex3f(0.0, 1.0, 1.0);
        glVertex3f(0.0, 0.0, 1.0);
      glEnd();

    glBegin(GL_LINES);
       glVertex3f(1.0, 0.0, 0.0);
       glVertex3f(1.0, 0.0, 1.0);
       glVertex3f(1.0, 1.0, 0.0);
       glVertex3f(1.0, 1.0, 1.0);
       glVertex3f(0.0, 1.0, 0.0);
       glVertex3f(0.0, 1.0, 1.0);
    glEnd();

  glPushMatrix();
    glColor3f(0.0f, 0.0f, 0.99f);
    glPointSize(5.0);
    glBegin(GL_POINTS);
      glVertex3f(0.0, 0.0, 0.0);
    glEnd();
  glPopMatrix();
  }

 bool GLOSelectionBox::eventFilter(QObject* o, QEvent* e)
  {
    bool relevantEvent = false;
    ALUGLViewer* viewer = 0;
    ALUGLViewer* v = static_cast<ALUGLViewer*>(o);
    for(vector<ALUGLViewer* >::iterator it = viewer_.begin(); it != viewer_.end(); ++it){
      if (v == (*it))
      {
        viewer = (*it);
        relevantEvent = true;
      }
    }

    if(!relevantEvent || !viewer)
      return false;

    if (e->type() == QEvent::MouseButtonPress) {
      QMouseEvent* m = static_cast<QMouseEvent*>(e);

//       if ((m->modifiers() & Qt::ShiftModifier) == 0 && m->buttons() & Qt::LeftButton && m->button() & Qt::RightButton) {
//         QPoint p (m->x(), m->y());
//          cout << m->x() << " " << m->y() << endl;
//         for(vector<ALUGLViewer* >::const_iterator it = viewer_.begin(); it != viewer_.end(); ++it){
//           (*it)->camera()->setSceneCenterFromPixel(p);
//         }
//         return true;
//       }

      if ((m->modifiers() & Qt::ShiftModifier) > 0 && m->button() & Qt::LeftButton) {
        double wx, wy, wz;
        if (viewer->getPoseFromGl(m->x(), m->y(), wx, wy, wz)) {
          //Eigen::Translation3f currentPose = (Eigen::Translation3f) pose_.translation();
          if(wx == 0.0)
            wx = pose_.translation()(0);
          if(wy == 0.0)
            wy = pose_.translation()(1);
          if(wz == 0.0)
            wz = pose_.translation()(2);
          pose_ = (Eigen::Affine3f) Eigen::Translation3f(wx, wy, wz);
        }
        updateGLViewers();
        return true;
      }
  
      if ((m->modifiers() & Qt::ShiftModifier) > 0 && m->button() & Qt::RightButton) {
        double wx, wy, wz;
        if (viewer->getPoseFromGl(m->x(), m->y(), wx, wy, wz)) {
          Eigen::Translation3f currentPose = (Eigen::Translation3f) pose_.translation();
          if(wx == 0.0){
            length_ = fabs(wy - currentPose.y());
            height_ = fabs(wz - currentPose.z());
          }
          if(wy == 0.0){
            width_ = fabs(wx - currentPose.x());
            height_ = fabs(wz - currentPose.z());
          }
          if(wz == 0.0){
            width_ = fabs(wx - currentPose.x());
            length_ = fabs(wy - currentPose.y());
          }
        }
        updateGLViewers();
        return true;
      }
  
    } else if (e->type() == QEvent::KeyPress) {
      QKeyEvent* k = static_cast<QKeyEvent*>(e);
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_T) {
  
      }
      else if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_S) {
  
      }
    }

    updateGLViewers();
    // we do not handle anything specific, the widget may still receive the events
    return false;
  }

  void GLOSelectionBox::readConfiguration(std::istream& in)
  {
    string head;
    in >> head;
    if(!strcmp(head.c_str(), "Ais3dTools::GLOSelectionBox")){
      Eigen::Matrix4f matrix;
      string head2;
      in >> head2;
      if(!strcmp(head2.c_str(), "Affine3f")){
        for(int i = 0; i < 4; ++i)
          for(int j = 0; j < 4; ++j)
            in >> matrix(i,j);
        pose_ = Eigen::Affine3f(matrix);
      }
      string type;
      in >> type >> width_ >> length_ >> height_;
    }
  }

  void GLOSelectionBox::writeConfiguration(std::ostream& out)
  {
    out << "Ais3dTools::GLOSelectionBox ";
    Eigen::Matrix4f matrix = pose_.matrix();
    out << "Affine3f ";
    for(int i = 0; i < 4; ++i)
      for(int j = 0; j < 4; ++j)
        out << matrix(i,j) << " ";
    out << std::endl;
    out << "w/l/h " << width_ << " " << length_ << " " << height_<< std::endl;
  }

}
