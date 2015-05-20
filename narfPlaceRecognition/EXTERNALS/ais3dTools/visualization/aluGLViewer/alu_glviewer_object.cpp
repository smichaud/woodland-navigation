#include "alu_glviewer_object.h"
#include <algorithm>
#include <vector>
#include <iostream>
#include <QMouseEvent>
#include <QKeyEvent>

#include "alu_glviewer.h"


namespace Ais3dTools {

  ALUGLViewerObject::ALUGLViewerObject() :
  QObject(), parent_(0), use_draw_list_(false),
  draw_color_r_(0.0f), draw_color_g_(0.0f), draw_color_b_(0.0f),
  apply_cloud_trans_(false), transformation_(Eigen::Isometry3f::Identity()) {

  }

  ALUGLViewerObject::ALUGLViewerObject(float r, float g, float b) :
  QObject(), parent_(0), use_draw_list_(false),
  draw_color_r_(r), draw_color_g_(g), draw_color_b_(b),
  apply_cloud_trans_(false), transformation_(Eigen::Isometry3f::Identity()){

  }

  ALUGLViewerObject::ALUGLViewerObject(const ALUGLViewerObject& other) :
  QObject(), parent_(other.parent_), use_draw_list_(other.use_draw_list_),
  draw_color_r_(other.draw_color_r_), draw_color_g_(other.draw_color_g_), draw_color_b_(other.draw_color_b_),
  apply_cloud_trans_(false), transformation_(Eigen::Isometry3f::Identity()){

  }

  ALUGLViewerObject::ALUGLViewerObject(QWidget*& widget) :
  QObject(),  parent_(0), use_draw_list_(false),
  draw_color_r_(0.0f), draw_color_g_(0.0f), draw_color_b_(0.0f),
  apply_cloud_trans_(false), transformation_(Eigen::Isometry3f::Identity()){
    (void) widget;
  }

  ALUGLViewerObject::~ALUGLViewerObject(){
//     cerr << "ALUGLViewerObject::~ALUGLViewerObject() not yet implemented" << endl;
  }

  void ALUGLViewerObject::preDraw() const{
//     cerr << "ALUGLViewerObject::preDraw() not yet implemented" << endl;
  }

  void ALUGLViewerObject::draw() const{
//     cerr << "ALUGLViewerObject::draw() not yet implemented" << endl;
  }

  void ALUGLViewerObject::drawWithChildren() const{
    glPushMatrix();
    glColor3f(draw_color_r_, draw_color_g_, draw_color_b_);

    if(apply_cloud_trans_){
      glMatrixMode(GL_MODELVIEW);
      glMultMatrixf(transformation_.data());
    }

    if(!use_draw_list_){
      draw();
    } else {
      glCallList(draw_list_);
    }
    for(std::vector<ALUGLViewerObject* >::const_iterator it = children_.begin(); it != children_.end(); ++it){
      (*it)->preDraw();
      (*it)->drawWithChildren();
      (*it)->postDraw();
    }
    glPopMatrix();
  }

  void ALUGLViewerObject::postDraw() const{

  }

  void ALUGLViewerObject::drawWithNames() const{
    cerr << "ALUGLViewerObject::drawWithNames() not yet implemented" << endl;
  }

  void ALUGLViewerObject::compileDrawLists(){
    draw_list_=glGenLists(1);
    glNewList(draw_list_, GL_COMPILE);
      draw();
    glEndList();
  }

  void ALUGLViewerObject::updateDrawLists(){
    glDeleteLists(draw_list_, 1);
    compileDrawLists();
  }

  void ALUGLViewerObject::updateGLViewers() const{
      for(vector<ALUGLViewer* >::const_iterator it = viewer_.begin(); it != viewer_.end(); ++it){
        (*it)->updateGL();
      }
  }

  void ALUGLViewerObject::setDrawColor(float r, float g, float b){
    draw_color_r_ = r;
    draw_color_g_ = g;
    draw_color_b_ = b;
  }

  ALUGLViewerObject*& ALUGLViewerObject::getParent(){
    return parent_;
  }

  void ALUGLViewerObject::add(ALUGLViewerObject* child){
    children_.push_back(child);
  }

  void ALUGLViewerObject::remove(ALUGLViewerObject* child){
    std::vector<ALUGLViewerObject* >::iterator it = std::find (children_.begin(), children_.end(), child);
    children_.erase(it);
  }

  bool ALUGLViewerObject::eventFilter(QObject* o, QEvent* e)
  {
    (void) e;
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

    // we do not handle anything specific, the widget may still receive the events
    return false;
  }

  void ALUGLViewerObject::setUseDrawList(bool use){
    use_draw_list_ = use;
    if(use_draw_list_)
      compileDrawLists();
  }

  bool& ALUGLViewerObject::getUseDrawList(){
    return use_draw_list_;
  }


  void ALUGLViewerObject::readTransformation(std::istream& in){
    Eigen::Matrix4f matrix;
    string head;
    in >> head;
    if(!strcmp(head.c_str(), "Isometry3f")){
    for(int i = 0; i < 4; ++i)
      for(int j = 0; j < 4; ++j)
        in >> matrix(i,j);
      transformation_ = Eigen::Isometry3f(matrix);
    apply_cloud_trans_ = true;
    }
  }

  void ALUGLViewerObject::writeTransformation(std::ostream& out){
    Eigen::Matrix4f matrix = transformation_.matrix();
    out << "Isometry3f ";
    for(int i = 0; i < 4; ++i)
      for(int j = 0; j < 4; ++j)
        out << matrix(i,j) << " ";
    out << std::endl;
  }

}
