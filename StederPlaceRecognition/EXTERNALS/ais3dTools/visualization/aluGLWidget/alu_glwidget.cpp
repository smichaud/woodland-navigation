#include "alu_glwidget.h"

#include <QHBoxLayout>
#include <QVBoxLayout>

//#include "g2o/stuff/macros.h"
#include "visualization/aluGLViewer/alu_glviewer.h"
#include "visualization/aluGLViewer/alu_glviewer_object.h"

#include <iostream>
using namespace std;
using namespace qglviewer;

namespace Ais3dTools{

  ALUGLWidget::ALUGLWidget(QWidget* parent) : QWidget(parent)
  {
    QWidget* widget = this;
    //setting up verticalLayout
    QLayout* tmpLayout = layout();
    delete tmpLayout;
    layout_ = new QVBoxLayout();
    setLayout(layout_);
    layout_->setContentsMargins(0, 0, 0, 0);
    layout_->setObjectName(QString::fromUtf8("layout_"));


    glViewer1_ = new ALUGLViewer(widget);
    glViewer1_->setObjectName(QString::fromUtf8("glViewer1_"));
    layout_->addWidget(glViewer1_);

    glViewer2_ = new ALUGLViewer(widget, glViewer1_);

    glViewer1_->setAxisIsDrawn();
    glViewer2_->setAxisIsDrawn();

    evtFilter.watchViewer(glViewer1_);
    mouseStereEventFilter.watchViewer(glViewer1_);
    evtFilter.watchViewer(glViewer2_);
    mouseStereEventFilter.watchViewer(glViewer2_);

    glViewer1_->camera()->setPosition(Vec(0.0, 0.0, 2.0));
    glViewer1_->camera()->lookAt(glViewer1_->sceneCenter());

    glViewer2_->camera()->setPosition(Vec(0.0, 0.0, 2.0));
    glViewer2_->camera()->lookAt(glViewer1_->sceneCenter());

    initMenu();

  }

  ALUGLWidget::~ALUGLWidget()
  {
    delete layout_;
    if(glViewer1_)
      delete glViewer1_;
    if(glViewer2_)
      delete glViewer2_;
  }
  
  void ALUGLWidget::initMenu()
  {
    menuBar_ = new QMenuBar(this);
    menuBar_->setObjectName(QString::fromUtf8("menuBar_"));
    menuBar_->setGeometry(QRect(0, 0, 1024, 20));
    menuBar_->setAutoFillBackground(true);

    /** File menu */
    menuFile_ = new QMenu(menuBar_);
    menuFile_->setObjectName(QString::fromUtf8("menuFile_"));
    menuFile_->setTitle("File");
    menuBar_->addAction(menuFile_->menuAction());

    menuFile_->addAction("Exit", this, SLOT(quit()));

    menuView_ = new QMenu(menuBar_);
    menuView_->setObjectName(QString::fromUtf8("menuView_"));
    menuView_->setTitle("View");
    menuBar_->addAction(menuView_->menuAction());

    menuView_->addAction("Hide Menu", this, SLOT(hideMenu()));
    connect(&evtFilter, SIGNAL(mPressed()), this, SLOT(toggleMenu()));

    menuView_->addAction("Toggle Draw Axis", this, SLOT(toggleAxisIsDrawn()));
    connect(&evtFilter, SIGNAL(aPressed()), glViewer1_, SLOT(toggleAxisIsDrawn()));
    connect(&evtFilter, SIGNAL(aPressed()), glViewer2_, SLOT(toggleAxisIsDrawn()));

    menuView_->addAction("Set Stereo Side-by-Side", this, SLOT(setSplitStereoSideBySide()));
    menuView_->addAction("Set Stereo Top-Down", this, SLOT(setSplitStereoTopDown()));
    menuView_->addAction("Disable Stereo", this, SLOT(unsetSplitStereo()));

    menuBar_->show();
    //menuBar_->addAction(menuFile_->menuAction());
    //menuBar_->addAction(menuView_->menuAction());
    //menuFile_->addAction(actionExit);

    //QMetaObject::connectSlotsByName(this);
  }

  void
  ALUGLWidget::updateGL()
  {
    if(glViewer1_)
      glViewer1_->updateGL();
    if(glViewer2_)
      glViewer2_->updateGL();
  }

  void
  ALUGLWidget::quit()
  { 
    exit(0);
  }

  void 
  ALUGLWidget::toggleMenu()
  {
    if(menuBar_->isVisible()){
      menuBar_->hide();
    } else {
      menuBar_->show();
    }
  }

  void
  ALUGLWidget::showMenu()
  {
    menuBar_->show();
  }

  void
  ALUGLWidget::hideMenu()
  {
    menuBar_->hide();
  }

  void
  ALUGLWidget::toggleAxisIsDrawn()
  {
    if(glViewer1_)
        glViewer1_->toggleAxisIsDrawn();
    if(glViewer2_)
        glViewer2_->toggleAxisIsDrawn();
  }

  void 
  ALUGLWidget::add(ALUGLViewerObject* child)
  {
    // add point cloud vis object 
    if(glViewer1_)
        glViewer1_->add(child);
    if(glViewer2_)
        glViewer2_->add(child);
  }

  void ALUGLWidget::remove(ALUGLViewerObject* child)
  {
    // remove viewer object
    if(glViewer1_)
      glViewer1_->remove(child);
    if(glViewer2_)
      glViewer2_->remove(child);
  }

  void ALUGLWidget::setSplitStereo(bool topDown = false)
  {
    QWidget* widget = this;
    //Make shure that two viewer exists with a shared glContext
    if(!glViewer1_)
      glViewer1_ = new ALUGLViewer(widget);
    if(!glViewer2_)
      glViewer2_ = new ALUGLViewer(widget, glViewer1_);

    layout_->removeWidget(glViewer1_);
    layout_->removeWidget(glViewer2_);
    //setting new layout_ up

    QLayout* tmpLayout = layout();
    delete tmpLayout;
    if(topDown){
      layout_ = new QVBoxLayout(widget);
    } else {
      layout_ = new QHBoxLayout(widget);
    }
    setLayout(layout_);
    layout_->setContentsMargins(0, 0, 0, 0);
    layout_->setObjectName(QString::fromUtf8("layout_"));

    layout_->addWidget(glViewer1_);
    layout_->addWidget(glViewer2_);

    // Move camera according to viewer type (on X, Y or Z axis)
    glViewer2_->camera()->setPosition(glViewer1_->camera()->position());
    glViewer2_->camera()->setOrientation(glViewer1_->camera()->orientation());
    glViewer2_->show();
  }

  void ALUGLWidget::setSplitStereoTopDown(){
    setSplitStereo(true);
  }

  void ALUGLWidget::setSplitStereoSideBySide(){
    setSplitStereo(false);
  }

  void ALUGLWidget::unsetSplitStereo()
  {
    //remove second glviewer
    if(glViewer2_ && layout_){
      layout_->removeWidget(glViewer2_);
      glViewer2_->hide();
//       Ais3dTools::ALUGLViewer* tmpGLviewer = glViewer2_;
//       glViewer2_ = 0;
//       delete tmpGLviewer;
    }
  }

  void ALUGLWidget::resizeEvent ( QResizeEvent * event )
  {
    glViewer1_->resizeEvent(event);
    glViewer2_->resizeEvent(event);
    menuBar_->setGeometry(QRect(0, 0, event->size().width(), 24));
  }

} //end namespace
