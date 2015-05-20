#include "alu_glwidget_stereo_event_filter.h"

#include <QMouseEvent>
#include <QKeyEvent>

//#include "g2o/stuff/macros.h"
#include "visualization/aluGLViewer/alu_glviewer.h"

#include <iostream>
using namespace std;

namespace Ais3dTools {

ALUGLWidgetStereoEventFilter::ALUGLWidgetStereoEventFilter() : QObject(),
  hasNewRobotPose(false), repositioning(0), hasNewTargetPose(false), quit(false),
  leftMouseMove(0), rightMouseMove(0), eyeDistance(0.04)
{
  robotPose[0] = 0.;
  robotPose[1] = 0.;
  robotPose[2] = 0.;
  targetPose[0] = 0.;
  targetPose[1] = 0.;
}

ALUGLWidgetStereoEventFilter::~ALUGLWidgetStereoEventFilter()
{
}

bool ALUGLWidgetStereoEventFilter::eventFilter(QObject* o, QEvent* e)
{

   if (e->type() == QEvent::MouseButtonRelease || e->type() == QEvent::MouseButtonPress || e->type() == QEvent::MouseMove || e->type() == QEvent::Wheel) {

    Ais3dTools::ALUGLViewer* v = static_cast<Ais3dTools::ALUGLViewer*>(o);
    size_t event_widget_index = 0;
    for(size_t i = 0;i < widgets.size(); ++i)
      if(v == widgets[i]){
        event_widget_index = i;
        if(e->type() == QEvent::Wheel){
          QWheelEvent* k = static_cast<QWheelEvent*>(e);
          const float wheelSensitivityCoef = 8E-4f;
          float coef = qMax(fabsf((v->camera()->frame()->coordinatesOf(v->camera()->revolveAroundPoint())).z), 0.2f*v->camera()->sceneRadius());
          qglviewer::Vec trans(0.0, 0.0, coef * k->delta() * v->camera()->frame()->wheelSensitivity() * wheelSensitivityCoef);
          v->camera()->frame()->translate(v->camera()->frame()->inverseTransformOf(trans));
        }

        if(e->type() == QEvent::MouseButtonPress){
          QMouseEvent* k = static_cast<QMouseEvent*>(e);
          prevPos_ = k->pos();
          if(k->button() & Qt::RightButton)
            rightMouseMove = true;
          if(k->button() & Qt::LeftButton)
            leftMouseMove = true;
        }

        if(e->type() == QEvent::MouseButtonRelease){
          rightMouseMove = false;
          leftMouseMove = false;
        }


/*  Quaternion rot(Vec(0.0, 0.0, 1.0), angle);
  //#CONNECTION# These two methods should go together (spinning detection and activation)
  computeMouseSpeed(event);
  setSpinningQuaternion(rot);
  spin();*/


       if(e->type() == QEvent::MouseMove || e->type() == QEvent::MouseButtonPress){
          QMouseEvent* k = static_cast<QMouseEvent*>(e);
          if(rightMouseMove){
            QPoint delta = prevPos_ - k->pos();
            qglviewer::Vec trans(static_cast<float>(delta.x()), static_cast<float>(-delta.y()), 0.0);
            trans *= 2.0 * tan(v->camera()->fieldOfView()/2.0) * fabs((v->camera()->frame()->coordinatesOf(v->camera()->frame()->revolveAroundPoint())).z) / v->camera()->screenHeight();
            v->camera()->frame()->translate(v->camera()->frame()->inverseTransformOf(trans));
          }

          if(leftMouseMove){
            qglviewer::Vec trans = v->camera()->projectedCoordinatesOf(v->camera()->frame()->revolveAroundPoint());

            float px = v->camera()->frame()->rotationSensitivity() * (prevPos_.x()  - trans[0]) / v->camera()->screenWidth();
            float py = v->camera()->frame()->rotationSensitivity() * (trans[1] - prevPos_.y())  / v->camera()->screenHeight();
            float dx = v->camera()->frame()->rotationSensitivity() * (k->x() - trans[0])     / v->camera()->screenWidth();
            float dy = v->camera()->frame()->rotationSensitivity() * (trans[1] - k->y())     / v->camera()->screenHeight();

            float d = 0.5 - (px*px + py*py);
            const qglviewer::Vec p1(px, py, sqrt(d));
            const qglviewer::Vec p2(dx, dy, sqrt(d));
            const qglviewer::Vec axis = cross(p2,p1);
            const float angle = 2.0 * asin(sqrt(axis.squaredNorm() / p1.squaredNorm() / p2.squaredNorm()));
            qglviewer::Quaternion rot = qglviewer::Quaternion(axis, angle);

            v->camera()->frame()->setSpinningQuaternion(rot);
            v->camera()->frame()->rotateAroundPoint(v->camera()->frame()->spinningQuaternion(), v->camera()->frame()->revolveAroundPoint());
          }
          prevPos_ = k->pos();
       }

      }

    qglviewer::Vec offset = qglviewer::Vec(eyeDistance, 0.0, 0.0);
    if(event_widget_index == 1){
      offset = -offset;
    }

    qglviewer::Vec pose2 = (widgets[event_widget_index]->camera()->orientation() * offset) + widgets[event_widget_index]->camera()->position();
    for(size_t i = 0;i < widgets.size(); ++i){
      if(i != event_widget_index){
        widgets[i]->camera()->setPosition(pose2);
        widgets[i]->camera()->setOrientation(widgets[event_widget_index]->camera()->orientation());
      }
      widgets[i]->updateGL();
    }

    if(e->type() == QEvent::MouseButtonPress){
      QMouseEvent* k = static_cast<QMouseEvent*>(e);
      prevPos_ = k->pos();
    }

    return true;
   }

  for(size_t i = 0;i < widgets.size(); ++i)
    widgets[i]->updateGL();
  // we do not handle anything specific, the widget may still receive the events
  return false;
}

void ALUGLWidgetStereoEventFilter::watchViewer(Ais3dTools::ALUGLViewer* v)
{
  widgets.push_back(v);
  v->installEventFilter(this);
}

}
