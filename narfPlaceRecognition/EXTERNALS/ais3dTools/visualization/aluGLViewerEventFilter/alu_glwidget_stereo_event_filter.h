#ifndef ALU_GL_WIDGET_STEREO_EVENT_FILTER_H
#define ALU_GL_WIDGET_STEREO_EVENT_FILTER_H

#include <QObject>
#include <QPoint>
#include <vector>

namespace Ais3dTools {
  class ALUGLViewer;

class ALUGLWidgetStereoEventFilter : public QObject
{
  Q_OBJECT
  public:
    ALUGLWidgetStereoEventFilter();
    ~ALUGLWidgetStereoEventFilter();

    void watchViewer(Ais3dTools::ALUGLViewer* v);

  public: // the public variables
    bool hasNewRobotPose;
    double robotPose[3];
    bool repositioning;

    bool hasNewTargetPose;
    double targetPose[2];

    bool quit;
    std::vector<Ais3dTools::ALUGLViewer*> widgets;

  protected:

    bool eventFilter(QObject* o, QEvent* e);
    bool getPoseFromGl(int x, int y, double& wx, double& wy);
    void togglePoseInit();

    QPoint prevPos_;
    bool leftMouseMove;
    bool rightMouseMove;
    double eyeDistance;
};

}

#endif
