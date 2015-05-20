#ifndef ALU_GL_WIDGET_H
#define ALU_GL_WIDGET_H

#include <QObject>
#include <QBoxLayout>
#include <QResizeEvent>
#include <QtGui/QAction>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>

#include "visualization/aluGLViewer/alu_glviewer_object.h"
#include "visualization/aluGLViewerObjects/glo_pcl_pointcloud.h"
#include "visualization/aluGLViewerObjects/glo_selection_box.h"
#include "visualization/aluGLViewerEventFilter/alu_glviewer_event_filter.h"
#include "visualization/aluGLViewerEventFilter/alu_glwidget_stereo_event_filter.h"

namespace Ais3dTools {
  class ALUGLViewer;
  class ALUGLViewerObject;

  class ALUGLWidget : public QWidget
  {
    Q_OBJECT
    public:
    ALUGLWidget(QWidget* parent);
    ~ALUGLWidget();

    void initMenu();

    void add(Ais3dTools::ALUGLViewerObject* child);
    void remove(Ais3dTools::ALUGLViewerObject* child);

    public slots:
      void updateGL();
      void quit();

      /** menu stuff */
      void toggleMenu();
      void showMenu();
      void hideMenu();

      void toggleAxisIsDrawn();

      /** Stereo vis stuff*/
      void setSplitStereoTopDown();
      void setSplitStereoSideBySide();
      void unsetSplitStereo();

    public: // the public variables
      ALUGLViewerEventFilter        evtFilter;
      ALUGLWidgetStereoEventFilter  mouseStereEventFilter;
      void setSplitStereo(bool topDown);

      void resizeEvent(QResizeEvent* event);

    protected:
      QBoxLayout*       layout_;
      Ais3dTools::ALUGLViewer* glViewer1_;
      Ais3dTools::ALUGLViewer* glViewer2_;

      QMenuBar*         menuBar_;
      QMenu*            menuFile_;
      QMenu*            menuView_;

  };

}
#endif
