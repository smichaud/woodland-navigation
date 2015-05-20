#ifndef GLO_SELECTION_BOX_H
#define GLO_SELECTION_BOX_H

#include <QGLViewer/qglviewer.h>
#include "visualization/aluGLViewer/alu_glviewer_object.h"
#include <Eigen/Geometry>

namespace Ais3dTools {

  using namespace std;

  class GLOSelectionBox : public ALUGLViewerObject
  {
    public:
      GLOSelectionBox();
      GLOSelectionBox(QWidget*& widget);
      ~GLOSelectionBox();

    virtual void draw() const;

    void drawBox() const;

    virtual bool eventFilter(QObject* o, QEvent* e);

    void readConfiguration(std::istream& filename);
    void writeConfiguration(std::ostream& filename);

    /** Member */
    Eigen::Affine3f pose_; 
    float   width_;
    float   length_;
    float   height_;

  };

}

#endif
