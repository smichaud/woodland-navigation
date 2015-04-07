#ifndef ALU_GL_OBJECT_EMPTY_BOX_H
#define ALU_GL_OBJECT_EMPTY_BOX_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"

#include <Eigen/Geometry>

namespace Ais3dTools {
  class EmptyBoxObject : public ALUGLViewerObject
  {
    public:
      EmptyBoxObject();
      EmptyBoxObject(const EmptyBoxObject& other);
      EmptyBoxObject(const Eigen::Isometry3f& pose, float size);
      
      
      ~EmptyBoxObject();

      virtual void draw() const;

      inline void   setSize(double size){ _size = size;}
      inline double getSize(){ return _size;}

      inline void              setPose(Eigen::Isometry3f pose){ _pose = pose;}
      inline Eigen::Isometry3f getPose(){ return _pose;};

      inline void setLineWidth(float lineWidth=1.0f) { _lineWidth=lineWidth;}

    protected:
      float             _lineWidth;
      float             _size;
      Eigen::Isometry3f _pose;
  };
  
}  // Namespace end


#endif
