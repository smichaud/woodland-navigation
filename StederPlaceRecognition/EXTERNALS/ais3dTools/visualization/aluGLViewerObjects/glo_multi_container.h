#ifndef ALU_GL_OBJECT_MULTI_CONTAINER_H
#define ALU_GL_OBJECT_MULTI_CONTAINER_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"

#include <Eigen/Geometry>

namespace Ais3dTools {
  template <typename DrawObjectType>
  class MultiContainerObject : public ALUGLViewerObject
  {
    public:
      MultiContainerObject(const std::vector<DrawObjectType>* objects=NULL);
      
      ~MultiContainerObject();

      virtual void draw() const;
      virtual void drawPoints() const;

    protected:
      const std::vector<DrawObjectType>* _objects;
  };
  
  #include "glo_multi_container.hpp"
}  // Namespace end


#endif
