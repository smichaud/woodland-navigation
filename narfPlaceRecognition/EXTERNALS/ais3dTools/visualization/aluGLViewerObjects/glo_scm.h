#ifndef ALU_GL_OBJECT_SPARSE_CODED_MODEL_H
#define ALU_GL_OBJECT_SPARSE_CODED_MODEL_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"

#include <Eigen/Geometry>

namespace Ais3dTools {
  ///forward declaration
  class SparseCodedModel;

  class GLOSparseCodedModel : public ALUGLViewerObject
  {
    public:
      GLOSparseCodedModel();
      GLOSparseCodedModel(SparseCodedModel* model, bool colorLevels = false);


      ~GLOSparseCodedModel();

      virtual void draw() const;

      bool colorizeLevels;
    protected:
      SparseCodedModel* _model;
  };

}  // Namespace end


#endif
