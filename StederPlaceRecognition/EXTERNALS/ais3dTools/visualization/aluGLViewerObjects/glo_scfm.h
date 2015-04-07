#ifndef ALU_GL_OBJECT_SPARSE_CODED_FREQUENCY_MODEL_H
#define ALU_GL_OBJECT_SPARSE_CODED_FREQUENCY_MODEL_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"

#include <Eigen/Geometry>

namespace Ais3dTools {
  ///forward declaration
  class SparseCodedMap;
  class SparseCodedModel;

  class GLOSparseCodedFrequencyModel : public ALUGLViewerObject
  {
    public:
      GLOSparseCodedFrequencyModel();
      GLOSparseCodedFrequencyModel(SparseCodedModel* model, SparseCodedMap* map);


      ~GLOSparseCodedFrequencyModel();

      virtual void draw() const;


    protected:
      SparseCodedModel* _model;
      SparseCodedMap* _map;
  };

}  // Namespace end


#endif
