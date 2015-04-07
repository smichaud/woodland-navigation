#ifndef ALU_GL_OBJECT_EIGEN_LINES_H
#define ALU_GL_OBJECT_EIGEN_LINES_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"

#include <Eigen/Core>

namespace Ais3dTools {
template <typename VectorType>
class EigenLinesObjectT : public ALUGLViewerObject
{
  public:
    typedef std::vector<VectorType, Eigen::aligned_allocator<VectorType> > PointsVector;

    EigenLinesObjectT(const PointsVector* startPoints=NULL, const PointsVector* endPoints=NULL);
    ~EigenLinesObjectT();
    
    virtual void draw() const;

   ///Getter / Setter for start points
   void setStartPoints(const PointsVector* startPoints) { _startPoints = startPoints;}
   const PointsVector* startPoints() const { return _startPoints;}

    ///Getter / Setter for end points
   void setEndPoints(const PointsVector* endPoints) { _endPoints = endPoints;}
   const PointsVector* endPoints() const { return _endPoints;}

  protected:
    const PointsVector* _startPoints;
    const PointsVector* _endPoints;
};

#include "glo_eigen_lines.hpp"

typedef EigenLinesObjectT< Eigen::Vector3d > GLOLineVector3d;
typedef EigenLinesObjectT< Eigen::Vector3f > GLOLineVector3f;

}  // Namespace end

#endif
