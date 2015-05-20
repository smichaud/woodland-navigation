#ifndef ALU_GL_OBJECT_EIGEN_MATRIX3_H
#define ALU_GL_OBJECT_EIGEN_MATRIX3_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"
#include "visualization/aluGLViewerObjects/glo_primitives.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace Ais3dTools {
template <typename MatrixType, typename VectorType>
class EigenMatrixObjectT : public ALUGLViewerObject
{
  public:
    //typedef typename PointCloudType::iterator       iterator;
    //typedef typename PointCloudType::const_iterator const_iterator;

    EigenMatrixObjectT(const std::vector<MatrixType>* matrices=NULL, const std::vector<VectorType>* vectors=NULL, const std::vector<VectorType>* scales=NULL);
    ~EigenMatrixObjectT();
    
    virtual void draw() const;
    
    void setMatrices(const std::vector<MatrixType>* matrices, const std::vector<VectorType>* vectors) { _matrices = matrices; _vectors = vectors;}
    const std::vector<MatrixType>* getMatrices() const { return _matrices;}

    /** enable / disable anaglyph drawing*/
    inline void setAnaglyph(bool state){ _anaglyph = state;}
    /** return anaglyph draw state */
    inline bool getAnaglyph(){ return _anaglyph;}

    /** 
      false: mangenta/gree 
      true:  blue / red
    */
    inline void setAnaglyphMode(bool state){ _anag_blue_red = state;} 
    inline bool getAnaglyphMode(){ return _anag_blue_red;}
    
    inline void setLineWidth(float lineWidth=1.0f) { _lineWidth=lineWidth;}
    inline void setScale(float scaleX=0.1f, float scaleY=0.1f, float scaleZ=0.1f) { _scaleX=scaleX; _scaleY=scaleY; _scaleZ=scaleZ;}
  protected:
    const std::vector<MatrixType>* _matrices;
    const std::vector<VectorType>* _vectors;
    const std::vector<VectorType>* _scales;        
    bool _anaglyph;
    bool _anag_blue_red;
    float _lineWidth;
    float _scaleX;
    float _scaleY;
    float _scaleZ;    
};

typedef EigenMatrixObjectT< Eigen::Matrix3d, Eigen::Vector3d >    GLOMatrix3d;
typedef EigenMatrixObjectT< Eigen::Matrix3f, Eigen::Vector3f >  GLOMatrix3f;

  #include "glo_eigen_matrix3.hpp"

}  // Namespace end



#endif
