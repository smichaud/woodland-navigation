#ifndef ALU_GL_OBJECT_EIGEN_MATRIX_OBJECT_H
#define ALU_GL_OBJECT_EIGEN_MATRIX_OBJECT_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace Ais3dTools {
template <typename MatrixType>
class EigenMatrixObjectT : public ALUGLViewerObject
{
  public:
    //typedef typename PointCloudType::iterator       iterator;
    //typedef typename PointCloudType::const_iterator const_iterator;

    EigenMatrixObjectT(const std::vector<MatrixType>* matrices=NULL);
    ~EigenMatrixObjectT();
    
    virtual void draw() const;
    
    void setMatrices(const std::vector<MatrixType>* matrices) { _matrices = matrices;}
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

  protected:
    const std::vector<MatrixType>* _matrices;
    bool _anaglyph;
    bool _anag_blue_red;
};

typedef EigenMatrixObjectT< Eigen::Affine3d > GLOAffine3d;
typedef EigenMatrixObjectT< Eigen::Affine3f > GLOAffine3f;

}  // Namespace end

#include "glo_eigen_affine3.hpp"

#endif
