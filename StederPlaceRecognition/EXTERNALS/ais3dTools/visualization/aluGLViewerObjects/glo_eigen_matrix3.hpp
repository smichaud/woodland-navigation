#include "pcl/common/transforms.h"

template <typename MatrixType, typename VectorType>
EigenMatrixObjectT<MatrixType, VectorType>::EigenMatrixObjectT(const typename std::vector<MatrixType>* matrices, const std::vector<VectorType>* vectors, const std::vector<VectorType>* scales) :
ALUGLViewerObject(), _matrices(matrices), _vectors(vectors), _scales(scales), _anaglyph(false), _anag_blue_red(false), _lineWidth(1.0) , _scaleX(0.1), _scaleY(0.1), _scaleZ(0.1)
{
  setDrawColor(0.0f, 0.0f, 0.8f);
}

template <typename MatrixType, typename VectorType>
EigenMatrixObjectT<MatrixType, VectorType>::~EigenMatrixObjectT() {
}

template <typename MatrixType, typename VectorType>
void EigenMatrixObjectT<MatrixType, VectorType>::draw() const
{
  if (!_matrices || !_vectors)
    return;
  glPushMatrix();
  GLboolean hasLight = glIsEnabled(GL_LIGHTING);
  if (hasLight)
    glDisable(GL_LIGHTING);
  glColor3f(draw_color_r_, draw_color_g_, draw_color_b_);
  glLineWidth(_lineWidth);
  int id=0;
  int increment = 1;
//   float scalingFactor = _lineScale;
  for(typename std::vector<MatrixType>::const_iterator it = _matrices->begin(); it != _matrices->end(); it+=increment)
  {
    const VectorType& translation = (*_vectors)[id];
    glPushMatrix();
      glTranslatef(translation(0), translation(1), translation(2));

      float factor1 = 1.0f;
      float factor2 = 1.0f;
      float factor3 = 1.0f;
      if(_scales != 0){
        if(id < (int) _scales->size()){
          factor1 = (*_scales)[id](0);
          factor2 = (*_scales)[id](1);
          factor3 = (*_scales)[id](2);
          glColor3f(1.0f, 0.0f, 0.0f);
          glBegin(GL_LINES);
          glVertex3f((*it).col(0)(0) * -factor1, (*it).col(0)(1) * -factor1, (*it).col(0)(2) * -factor1);
          glVertex3f((*it).col(0)(0) * factor1, (*it).col(0)(1) * factor1, (*it).col(0)(2) * factor1);
          glEnd();
          
          glColor3f(0.0f, 1.0f, 0.0f);
          glBegin(GL_LINES);
          glVertex3f((*it).col(1)(0) * -factor2, (*it).col(1)(1) * -factor2, (*it).col(1)(2) * -factor2);
          glVertex3f((*it).col(1)(0) * factor2, (*it).col(1)(1) * factor2, (*it).col(1)(2) * factor2);
          glEnd();
          
          glColor3f(0.0f, 0.0f, 1.0f);
          glBegin(GL_LINES);
          glVertex3f((*it).col(2)(0) * -factor3, (*it).col(2)(1) * -factor3, (*it).col(2)(2) * -factor3);
          glVertex3f((*it).col(2)(0) * factor3, (*it).col(2)(1) * factor3, (*it).col(2)(2) * factor3);
          glEnd();
          //drawEllipsoid((*_scales)[id](0), (*_scales)[id](1), (*_scales)[id](2));
        } else {
          std::cerr << "no valid scaling factor for id " << id << " " << PVAR(_scales->size()) << std::endl;
        }
      } else {
        glScalef(_scaleX, _scaleY, _scaleZ);
       
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINES);
          glVertex3f(0.0f, 0.0f, 0.0f);
          glVertex3f((*it).col(0)(0) * factor1, (*it).col(0)(1) * factor1, (*it).col(0)(2) * factor1);
        glEnd();

        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
          glVertex3f(0.0f, 0.0f, 0.0f);
          glVertex3f((*it).col(1)(0) * factor2, (*it).col(1)(1) * factor2, (*it).col(1)(2) * factor2);
        glEnd();

        glColor3f(0.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);
          glVertex3f(0.0f, 0.0f, 0.0f);
          glVertex3f((*it).col(2)(0) * factor3, (*it).col(2)(1) * factor3, (*it).col(2)(2) * factor3);
        glEnd();
      }
    glPopMatrix();
    id += increment;
  }
  if (hasLight)
    glEnable(GL_LIGHTING);
  glPopMatrix();
}
