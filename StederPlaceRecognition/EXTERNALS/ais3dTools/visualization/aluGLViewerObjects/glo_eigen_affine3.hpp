namespace Ais3dTools {

template <typename MatrixType>
EigenMatrixObjectT<MatrixType>::EigenMatrixObjectT(const typename std::vector<MatrixType>* matrices) :
  ALUGLViewerObject(), _matrices(matrices), _anaglyph(false), _anag_blue_red(false)
{
  setDrawColor(0.0f, 0.0f, 0.8f);
}

template <typename MatrixType>
EigenMatrixObjectT<MatrixType>::~EigenMatrixObjectT() {
}

template <typename MatrixType>
void EigenMatrixObjectT<MatrixType>::draw() const
{
  if (!_matrices)
    return;
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glDisable(GL_LIGHTING);
  float scalingFactor = 0.05f;
  for(typename std::vector<MatrixType>::const_iterator it = _matrices->begin(); it != _matrices->end(); ++it)
  {
    glPushMatrix();
      glTranslatef(it->translation()(0), it->translation()(1), it->translation()(2));
      glScalef(scalingFactor, scalingFactor, scalingFactor);
      glColor3f(1.0f, 0.0f, 0.0f);
      glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(it->matrix()(0,0), it->matrix()(0,1), it->matrix()(0,2));
      glEnd();
  
      glColor3f(0.0f, 1.0f, 0.0f);
      glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(it->matrix()(1,0), it->matrix()(1,1), it->matrix()(1,2));
      glEnd();
  
      glColor3f(0.0f, 0.0f, 1.0f);
      glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(it->matrix()(2,0), it->matrix()(2,1), it->matrix()(2,2));
      glEnd();
    glPopMatrix();
  }
  glPopAttrib();
}



}  // Namespace end
