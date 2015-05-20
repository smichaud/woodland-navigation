template <typename VectorType>
EigenLinesObjectT<VectorType>::EigenLinesObjectT(const PointsVector* startPoints, const PointsVector* endPoints) :
 ALUGLViewerObject(0.8f, 0.0f, 0.0f), _startPoints(startPoints), _endPoints(endPoints)
{
  draw_color_r_=1.0f, draw_color_g_=0.0f, draw_color_b_=0.0f;
}

template <typename VectorType>
EigenLinesObjectT<VectorType>::~EigenLinesObjectT() {
}

template <typename VectorType>
void EigenLinesObjectT<VectorType>::draw() const
{
  if (!_startPoints || !_endPoints)
    return;
  glPushMatrix();
  GLboolean hasLight = glIsEnabled(GL_LIGHTING);
  if (hasLight)
    glDisable(GL_LIGHTING);

    size_t size = min(_startPoints->size(), _endPoints->size());
    glPushMatrix();
      glColor3f(draw_color_r_, draw_color_g_, draw_color_b_);
      glBegin(GL_LINES);
        for(size_t i = 0; i < size; ++i)
        {
          const VectorType& start = (*_startPoints)[i];
          glVertex3f(start(0), start(1), start(2));

          const VectorType& end   = (*_endPoints)[i];
          glVertex3f(end(0), end(1), end(2));
        }
      glEnd();
    glPopMatrix();

  if (hasLight)
    glEnable(GL_LIGHTING);
  glPopMatrix();
}
