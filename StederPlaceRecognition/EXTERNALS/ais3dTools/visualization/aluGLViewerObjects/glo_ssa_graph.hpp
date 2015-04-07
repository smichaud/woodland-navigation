
template <typename SSAGraphType>
SSAGraphObjectT<SSAGraphType>::SSAGraphObjectT(SSAGraphType* graph) :
  ALUGLViewerObject(), showCorrespondences(false), level_(0)
{
  setDrawColor(0.0f, 0.0f, 0.8f);
  setPointSize(1.0);
  graph_ = graph;
}

template <typename SSAGraphType>
SSAGraphObjectT<SSAGraphType>::~SSAGraphObjectT() {
}


template <typename SSAGraphType>
void SSAGraphObjectT<SSAGraphType>::drawPoseBox() const
{
  glPushMatrix();
  glScalef(0.5,1,1);
  glPushMatrix();
  glScalef(1,0.25,0.5);
  glTranslatef(-0.5,0.5,0);
  glColor3f(1.0, 0.3, 0.3);
  drawBox(1, 1, 1);
  glPopMatrix();

  glPushMatrix();
  glScalef(1,0.25,0.5);
  glTranslatef(-0.5,-0.5,0);
  glColor3f(1.0, 0.1, 0.1);
  drawBox(1, 1, 1);
  glPopMatrix();

  glPushMatrix();
  glScalef(1,0.25,0.5);
  glTranslatef(+0.5,0.5,0);
  glColor3f(0.3, 0.3, 1.0);
  drawBox(1, 1, 1);
  glPopMatrix();

  glPushMatrix();
  glScalef(1,0.25,0.5);
  glTranslatef(+0.5,-0.5,0);
  glColor3f(0.1, 0.1, 1.);
  drawBox(1, 1, 1);
  glPopMatrix();
  glPopMatrix();
}

template <typename SSAGraphType>
void SSAGraphObjectT<SSAGraphType>::drawBox(GLfloat l, GLfloat w, GLfloat h) const
{
  GLfloat sx = l*0.5f;
  GLfloat sy = w*0.5f;
  GLfloat sz = h*0.5f;

  glBegin(GL_QUADS);
  // bottom
  glNormal3f( 0.0f, 0.0f,-1.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, -sy, -sz);
  // top
  glNormal3f( 0.0f, 0.0f,1.0f);
  glVertex3f(-sx, -sy, sz);
  glVertex3f(-sx, sy, sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(sx, -sy, sz);
  // back
  glNormal3f(-1.0f, 0.0f, 0.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(-sx, sy, sz);
  glVertex3f(-sx, -sy, sz);
  // front
  glNormal3f( 1.0f, 0.0f, 0.0f);
  glVertex3f(sx, -sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(sx, -sy, sz);
  // left
  glNormal3f( 0.0f, -1.0f, 0.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(sx, -sy, -sz);
  glVertex3f(sx, -sy, sz);
  glVertex3f(-sx, -sy, sz);
  //right
  glNormal3f( 0.0f, 1.0f, 0.0f);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(-sx, sy, sz);
  glEnd();
}

template<typename SSAGraphType>
inline void SSAGraphObjectT<SSAGraphType>::drawVertices() const
{
//   int increment = graph_->_optimizer.vertices().size() / 1000;
//   increment = min(increment, 1);
//   int i = 0;
   // Draw surface vertices
  if(level_ == 0){
    for (std::vector<ssa::VertexPointXYZCov*>::const_iterator it=graph_->_verticies_points.begin(); it!=graph_->_verticies_points.end(); it++){
      ssa::VertexPointXYZCov* v=(*it);
      if(v){
        if(v->covariance() == Eigen::Matrix3d::Identity() || v->edges().size() < 1)
          continue;
        glPointSize(_pointSize);
        glColor3ub(v->cr, v->cg, v->cb);
        glBegin(GL_POINTS);
          glVertex3f(v->estimate()(0),v->estimate()(1), v->estimate()(2));
        glEnd();
      }
    }
  } else {
    for (size_t i=0; i < graph_->_edges_observations.size(); i++){
      ssa::EdgeSE3PointXYZCov*& edge=graph_->_edges_observations[i];
      if(edge->level() >= level_ && edge->vertices().size() == 2){
        ssa::VertexPointXYZCov* v=dynamic_cast<ssa::VertexPointXYZCov*>(edge->vertices()[1]);
        if(v && (v->covariance() == Eigen::Matrix3d::Identity() || v->edges().size() < 1))
          continue;
        glPointSize(_pointSize);
        glColor3ub(v->cr, v->cg, v->cb);
        glBegin(GL_POINTS);
          glVertex3f(v->estimate()(0),v->estimate()(1), v->estimate()(2));
        glEnd();
      } else {
        if(edge->level() == level_)
          std::cerr << __PRETTY_FUNCTION__ << " graph structure is corrupt. Check _edges_observations. " << i << " " << edge->vertices().size() << std::endl;
      }
    }
  }
}


template <typename SSAGraphType>
void SSAGraphObjectT<SSAGraphType>::draw() const
{
  if (!graph_)
    return;
  glPushMatrix();
  GLboolean hasLight = glIsEnabled(GL_LIGHTING);
  if (hasLight)
    glDisable(GL_LIGHTING);
  glColor3f(draw_color_r_, draw_color_g_, draw_color_b_);
  glPointSize(_pointSize);

  for (std::vector<ssa::VertexSE3*>::iterator it=graph_->_verticies_poses.begin(); it!=graph_->_verticies_poses.end(); it++){
    g2o::VertexSE3*& v= (*it);
    if(v){
        glColor3f(1.0, 1.0, 1.0);
        glPushMatrix();
          glMatrixMode(GL_MODELVIEW);
          glMultMatrixd(v->estimate().data());
          glScalef(0.01f, 0.01f, 0.01f);
          drawPoseBox();
        glPopMatrix();
    }
  }
  drawVertices();

   /** draw edges */
//   for (g2o::OptimizableGraph::EdgeSet::iterator it=graph_->_optimizer.edges().begin(); it!=graph_->_optimizer.edges().end(); it++){
//     g2o::EdgeSE3* e =dynamic_cast<g2o::EdgeSE3*>(*it);
//     if(e){
//       g2o::VertexSE3* from = dynamic_cast<g2o::VertexSE3*>(e->vertices()[0]);
//       g2o::VertexSE3* to = dynamic_cast<g2o::VertexSE3*>(e->vertices()[1]);
//       glColor3f(1.0, 1.0, 1.0);
//       glBegin(GL_LINES);
//         glVertex3f(from->estimate().translation()[0], from->estimate().translation()[1], from->estimate().translation()[2]);
//         glVertex3f(to->estimate().translation()[0], to->estimate().translation()[1], to->estimate().translation()[2]);
//       glEnd();
//     }
//   }

  glColor3f(1.0, 0.0, 0.0);
  if(showCorrespondences)
  for(unsigned int i = 0; i < graph_->_edges_data_association.size(); ++i){
     ssa::EdgePointXYZCovPointXYZCov*& edge = graph_->_edges_data_association[i];
     ssa::VertexPointXYZCov* from = dynamic_cast<ssa::VertexPointXYZCov* >(edge->vertices()[0]);
     ssa::VertexPointXYZCov* to = dynamic_cast<ssa::VertexPointXYZCov* >(edge->vertices()[1]);
     glBegin(GL_LINES);
      glVertex3f(from->estimate()[0], from->estimate()[1], from->estimate()[2]);
      glVertex3f(to->estimate()[0], to->estimate()[1], to->estimate()[2]);
     glEnd();
  }

  glPointSize(1.0f);
  if (hasLight)
    glEnable(GL_LIGHTING);
  glPopMatrix();
}

template <typename SSAGraphType>
void SSAGraphObjectT<SSAGraphType>::setDrawLevel(int level){
  level_ = level;
  if(getUseDrawList())
    updateDrawLists();
}
