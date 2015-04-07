
template <typename PointCloudType>
CloudGraphObjectT<PointCloudType>::CloudGraphObjectT(CloudGraphT< PointCloudType >* graph) :
  ALUGLViewerObject(), _apply_trans(false), _anaglyph(false), _anag_blue_red(false)
{
  setDrawColor(0.0f, 0.0f, 0.8f);
  setPointSize();
  _apply_trans = false;
  graph_ = graph;
  _transformation = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

}

template <typename PointCloudType>
CloudGraphObjectT<PointCloudType>::~CloudGraphObjectT() {
}


template <typename PointCloudType>
void CloudGraphObjectT<PointCloudType>::drawPoseBox() const
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

template <typename PointCloudType>
void CloudGraphObjectT<PointCloudType>::drawBox(GLfloat l, GLfloat w, GLfloat h) const
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

template <typename PointCloudType>
void CloudGraphObjectT<PointCloudType>::drawPoints(int id) const
{
  PointCloudType& cloud = graph_->observations_[id];
  if(cloud.size() == 0)
    return;
  glBegin(GL_POINTS);
  for (size_t i = 0; i < cloud.points.size (); ++i){
    if(std::isnan(cloud.points[i].x) || std::isnan(cloud.points[i].y) || std::isnan(cloud.points[i].z))
      continue;
    glVertex3f(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
  }
  glEnd();
}

template< >
inline void CloudGraphObjectT< pcl::PointCloud<pcl::PointXYZRGB> >::drawPoints(int id) const
{
  pcl::PointCloud<pcl::PointXYZRGB>& cloud = graph_->observations_[id];
  if(cloud.size() == 0)
    return;
  int increment = cloud.size() / 10000;
  if(increment < 1)
      increment = 1;
  glBegin(GL_POINTS);
  for (size_t i = 0; i < cloud.points.size (); i=i+increment){
    if(std::isnan(cloud.points[i].x) || std::isnan(cloud.points[i].y) || std::isnan(cloud.points[i].z))
      continue;
    glColor3ub(cloud.points[i].r, cloud.points[i].g, cloud.points[i].b);
    glVertex3f(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
  }
  glEnd();
}

template< >
inline void CloudGraphObjectT< pcl::PointCloud<pcl::PointXYZI> >::drawPoints(int id) const
{
  pcl::PointCloud<pcl::PointXYZI>& cloud = graph_->observations_[id];
  if(cloud.size() == 0)
    return;
  int increment = cloud.size() / 10000;
  if(increment < 1)
      increment = 1;
  glBegin(GL_POINTS);
  for (size_t i = 0; i < cloud.points.size (); i+=increment){
    if(std::isnan(cloud.points[i].x) || std::isnan(cloud.points[i].y) || std::isnan(cloud.points[i].z))
      continue;
    if(cloud.points[i].intensity > 0.55){
      glColor3f(0.0, 0.4, 0.0);
    } else { 
      glColor3f(cloud.points[i].intensity, cloud.points[i].intensity, cloud.points[i].intensity);
    }
    glVertex3f(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
  }
  glEnd();
}

template <typename PointCloudType>
void CloudGraphObjectT<PointCloudType>::draw() const
{
  if (!graph_)
    return;
  glPushMatrix();
  GLboolean hasLight = glIsEnabled(GL_LIGHTING);
  if (hasLight)
    glDisable(GL_LIGHTING);
  glColor3f(draw_color_r_, draw_color_g_, draw_color_b_);
  glPointSize(_pointSize);
  if(_apply_trans){
    glMatrixMode(GL_MODELVIEW);
    glMultMatrixf(_transformation.data());
  }

  for (g2o::OptimizableGraph::VertexIDMap::const_iterator it=graph_->optimizer_.vertices().begin(); it!=graph_->optimizer_.vertices().end(); it++){
    g2o::VertexSE3* v=dynamic_cast<g2o::VertexSE3*>(it->second);
    if(v){
        glColor3f(1.0, 1.0, 1.0);
        glPushMatrix();
          glMatrixMode(GL_MODELVIEW);
          glMultMatrixd(v->estimate().data());

          drawPoints(v->id());

          glScalef(0.1f, 0.1f, 0.1f);
          drawPoseBox();
        glPopMatrix();
    }
  }
   /** draw edges */
  for (g2o::OptimizableGraph::EdgeSet::iterator it=graph_->optimizer_.edges().begin(); it!=graph_->optimizer_.edges().end(); it++){
    g2o::EdgeSE3* e =dynamic_cast<g2o::EdgeSE3*>(*it);
    if(e){
      g2o::VertexSE3* from = dynamic_cast<g2o::VertexSE3*>(e->vertices()[0]);
      g2o::VertexSE3* to = dynamic_cast<g2o::VertexSE3*>(e->vertices()[1]);
      glColor3f(1.0, 1.0, 1.0);
      glBegin(GL_LINES);
        glVertex3f(from->estimate().translation()[0], from->estimate().translation()[1], from->estimate().translation()[2]);
        glVertex3f(to->estimate().translation()[0], to->estimate().translation()[1], to->estimate().translation()[2]);
      glEnd();  
    }
  }

  glPointSize(1.0f);
  if (hasLight)
    glEnable(GL_LIGHTING);
  glPopMatrix();
}

