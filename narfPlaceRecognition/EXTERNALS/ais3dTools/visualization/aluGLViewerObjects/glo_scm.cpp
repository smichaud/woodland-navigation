#include "glo_scm.h"
#include "modelling/sparseCodedModel/base/sparseCodedModel.h"

namespace Ais3dTools {

  GLOSparseCodedModel::GLOSparseCodedModel() :
  ALUGLViewerObject(), colorizeLevels(false),  _model(0)
  {
    setDrawColor(1.0f, 1.0f, 1.0f);
  }

  GLOSparseCodedModel::GLOSparseCodedModel(SparseCodedModel* model, bool colLevels) :
  ALUGLViewerObject(), colorizeLevels(colLevels), _model(model)
  {
    setDrawColor(1.0f, 1.0f, 1.0f);
  }

  GLOSparseCodedModel::~GLOSparseCodedModel() {
  }


  void GLOSparseCodedModel::draw() const
  {
    if(_model == 0)
      return;

    glPushMatrix();
    GLboolean hasLight = glIsEnabled(GL_LIGHTING);
    if (hasLight)
      glDisable(GL_LIGHTING);

    int width = _model->getWidth();
    float whalf = ((float) width) /2.0f;
    float wordResolution = _model->getWordResolution(0);
    glColor3f(draw_color_r_, draw_color_g_, draw_color_b_);
    glMatrixMode(GL_MODELVIEW);

    glFrontFace(GL_CW);
    Eigen::MatrixXf depth(width+1, width+1);
    Eigen::MatrixXf rgb(3*(width+1), width+1);
    Eigen::MatrixXf depth2(width, width);
    Eigen::MatrixXf rgb2(3*(width), width);
    for(size_t i=0; i < _model->instances().size(); ++i){
//       if(i != 100)
//         continue;
      glPushMatrix();
      SparseCodedInstance* instance = _model->instances()[i];
      glMultMatrixf(instance->pose.data());
      _model->decodeInstanceMesh(instance, depth, rgb);
      _model->decodeInstanceDepth(instance, depth2);
      _model->decodeInstanceRGB(instance, rgb2);
/*      std::cerr << PVAR(width) << std::endl << PVAR(depth2) << std::endl;
      std::cerr << PVAR(depth) << std::endl;   */
      rgb2 *= 1.f/255.f;
      width = _model->getWidth(instance->resolutionLevel);
      whalf = ((float) width) /2.0f;
      wordResolution = _model->getWordResolution(instance->resolutionLevel);

//       std::cerr << PVAR(depth.cols()) << std::endl;
//       for(int y = 0; y < width; ++y)
//       {
//         glBegin(GL_TRIANGLE_STRIP);
//         for(int x= 0; x < width+1; ++x){
// //           std::cerr << x << " " << y+1 << std::endl;
//           if(!std::isnan(depth(x,y+1))){
//             glColor3f(rgb(x , y+1), rgb(x + width+1, y+1), rgb( x + 2 * (width+1), y+1));
//             glVertex3f( (x - whalf) * wordResolution, (y+1 - whalf) * wordResolution, depth(x,y+1));
//           } else {
//             glEnd();
//             glBegin(GL_TRIANGLE_STRIP);
//           }
// //           std::cerr << x << " " << y << std::endl;
//           if(!std::isnan(depth(x,y))){
//             glColor3f(rgb( x, y  ), rgb( x + width+1, y  ), rgb( x + 2 * (width+1), y  ));
// //             std::cerr << (x - whalf) * wordResolution << " " << (y - whalf) * wordResolution << " " << depth(x,y) << std::endl;
//             glVertex3f( (x - whalf) * wordResolution, (y - whalf)   * wordResolution, depth(x,y));
//           } else {
//             glEnd();
//             glBegin(GL_TRIANGLE_STRIP);
//           }
//         }
//         glEnd();
//       }

        for(int y = 0; y < width; ++y)
        {
            glBegin(GL_QUADS);
          for(int x= 0; x < width; ++x){
            if(!colorizeLevels){
                glColor3f(rgb2(x , y), rgb2(x + width, y), rgb2( x + 2 * (width), y));
            } else {
                if(instance->resolutionLevel == 0)
                  glColor3f(1.0f, 0.0f, 0.0f);
                if(instance->resolutionLevel == 1)
                  glColor3f(0.0, 1.0, 0.0);
                if(instance->resolutionLevel == 2)
                  glColor3f(0.0, 0.0, 1.0);
                if(instance->resolutionLevel == 3)
                  glColor3f(0.5, 0.5, 0.0);
                if(instance->resolutionLevel == 4)
                  glColor3f(0.0, 0.5, 0.5);
                if(instance->resolutionLevel == 5)
                  glColor3f(0.5, 0.5, 0.5);
                if(instance->resolutionLevel == 6)
                  glColor3f(0.0, 0.0, 0.5);
                if(instance->resolutionLevel == 7)
                  glColor3f(0.0, 0.5, 0.0);
                if(instance->resolutionLevel == 8)
                  glColor3f(0.5, 0.0, 0.0);
                if(instance->resolutionLevel == 9)
                  glColor3f(0.8, 0.8, 0.8);
            }
            if(std::isnan(depth2(x,y))){

            } else {
              glVertex3f( (x - whalf) * wordResolution, (y - whalf)   * wordResolution, depth(x,y));
              glVertex3f( (x - whalf) * wordResolution, (y+1 - whalf) * wordResolution, depth(x,y+1));
              glVertex3f( (x+1 - whalf) * wordResolution, (y+1 - whalf)   * wordResolution, depth(x+1,y+1));
              glVertex3f( (x+1 - whalf) * wordResolution, (y - whalf) * wordResolution, depth(x+1,y));
            }
          }
          glEnd();
        }
      glPopMatrix();
    }
    if (hasLight)
      glEnable(GL_LIGHTING);
    glPopMatrix();
  }
}  // Namespace end
