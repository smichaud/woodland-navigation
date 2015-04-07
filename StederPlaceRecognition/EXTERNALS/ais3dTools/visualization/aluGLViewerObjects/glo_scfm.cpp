#include "glo_scfm.h"
#include "modelling/sparseCodedModel/mapping/map_3d.h"
#include "modelling/sparseCodedModel/base/sparseCodedModel.h"

namespace Ais3dTools {

  GLOSparseCodedFrequencyModel::GLOSparseCodedFrequencyModel() :
  ALUGLViewerObject(), _model(0), _map(0)
  {
    setDrawColor(1.0f, 1.0f, 1.0f);
  }

  GLOSparseCodedFrequencyModel::GLOSparseCodedFrequencyModel(SparseCodedModel* model, SparseCodedMap* map) :
  ALUGLViewerObject(), _model(model), _map(map)
  {
    setDrawColor(1.0f, 1.0f, 1.0f);
  }

  GLOSparseCodedFrequencyModel::~GLOSparseCodedFrequencyModel() {
  }


  void GLOSparseCodedFrequencyModel::draw() const
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
    Eigen::MatrixXf rgb2(3*(width), width);


    for(SparseCodedMap::iterator it = _map->begin(); it != _map->end(); ++it){
      for(std::tr1::unordered_map<int, std::tr1::unordered_map<int, std::vector< SparseCodedMapCell > > >::iterator itt = it->second.begin(); itt != it->second.end(); ++itt){
	for(std::tr1::unordered_map<int, std::vector< SparseCodedMapCell > >::iterator ittt = itt->second.begin(); ittt != itt->second.end(); ++ittt){
	  for(std::vector< SparseCodedMapCell >::iterator itttt = ittt->second.begin(); itttt != ittt->second.end(); ++itttt){
	    glPushMatrix();
	    SparseCodedInstance* instance = &(*itttt);
	    glMultMatrixf(instance->pose.data());
	    _model->decodeInstanceMesh(instance, depth, rgb);
	    _model->decodeInstanceRGB(instance, rgb2);
	    rgb2 *= 1.f/255.f;
	    for(int y = 0; y < width; ++y)
	    {
	      glBegin(GL_QUADS);
	      for(int x= 0; x < width; ++x){
		glColor3f(rgb2(x , y), rgb2(x + width, y), rgb2( x + 2 * (width), y));
		//glColor3f(0.5f, 0.5f, 0.5f);
		if(!std::isnan(depth(x,y))){
		  glVertex3f( (x - whalf) * wordResolution, (y - whalf)   * wordResolution, depth(x,y));
		} else {
		    glEnd();
		    glBegin(GL_QUADS);
		}

		if(!std::isnan(depth(x,y+1))){
		  glVertex3f( (x - whalf) * wordResolution, (y+1 - whalf) * wordResolution, depth(x,y+1));
		} else {
		    glEnd();
		    glBegin(GL_QUADS);
		}
		if(!std::isnan(depth(x+1,y+1))){
		  glVertex3f( (x+1 - whalf) * wordResolution, (y+1 - whalf) * wordResolution, depth(x+1,y+1));
		} else {
		  glEnd();
		  glBegin(GL_QUADS);
		}
		if(!std::isnan(depth(x,y))){
		  glVertex3f( (x+1 - whalf) * wordResolution, (y - whalf)   * wordResolution, depth(x+1,y));
		} else {
		  glEnd();
		  glBegin(GL_QUADS);
		}
	      }
	      glEnd();
	    }
	  glPopMatrix();
	  }
	}
      }
    }
    if (hasLight)
      glEnable(GL_LIGHTING);
    glPopMatrix();
  }
}  // Namespace end
