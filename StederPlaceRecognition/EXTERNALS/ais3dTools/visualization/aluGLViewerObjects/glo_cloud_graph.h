#ifndef ALU_GL_OBJECT_CLOUD_GRAPH_SHARED_H
#define ALU_GL_OBJECT_CLOUD_GRAPH_SHARED_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"
#include "modelling/cloudGraph/cloud_graph.h"

namespace Ais3dTools {

template <typename PointCloudType>
class CloudGraphObjectT : public ALUGLViewerObject
{
  public:
    //typedef typename PointCloudType::iterator       iterator;
    //typedef typename PointCloudType::const_iterator const_iterator;
    
    CloudGraphObjectT(CloudGraphT< PointCloudType >* graph=NULL);
    ~CloudGraphObjectT();

    void drawPoseBox() const;
    void drawBox(GLfloat l, GLfloat w, GLfloat h) const;
    virtual void draw() const;
    virtual void drawPoints(int id) const;
    
    void set(CloudGraphT< PointCloudType >* graph) { graph_ = graph;}
    CloudGraphT< PointCloudType>* get() { return graph_;}
    
    inline void setPointSize(float pointSize=1.0f) { _pointSize=pointSize;}
    inline void setHasColor(bool state){ _hasColor = state;}
    inline bool getHasColor(){ return _hasColor;}

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

    inline void setTransformation(Eigen::Affine3f& trans){ _transformation = trans; _apply_trans = true;}
    inline Eigen::Affine3f& getTransformation(){ return _transformation;}

  protected:
    CloudGraphT< PointCloudType>* graph_;
    float _pointSize;
    bool _hasColor;
    bool _apply_trans;
    Eigen::Affine3f  _transformation;
    bool _anaglyph;
    bool _anag_blue_red;
};

#include "glo_cloud_graph.hpp"

}  // Namespace end


#endif
