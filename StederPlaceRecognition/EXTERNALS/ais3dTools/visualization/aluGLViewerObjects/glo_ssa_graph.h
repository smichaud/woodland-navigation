#ifndef ALU_GL_OBJECT_SSA_GRAPH_SHARED_H
#define ALU_GL_OBJECT_SSA_GRAPH_SHARED_H

#include <QObject>
#include "ssa/core/ssa_graph_3d.h"
#include "ssa/types_3d/edge_se3_xyzcov.h"
#include "visualization/aluGLViewer/alu_glviewer_object.h"

namespace Ais3dTools {

template <typename SSAGraphType>
class SSAGraphObjectT : public ALUGLViewerObject
{
  public:
    //typedef typename PointCloudType::iterator       iterator;
    //typedef typename PointCloudType::const_iterator const_iterator;

    SSAGraphObjectT(SSAGraphType* graph=NULL);
    ~SSAGraphObjectT();

    void drawPoseBox() const;
    void drawBox(GLfloat l, GLfloat w, GLfloat h) const;
    virtual void draw() const;
    virtual void drawVertices() const;

    void set(SSAGraphType* graph) { graph_ = graph;}
    SSAGraphType* get() { return graph_;}

    inline void setPointSize(float pointSize=1.0f) { _pointSize=pointSize;}

    bool showCorrespondences;

    public slots:
      void setDrawLevel(int level);

  protected:
    SSAGraphType* graph_;
    float _pointSize;
    int level_;
};

#include "glo_ssa_graph.hpp"

}  // Namespace end


#endif
