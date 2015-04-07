#ifndef ALU_GL_VIEWER_OBJECT_H
#define ALU_GL_VIEWER_OBJECT_H

#include <QGLViewer/qglviewer.h>
#include <Eigen/Geometry>

namespace Ais3dTools {
  //using namespace Eigen;
  using namespace std;
  class ALUGLViewer;

  class ALUGLViewerObject : public QObject
  {
    Q_OBJECT
    public:
      ALUGLViewerObject();
      ALUGLViewerObject(float r, float g, float b);
      ALUGLViewerObject(QWidget*& widget);
      ALUGLViewerObject(const ALUGLViewerObject& other);
      ~ALUGLViewerObject();

    virtual void preDraw() const;

    virtual void draw() const;

    virtual void postDraw() const;

    virtual void drawWithNames() const;

    virtual void drawWithChildren() const;

    virtual void compileDrawLists();

    virtual void updateDrawLists();

    void updateGLViewers() const;

    /** set the draw color */
    void setDrawColor(float r, float g, float b);

    /** children related */
    void add(ALUGLViewerObject* child);
    void remove(ALUGLViewerObject* child);

    /** parent getter */
    ALUGLViewerObject*& getParent();

    virtual bool eventFilter(QObject* o, QEvent* e);

    void setUseDrawList(bool use);
    bool& getUseDrawList();

    inline void setTransformation(const Eigen::Affine3f& trans){ transformation_.matrix() =  trans.matrix(); apply_cloud_trans_ = true;}
    inline void setTransformation(const Eigen::Isometry3f& trans){ transformation_ =  trans; apply_cloud_trans_ = true;}
    inline Eigen::Isometry3f& getTransformation(){ return transformation_;}
    inline Eigen::Affine3f getAffineTransformation(){  Eigen::Affine3f trans(transformation_.matrix()); return trans;}

    inline void setUseTransformation(bool state){ apply_cloud_trans_ = state;}
    inline bool getUseTransformation(){ return apply_cloud_trans_;}

    void readTransformation(std::istream& filename);
    void writeTransformation(std::ostream& filename);

    /** member variables */
    vector<ALUGLViewer* >            viewer_;
    ALUGLViewerObject*               parent_;
    std::vector<ALUGLViewerObject* > children_;

    GLint draw_list_;
    bool use_draw_list_;

    float draw_color_r_;
    float draw_color_g_;
    float draw_color_b_;

  protected:
    bool apply_cloud_trans_;
    Eigen::Isometry3f  transformation_;

  };

}

#endif
