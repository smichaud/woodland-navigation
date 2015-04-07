#ifndef TEXTURED_PLANE_H
#define TEXTURED_PLANE_H

#include "visualization/aluGLViewer/alu_glviewer_object.h"

class QImage;

namespace Ais3dTools {

typedef unsigned char uchar;
class TexturedPlane : public ALUGLViewerObject {
  public:
    TexturedPlane();
    ~TexturedPlane();
    virtual void draw() const;
    virtual void writeVrml(std::ostream& os) const;
    /**
     * set from rgb buffer
     */
    void setImage(const uchar* imageData, unsigned int width, unsigned int height);
    /** giving a RGBA buffer */
    void setImageRGBA(const uchar* imageData, unsigned int width, unsigned int height);
    /**
     * set using a qimage
     */
    void setImage(const QImage& image);
    void setSize(double width, double height);
    void setTransparency(bool on) { _transparent = on;}
    void setDisableDepthBuffer(bool on) { _disableDepthBuffer = on;}
    void setCompressTexture(bool on) { _compressTexture = on;}

  protected:
    double _width, _height;
    bool _transparent;
    bool _disableDepthBuffer;
    bool _compressTexture;
    GLuint _currentTexture;
    bool _haveTexture;

};

}

#endif
