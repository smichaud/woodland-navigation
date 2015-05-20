#include "glo_texturedPlane.h"
#include <GL/glu.h>
#include <QImage>
#include <cassert>

namespace Ais3dTools {

TexturedPlane::TexturedPlane() : ALUGLViewerObject(),
  _width(1.0), _height(1.0), _transparent(false), _disableDepthBuffer(false), _compressTexture(false), _haveTexture(false)
{
  _currentTexture = 0;
  glGenTextures(1, &_currentTexture);  // Get a texture

  if(_currentTexture == 0){
   std::cerr << __PRETTY_FUNCTION__ << ":" << std::endl;
   std::cerr << "Error: Texture generation failed missing glContext." << std::endl;
   std::cerr << "Keep in mind to initialize gl window before generating texture objects." << std::endl;
  }
  assert(_currentTexture != 0);
}

TexturedPlane::~TexturedPlane()
{
  glDeleteTextures(1, &_currentTexture);
}

void TexturedPlane::setImage(const uchar* imageData, unsigned int width, unsigned int height)
{
  if (imageData == NULL) {
    _haveTexture = false;
    return;
  }

  glBindTexture(GL_TEXTURE_2D, _currentTexture);
  gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB,  width, height, GL_RGB, GL_UNSIGNED_BYTE, imageData);
  //glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, imageData); // Texture Generation From RGB Bitmap
  //glTexImage2D(GL_TEXTURE_2D, 0, 4, 1024, 512, 0, GL_RGBA, GL_UNSIGNED_BYTE, _currentImage); // Texture Generation From RGBA Bitmap

  _haveTexture = true;
}

void TexturedPlane::draw() const
{
  if (!_haveTexture)
    return;
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);  // Choose type of interpolation to decrease size
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);  // Choose type of interpolation to increase size

  //glDisable(GL_COLOR_MATERIAL);
  glEnable(GL_TEXTURE_2D);  // Enable Texture Mapping
  if(_disableDepthBuffer){
   glDisable(GL_DEPTH_TEST);
  }
  if (_transparent) {
    glPushAttrib(GL_COLOR_BUFFER_BIT); // store current state of blend and blendfunc
    glEnable(GL_BLEND);       // Enable transparency
    glBlendFunc(GL_ONE, GL_ONE);//GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);//GL_ONE_MINUS_DST_COLOR, GL_ZERO);
  }
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
  glBindTexture(GL_TEXTURE_2D, _currentTexture);

  //GLboolean hasLight = glIsEnabled(GL_LIGHTING);
  //if (hasLight) glDisable(GL_LIGHTING);
  glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5*_width, -0.5*_height, 0.f);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5*_width, -0.5*_height, 0.f);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5*_width, 0.5*_height, 0.f);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5*_width, 0.5*_height, 0.f);
  glEnd();
  //if (hasLight) glEnable(GL_LIGHTING);

  if (_transparent) {
    glDisable(GL_BLEND);       // Disable transparency
    glPopAttrib();             // restore previous state
  }
  if(_disableDepthBuffer){
   glEnable(GL_DEPTH_TEST);
  }
  glDisable(GL_TEXTURE_2D);  // Disable Texture Mapping
}

void TexturedPlane::setImage(const QImage& image)
{
  QImage tex1 = QGLWidget::convertToGLFormat(image);

  glBindTexture(GL_TEXTURE_2D, _currentTexture);
  if(!_compressTexture){
    gluBuild2DMipmaps( GL_TEXTURE_2D, GL_RGBA,  tex1.width(), tex1.height(), GL_RGBA, GL_UNSIGNED_BYTE, tex1.bits());
    //glTexImage2D(GL_TEXTURE_2D, 0, 4, tex1.width(), tex1.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex1.bits());
  } else {
    gluBuild2DMipmaps( GL_TEXTURE_2D, GL_COMPRESSED_RGBA_ARB,  tex1.width(), tex1.height(), GL_RGBA, GL_UNSIGNED_BYTE, tex1.bits());
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_COMPRESSED_RGBA_ARB, tex1.width(), tex1.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex1.bits());
  }
  _haveTexture = true;
}

void TexturedPlane::writeVrml(std::ostream& os) const
{
  std::cerr << __PRETTY_FUNCTION__ << " Not implemented yet.\n";
  os << "# TexturedPlane" << std::endl;
  os << "# " << __PRETTY_FUNCTION__ << " Not implemented yet.\n";
}

void TexturedPlane::setImageRGBA(const uchar* imageData, unsigned int width, unsigned int height)
{
  if (imageData == NULL) {
    _haveTexture = false;
    return;
  }

  glBindTexture(GL_TEXTURE_2D, _currentTexture);
  gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB,  width, height, GL_RGBA, GL_UNSIGNED_BYTE, imageData);
  //glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, imageData); // Texture Generation From RGB Bitmap
  //glTexImage2D(GL_TEXTURE_2D, 0, 4, 1024, 512, 0, GL_RGBA, GL_UNSIGNED_BYTE, _currentImage); // Texture Generation From RGBA Bitmap

  _haveTexture = true;
}

void TexturedPlane::setSize(double width, double height)
{
  _width = width;
  _height = height;
}

} // end namespace
