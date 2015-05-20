// ALUFR PCL TOOLS
// Copyright (C) 2011 M. Ruhnke, R. Kuemmerle, B. Steder, W. Burgard
// 
// ALUFR PCL TOOLS is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// ALUFR PCL TOOLS is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

///windows compatibility
#ifdef _MSC_VER
  #include <windows.h>
#endif 

#include <GL/gl.h>
#include <GL/glu.h>
#include "alu_glviewer.h"
#include "alu_glviewer_object.h"
#include <algorithm>
#include <vector>
#include <iomanip> 
#include "standard_camera.h"
#include "basics/transformationRepresentation.h"
#include "basics/filesys_tools.h"
#include "visualization/imageWidget/imageWidget.h"

namespace Ais3dTools {

/**
 * \brief handle the GLU quadratic
 */
class GLUWrapper
{
  public:
    static GLUquadricObj* getQuadradic()
    {
      static GLUWrapper inst;
      return inst._quadratic;
    }
  protected:
    GLUWrapper()
    {
      _quadratic = gluNewQuadric();              // Create A Pointer To The Quadric Object ( NEW )
      gluQuadricNormals(_quadratic, GLU_SMOOTH); // Create Smooth Normals ( NEW )
    }
    ~GLUWrapper()
    {
      gluDeleteQuadric(_quadratic);
    }
    GLUquadricObj* _quadratic;;
};

  ALUGLViewer::ALUGLViewer(QWidget* widget, const QGLWidget* shareWidget, Qt::WFlags flags) : QGLViewer(widget, shareWidget, flags), mode_(FREE){
    (void) widget;
    init();
  }
  
  ALUGLViewer::~ALUGLViewer(){
  }
  
  void ALUGLViewer::init(){

    scale_[0] = 1.0;
    scale_[1] = 1.0;
    scale_[2] = 1.0;

    // replace camera
    qglviewer::Camera* oldcam = camera();
    qglviewer::Camera* cam = new Ais3dTools::StandardCamera();
    cam->setPosition(qglviewer::Vec(0., 0., 10.));
    cam->setUpVector(qglviewer::Vec(0., 1., 0.));
    cam->lookAt(qglviewer::Vec(0., 0., 0.));
    setCamera(cam);
    delete oldcam;

    setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
    setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);

    setStateFileName(QString::null);  // QGL viewer should not write an XML file to disk
  
    // keyboard shortcuts
    setShortcut(CAMERA_MODE, 0);
    setShortcut(EXIT_VIEWER, 0);
    setShortcut(SAVE_SCREENSHOT, 0);

//     setMouseBinding(Qt::LeftButton,  CAMERA, ROTATE);
//     setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
//     setMouseBinding(Qt::LeftButton,  ALIGN_CAMERA, true);
//     setMouseBinding(Qt::RightButton, CENTER_SCENE, true);

    screenShotStartTime_ = 0.0;
    screenShotFrameRate_ = 10.0;
    noOfWrittenImages_ = 0;

    drawCoordinateSystem_ = false;
  }

  void ALUGLViewer::draw(){
    glScalef(scale_[0], scale_[1], scale_[2]);
    int id = 1;
    makeCurrent();
    for(std::vector<ALUGLViewerObject* >::iterator it = children_.begin(); it != children_.end(); ++it){
      (*it)->preDraw();
      glPushName(id);
        (*it)->drawWithChildren();
        id++;
      glPopName();
      (*it)->postDraw();
    }
    if (drawCoordinateSystem_)
      drawAxis(1.0f);
  }

  void ALUGLViewer::drawWithNames(){
    for (size_t i = 0; i < children_.size(); ++i) {
      glPushName(i);
      children_[i]->draw();
      glPopName();
    }
  }

  void ALUGLViewer::add(ALUGLViewerObject* child){
    child->viewer_.push_back(this);
    children_.push_back(child);
    //NOTE: works only for the first level of the hierachy of GLViewerObjects!!!!!
    QObject* o = dynamic_cast<QObject*>(child);
    this->installEventFilter(o);
  }

  void ALUGLViewer::remove(ALUGLViewerObject* child){
    std::vector<ALUGLViewerObject* >::iterator it = std::find (children_.begin(), children_.end(), child);
    if (it != children_.end())
      children_.erase(it);
  }

  bool ALUGLViewer::getPoseFromGl(int x, int y, double& wx, double& wy, double& wz)
  {

    QPoint p(x,y);
    qglviewer::Vec orig, dir;
    camera()->convertClickToLine(p, orig, dir);

    if(mode_ == FREE){
      camera()->convertClickToLine(p, orig, dir);
      bool found;
      qglviewer::Vec  selectedPoint = camera()->pointUnderPixel(p, found);
//       double t = -orig.z / dir.z;
      if(found){
       wx = selectedPoint[0];
       wy = selectedPoint[1];
       wz = selectedPoint[2];
      }
//       std::cerr << "selectedPoint: (" << selectedPoint[0] << ", " << selectedPoint[1] << ", " << selectedPoint[2] << ")" << std::endl;
    }
    if(mode_ == XYPLANAR){
      double t = -orig.z / dir.z;
      wx = orig.x+dir.x*t;
      wy = orig.y+dir.y*t;
      wz = 0.0;
      return true;
    }
    if(mode_ == XZPLANAR){
      double t = -orig.y / dir.y;
      wx = orig.x+dir.x*t;
      wy = 0.0;
      wz = orig.z+dir.z*t;
      return true;
    }

      camera()->convertClickToLine(p, orig, dir);
      bool found;
      qglviewer::Vec  _selectedPoint = camera()->pointUnderPixel(p, found);

      if(found){
       wx = _selectedPoint[0];
       wy = _selectedPoint[1];
       wz = _selectedPoint[2];
      }
//       std::cerr << "selectedPoint: (" << _selectedPoint[0] << ", " << _selectedPoint[1] << ", " << _selectedPoint[2] << ")" << std::endl;
      return found;
  }

  bool ALUGLViewer::getSelectedPoint(qglviewer::Vec& point)
  {
    if (selectedName() == -1)
      return false;
    point = selectedPoint_;
    return true;
  }
  
  void ALUGLViewer::postSelection(const QPoint& point)
  {
    if (selectedName() != -1) {
      qglviewer::Vec orig;
      qglviewer::Vec dir;
  
      // Compute orig and dir, used to draw a representation of the intersecting line
      camera()->convertClickToLine(point, orig, dir);
      bool found;
      selectedPoint_ = camera()->pointUnderPixel(point, found);
      QString message = QString().sprintf("Selected point (%.2f, %.2f, %.2f)",
          selectedPoint_.x, selectedPoint_.y, selectedPoint_.z);
      cerr << message.toStdString() << endl;
    }
  }

  void ALUGLViewer::setMode(Mode m){
    mode_ = m;
  }

  Mode& ALUGLViewer::getMode(){
     return mode_;
  }

  void ALUGLViewer::resizeEvent ( QResizeEvent * event )
  {
    QGLViewer::resizeEvent(event);
  }

  void ALUGLViewer::setUpVector(double x, double y, double z) {
    camera()->setUpVector(qglviewer::Vec(x, y, z));
  }

  void ALUGLViewer::lookAt(double x, double y, double z) {
    camera()->lookAt(qglviewer::Vec(x, y, z));
  }

  void ALUGLViewer::setOrientation(const qglviewer::Quaternion& q) {
    qglviewer::Quaternion world2Cam;
    world2Cam.setFromRotatedBasis(qglviewer::Vec(0,-1,0), qglviewer::Vec(0,0,1), qglviewer::Vec(-1,0,0));
    camera()->setOrientation(q * world2Cam);
  }

  void ALUGLViewer::getOrientation(qglviewer::Quaternion& q) const {
    qglviewer::Quaternion cam2World;
    cam2World.setFromRotatedBasis(qglviewer::Vec(0,0,-1), qglviewer::Vec(-1,0,0), qglviewer::Vec(0,1,0));
    q = camera()->orientation() * cam2World;
  }

  void ALUGLViewer::setOrientation(double roll, double pitch, double yaw) {
    qglviewer::Quaternion q;
    TransformationRepresentation::getQuaternionFromEuler(roll, pitch, yaw, q[3], q[0], q[1], q[2]);
    setOrientation(q);
  }

  void ALUGLViewer::setPosition(double x, double y, double z) {
    camera()->setPosition(qglviewer::Vec(x, y, z));
  }

  void ALUGLViewer::setPosition(qglviewer::Vec& pos, qglviewer::Quaternion& rotation)
  {
    camera()->setPosition(pos);
    setOrientation(rotation);
  }

  void ALUGLViewer::getPosition(double& x, double& y, double& z, double& roll, double& pitch, double& yaw)
  {
    qglviewer::Vec pos;
    qglviewer::Quaternion camOrient;
    getPosition(pos, camOrient);
    x     = pos[0];
    y     = pos[1];
    z     = pos[2];
    TransformationRepresentation::getEulerFromQuaternion(camOrient[3], camOrient[0], camOrient[1], camOrient[2], roll, pitch, yaw);
  }

  void ALUGLViewer::getPosition(float& x, float& y, float& z, float& roll, float& pitch, float& yaw)
  {
    double tmpX, tmpY, tmpZ, tmpRoll, tmpPitch, tmpYaw;
    getPosition(tmpX, tmpY, tmpZ, tmpRoll, tmpPitch, tmpYaw);
    x=tmpX; y=tmpY; z=tmpZ; roll=tmpRoll; pitch=tmpPitch; yaw=tmpYaw;
  }

  void ALUGLViewer::getPosition(qglviewer::Vec& pos, qglviewer::Quaternion& rotation)
  {
    pos = camera()->position();
    getOrientation(rotation);
  }

  void ALUGLViewer::setBackgroundColor(const QColor& color)
  {
    makeCurrent();
    QGLViewer::setBackgroundColor(color);
  }

  void ALUGLViewer::saveJpeg(const char* filename, int quality)
  {
    setSnapshotFormat(QString("JPEG"));
    setSnapshotQuality(quality);
    saveSnapshot(QString(filename), true);
  }

  void ALUGLViewer::saveTimedJpeg(double currentTime, const char* filename, int quality) {
    if (noOfWrittenImages_ == 0)
      screenShotStartTime_ = currentTime;
    
    int wantedNoOfImages = lrint((currentTime-screenShotStartTime_)*screenShotFrameRate_ + 1);
    
    if (wantedNoOfImages<noOfWrittenImages_) {
      cerr << __PRETTY_FUNCTION__ <<": Something with the timing is wrong (Playing a log or change of framerate). Resetting screenshot data. This will overwrite old images.\n";
      noOfWrittenImages_ = 0;
      screenShotStartTime_ = currentTime;
      return;
    }
    if (noOfWrittenImages_ < wantedNoOfImages) {  // Save enough screenshots to have a constant framerate
      std::stringstream fileNameWithNumber;
      fileNameWithNumber.fill('0');
      fileNameWithNumber << filename << std::setw(6) << noOfWrittenImages_+1 << ".jpg";
      saveJpeg(fileNameWithNumber.str().c_str(), quality);
      noOfWrittenImages_++;

      // copy the orig file several times to get proper framerate
      string prevFilename = fileNameWithNumber.str();
      while (noOfWrittenImages_ < wantedNoOfImages) {
        fileNameWithNumber.str("");
        fileNameWithNumber << filename << std::setw(6) << noOfWrittenImages_+1 << ".jpg";
        //cerr << PVAR(prevFilename) << " " << PVAR(fileNameWithNumber.str()) << endl;
        bool copyStatus = copyFile(prevFilename.c_str(), fileNameWithNumber.str().c_str());
        if (!copyStatus)
          cerr << __PRETTY_FUNCTION__ << ": Error while copying file" << endl;
        noOfWrittenImages_++;
      }
    }
  }

  void ALUGLViewer::savePng(const char* filename)
  {
    setSnapshotFormat(QString("PNG"));
    setSnapshotQuality(-1);
    saveSnapshot(QString(filename), true);
  }

  void ALUGLViewer::saveStereoPng(float x, float y, float z, const char* filename0, const char* filename1)
  {
    unsigned char* image,
                 * displacedImage;
    int imageWidth, imageHeight;
    getStereoImages(x, y, z, image, displacedImage, imageWidth, imageHeight);
    
    ImageWidget imageWidget;
    imageWidget.setRGBImage(image, imageWidth, imageHeight, false);
    imageWidget.savePng(filename0);
    imageWidget.setRGBImage(displacedImage, imageWidth, imageHeight, false);
    imageWidget.savePng(filename1);
    
    delete[] image;
    delete[] displacedImage;
  }

  void ALUGLViewer::saveTimedPng(double currentTime, const char* filename) {
    if (noOfWrittenImages_ == 0)
      screenShotStartTime_ = currentTime;
    
    int wantedNoOfImages = lrint((currentTime-screenShotStartTime_)*screenShotFrameRate_ + 1);
    
    if (wantedNoOfImages<noOfWrittenImages_) {
      cerr << __PRETTY_FUNCTION__ <<": Something with the timing is wrong (Playing a log or change of framerate). Resetting screenshot data. This will overwrite old images.\n";
      noOfWrittenImages_ = 0;
      screenShotStartTime_ = currentTime;
      return;
    }
    if (noOfWrittenImages_ < wantedNoOfImages) {  // Save enough screenshots to have a constant framerate
      std::stringstream fileNameWithNumber;
      fileNameWithNumber.fill('0');
      fileNameWithNumber << filename << std::setw(6) << noOfWrittenImages_+1 << ".png";
      savePng(fileNameWithNumber.str().c_str());
      noOfWrittenImages_++;

      // copy the orig file several times to get proper framerate
      string prevFilename = fileNameWithNumber.str();
      while (noOfWrittenImages_ < wantedNoOfImages) {
        fileNameWithNumber.str("");
        fileNameWithNumber << filename << std::setw(6) << noOfWrittenImages_+1 << ".png";
        //cerr << PVAR(prevFilename) << " " << PVAR(fileNameWithNumber.str()) << endl;
        bool copyStatus = copyFile(prevFilename.c_str(), fileNameWithNumber.str().c_str());
        if (!copyStatus)
          cerr << __PRETTY_FUNCTION__ << ": Error while copying file" << endl;
        noOfWrittenImages_++;
      }
    }
  }

  void ALUGLViewer::saveTimedStereoPng(float x, float y, float z, double currentTime, const char* filename) {
    if (noOfWrittenImages_ == 0)
      screenShotStartTime_ = currentTime;
    
    int wantedNoOfImages = lrint((currentTime-screenShotStartTime_)*screenShotFrameRate_ + 1);
    
    if (wantedNoOfImages<noOfWrittenImages_) {
      cerr << __PRETTY_FUNCTION__ <<": Something with the timing is wrong (Playing a log or change of framerate). Resetting screenshot data. This will overwrite old images.\n";
      noOfWrittenImages_ = 0;
      screenShotStartTime_ = currentTime;
      return;
    }
    if (noOfWrittenImages_ < wantedNoOfImages) {  // Save enough screenshots to have a constant framerate
      std::stringstream fileNameWithNumber0, fileNameWithNumber1;
      fileNameWithNumber0 << filename << "_0_" << setfill('0')<<std::setw(6)<<noOfWrittenImages_+1 << ".png";
      fileNameWithNumber1 << filename << "_1_" << setfill('0')<<std::setw(6)<<noOfWrittenImages_+1 << ".png";
      saveStereoPng(x, y, z, fileNameWithNumber0.str().c_str(), fileNameWithNumber1.str().c_str());
      noOfWrittenImages_++;

      // copy the orig file several times to get proper framerate
      string prevFilename0 = fileNameWithNumber0.str(),
             prevFilename1 = fileNameWithNumber1.str();
      while (noOfWrittenImages_ < wantedNoOfImages) {
        fileNameWithNumber0.str("");
        fileNameWithNumber0 << filename << "_0_" << setfill('0')<<std::setw(6)<<noOfWrittenImages_+1 << ".png";
        bool copyStatus = copyFile(prevFilename0.c_str(), fileNameWithNumber0.str().c_str());
        if (!copyStatus)
          cerr << __PRETTY_FUNCTION__ << ": Error while copying file" << endl;
        fileNameWithNumber1.str("");
        fileNameWithNumber1 << filename << "_1_" << setfill('0')<<std::setw(6)<<noOfWrittenImages_+1 << ".png";
        copyStatus = copyFile(prevFilename1.c_str(), fileNameWithNumber1.str().c_str());
        if (!copyStatus)
          cerr << __PRETTY_FUNCTION__ << ": Error while copying file" << endl;
        ++noOfWrittenImages_;
      }
    }
  }

  unsigned char* ALUGLViewer::getRGBImage()
  {
    makeCurrent();
    unsigned char* rgbImage = new unsigned char[3*width()*height()];
    unsigned char* imageLine = new unsigned char[4*width()];
    unsigned char* rgbImagePtr = rgbImage;
    for (int y=0; y<int(height()); ++y) {
      glReadPixels(0, int(height())-1-y, width(), 1, GL_RGBA, GL_UNSIGNED_BYTE, imageLine);
      unsigned char* imageLinePtr = imageLine;
      for (int pixelIdx=0; pixelIdx<width(); ++pixelIdx) {
        *(rgbImagePtr++) = *(imageLinePtr++);
        *(rgbImagePtr++) = *(imageLinePtr++);
        *(rgbImagePtr++) = *(imageLinePtr++);
        ++imageLinePtr;
      }
    }
    delete[] imageLine;
    return rgbImage;
  }

  unsigned char* ALUGLViewer::getTopBottomStereoImage(float x, float y, float z, int& imageWidth, int& imageHeight)
  {
    unsigned char* image,
                 * displacedImage;
    getStereoImages(x, y, z, image, displacedImage, imageWidth, imageHeight);
    
    unsigned char* rgbImage = new unsigned char[4*3*imageWidth*imageHeight];
    
    unsigned char* rgbImagePtr = rgbImage;
    for (int imageIdx=0; imageIdx<2; ++imageIdx) {
      unsigned char* imagePtr = (imageIdx==0 ? image : displacedImage);
      for (int pixelIdx=0; pixelIdx<imageWidth*imageHeight; ++pixelIdx) {
        *(rgbImagePtr++) = *(imagePtr++);
        *(rgbImagePtr++) = *(imagePtr++);
        *(rgbImagePtr++) = *imagePtr;
        imagePtr -= 2;
        *(rgbImagePtr++) = *(imagePtr++);
        *(rgbImagePtr++) = *(imagePtr++);
        *(rgbImagePtr++) = *(imagePtr++);
      }
    }
    imageWidth  *= 2,
    imageHeight *= 2;
    
    delete[] image;
    delete[] displacedImage;
    
    return rgbImage;
  }

  unsigned char* ALUGLViewer::getLeftRightStereoImage(float x, float y, float z, int& imageWidth, int& imageHeight)
  {
    unsigned char* image,
                 * image2;
    getStereoImages(x, y, z, image, image2, imageWidth, imageHeight);
    
    unsigned char* rgbImage = new unsigned char[12*imageWidth*imageHeight];
    
    for (int imageY=0; imageY<imageHeight; ++imageY) {
      for (int imageX=0; imageX<imageWidth; ++imageX) {
        int srcIdx = imageY*3*imageWidth + 3*imageX,
            dstIdx = imageY*12*imageWidth + 3*imageX;
        rgbImage[dstIdx]=image[srcIdx], rgbImage[dstIdx+1]=image[srcIdx+1], rgbImage[dstIdx+2]=image[srcIdx+2];
        dstIdx += 3*imageWidth;
        rgbImage[dstIdx]=image2[srcIdx], rgbImage[dstIdx+1]=image2[srcIdx+1], rgbImage[dstIdx+2]=image2[srcIdx+2];
        dstIdx += 3*imageWidth;
        rgbImage[dstIdx]=image[srcIdx], rgbImage[dstIdx+1]=image[srcIdx+1], rgbImage[dstIdx+2]=image[srcIdx+2];
        dstIdx += 3*imageWidth;
        rgbImage[dstIdx]=image2[srcIdx], rgbImage[dstIdx+1]=image2[srcIdx+1], rgbImage[dstIdx+2]=image2[srcIdx+2];
      }
    }
    imageWidth  *= 2,
    imageHeight *= 2;
    
    delete[] image;
    delete[] image2;
    
    return rgbImage;
  }

  void ALUGLViewer::getStereoImages(float x, float y, float z,
      unsigned char*& image, unsigned char*& displacedImage,
      int& imageWidth, int& imageHeight)
  {
    imageWidth  = width(),
    imageHeight = height();
    
    image = new unsigned char[3*imageWidth*imageHeight];
    displacedImage = new unsigned char[3*imageWidth*imageHeight];
    
    qglviewer::Vec originalPos;
    qglviewer::Quaternion originalOrient;
    getPosition(originalPos, originalOrient);
    
    qglviewer::Vec newPos = originalOrient*qglviewer::Vec(x,y,z) + originalPos;
    qglviewer::Quaternion newOrient = originalOrient;
    setPosition(newPos, newOrient);
    updateGL();

    unsigned char* imageLine = new unsigned char[4*width()];
    unsigned char* imagePtr = displacedImage;
    for (int imageY=0; imageY<int(height()); imageY+=1) {
      glReadPixels(0, int(height())-1-imageY, width(), 1, GL_RGBA, GL_UNSIGNED_BYTE, imageLine);
      unsigned char* imageLinePtr = imageLine;
      for (int pixelIdx=0; pixelIdx<width(); ++pixelIdx) {
        *(imagePtr++) = *(imageLinePtr++);
        *(imagePtr++) = *(imageLinePtr++);
        *(imagePtr++) = *(imageLinePtr++);
        ++imageLinePtr;
      }
    }

    setPosition(originalPos, originalOrient);
    updateGL();

    imagePtr = image;
    for (int imageY=0; imageY<int(height()); imageY+=1) {
      glReadPixels(0, int(height())-1-imageY, width(), 1, GL_RGBA, GL_UNSIGNED_BYTE, imageLine);
      unsigned char* imageLinePtr = imageLine;
      for (int pixelIdx=0; pixelIdx<width(); ++pixelIdx) {
        *(imagePtr++) = *(imageLinePtr++);
        *(imagePtr++) = *(imageLinePtr++);
        *(imagePtr++) = *(imageLinePtr++);
        ++imageLinePtr;
      }
    }
    
    delete[] imageLine;
  }

} //end namespace
