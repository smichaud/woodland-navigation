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

#ifndef ALUFR_GLVIEWER_H
#define ALUFR_GLVIEWER_H

#include "basics/macros.h"
#include <QGLViewer/qglviewer.h>
#include <vector>

namespace Ais3dTools {
  /** Forward declaration */
  class ALUGLViewerObject;

  enum Mode{FREE, XYPLANAR, XZPLANAR, YZPLANAR};

  class ALUGLViewer: public QGLViewer
  {
    Q_OBJECT
    public:
      ALUGLViewer(QWidget* widget=NULL, const QGLWidget* shareWidget=NULL, Qt::WFlags flags=0);
      ~ALUGLViewer();

      void init();

      /** \brief draws all added children */
      virtual void draw();

      virtual void drawWithNames();

      /** \brief adds ALUGLViewerObject as children to this viewer */
      void add(ALUGLViewerObject* child);

      /** \brief removes ALUGLViewerObject children from this viewer */
      void remove(ALUGLViewerObject* child);

      /** x/y mouse coordinate to 3D GL coordinate calculation */
      bool getPoseFromGl(int x, int y, double& wx, double& wy, double& wz);
      bool getSelectedPoint(qglviewer::Vec& point);
      void postSelection(const QPoint& point);
      void setDrawCoordinateSystem(bool drawCoordinateSystem) { drawCoordinateSystem_=drawCoordinateSystem; }

      /** \brief set camera mode FREE or constrained (XYPLANAR, XZPLANAR, YZPLANAR) */
      void setMode(Mode m);
      /** \brief gets the current camera mode */
      Mode& getMode();

      void resizeEvent(QResizeEvent* event);

      //! set the up vector of the camera
      void setUpVector(double x, double y, double z);
      
      //!specify the point the camera looks at
      void lookAt(double x, double y, double z);

      //!specify the orientation of the camera (In the world system)
      void setOrientation(const qglviewer::Quaternion& q);

      //!get the orientation of the camera (In the world system)
      void getOrientation(qglviewer::Quaternion& q) const;

      //!specify the orientation of the camera (In the world system)
      void setOrientation(double roll, double pitch, double yaw);
      
      //!set the position of the camera
      void setPosition(double x, double y, double z);

      //!set the position of the camera
      void setPosition(qglviewer::Vec& pos, qglviewer::Quaternion& rotation);
      
      //! set the position around which the orientation revolves
      void setSceneCenter(double x, double y, double z) { camera()->setSceneCenter(qglviewer::Vec(x, y, z));}

      //! Get the pose of the virtual observer */
      void getPosition(double& x, double& y, double& z, double& roll, double& pitch, double& yaw);

      //! Get the pose of the virtual observer */
      void getPosition(float& x, float& y, float& z, float& roll, float& pitch, float& yaw);

      //! get the position of the camera
      void getPosition(qglviewer::Vec& pos, qglviewer::Quaternion& rotation);
      
      //! overwrite set background from base class
      void setBackgroundColor(const QColor& color);

      //! Project the 3D point into 2D screen coordinates
      template <typename real>
      inline void project3dTo2D(real inputX, real inputY, real inputZ, real& outputX, real& outputY) const {
        qglviewer::Vec output = camera()->projectedCoordinatesOf(qglviewer::Vec(inputX, inputY, inputZ));
        outputX=output[0], outputY=output[1];
      }
      
      //! Project the 3D point into 2D screen coordinates and round to int
      template <typename real>
      inline void project3dTo2D(real inputX, real inputY, real inputZ, int& outputX, int& outputY) const {
        real realX, realY;
        project3dTo2D(inputX, inputY, inputZ, realX, realY);
        outputX=lrint(realX), outputY=lrint(realY);
      }


      /**
       * save an image of the displayed stuff as a JPEG.
       * Maybe you want to hide the mainmenu before doing this.
       */
      void saveJpeg(const char* filename, int quality = 75);

      void setScreenShotFrameRate(double fps) { screenShotFrameRate_=fps; }

      /**
       * If called regularly, this function saves Jpegs of the window content
       * at a constant framerate (Use setScreenShotFrameRate to change this value).
       * The number of the current image is automatically added to the given filename
       * (which should be given without an extension). 
       * Maybe you want to hide the mainmenu when using this.
       */
      void saveTimedJpeg(double currentTime, const char* filename="viewer", int quality = 75);

      /**
       * save an image of the displayed stuff as a PNG.
       * Maybe you want to hide the mainmenu before doing this.
       */
      void savePng(const char* filename);

      /**
       * Same as above, but saving stereo images.
       */
      void saveStereoPng(float x, float y, float z, const char* filename0, const char* filename1);

      /**
       * If called regularly, this function saves Pngs of the window content
       * at a constant framerate (Use setScreenShotFrameRate to change this value).
       * The number of the current image is automatically added to the given filename
       * (which should be given without an extension). 
       * Maybe you want to hide the mainmenu when using this.
       */
      void saveTimedPng(double currentTime, const char* filename="viewer");

      /**
       * Same as above, but saving stereo images.
       */
      void saveTimedStereoPng(float x, float y, float z, double currentTime, const char* filename="viewer");

      /**
       * Grabs the current widget into an RGB image buffer
       */
      unsigned char* getRGBImage();

      /**
       * Grabs a top/bottom stereo image with x,y,z as the displacement (RGB image buffer)
       * Warning: Viewer 'flickers' once to the other perspective, when using this fundtion.
       */
      unsigned char* getTopBottomStereoImage(float x, float y, float z, int& imageWidth, int& imageHeight);

      /**
       * Grabs a left/right stereo image with x,y,z as the displacement (RGB image buffer)
       * Warning: Viewer 'flickers' once to the other perspective, when using this fundtion.
       */
      unsigned char* getLeftRightStereoImage(float x, float y, float z, int& imageWidth, int& imageHeight);
      
      /**
       * Grabs stereo images with x,y,z as the displacement (RGB image buffer)
       * Warning: Viewer 'flickers' once to the other perspective, when using this fundtion.
       */
      void getStereoImages(float x, float y, float z,
                           unsigned char*& image, unsigned char*& displacedImage,
                           int& imageWidth, int& imageHeight);

      
      void setMouseBindingCAMERA(int state, MouseAction action, bool withConstraint = true){
	QGLViewer::setMouseBinding(state, QGLViewer::CAMERA, action, withConstraint);
      }
    protected:
      /** Member */
      std::vector<ALUGLViewerObject* > children_;
      Mode mode_;
      qglviewer::Vec  selectedPoint_;
      qglviewer::Vec  scale_;
      double screenShotStartTime_;
      double screenShotFrameRate_;
      int noOfWrittenImages_;
      bool drawCoordinateSystem_;
  };

}

#endif
