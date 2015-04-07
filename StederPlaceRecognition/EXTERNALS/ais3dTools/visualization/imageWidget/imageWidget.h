#ifndef IMAGE_WIDGET_H
#define IMAGE_WIDGET_H

#include <iostream>
using std::cout;
#include <limits>
#include <vector>
using std::vector;
#include <cmath>
#include <QWidget>
#include <QImage>
#include <QPen>
#include <QPainter>
#include "basics/macros.h"

namespace Ais3dTools {

/**
 * \brief Simple widget to show images
 *
 * This file specifies a qt widget that shows a given image
 * in a seperate window
 **/
class ImageWidget : public QWidget {
  Q_OBJECT
  
  public:
    ImageWidget(QWidget* parent = 0, Qt::WindowFlags fl=0);
    ~ImageWidget();
    
    void reset();

    void resetAllButImages();

    void setGreyscaleImage(const unsigned char* data, unsigned int width, unsigned int height, bool showImageNow=true);
    
    void setBoolImage(const bool* data, unsigned int width, unsigned int height, bool showImageNow=true);

    void setRGBImage(const unsigned char* data, unsigned int width, unsigned int height, bool showImageNow=true);
    
    void setBGRAImage(const unsigned char* data, unsigned int width, unsigned int height, bool showImageNow=true);

    void setBayerGRBGImage(const unsigned char* data, unsigned int width, unsigned int height, bool showImageNow=true);
    
    void setYuv422Image(const unsigned char* data, unsigned int width, unsigned int height, bool showImageNow=true);
    
    void setRGBPointersImage(const unsigned char** data, unsigned int width, unsigned int height, bool showImageNow=true);
    
    void setQImage(const QImage& qImage, bool showImageNow=true);

    template <typename real>
    void setRealImage(const real* data, unsigned int width, unsigned int height, bool showImageNow=true,
                      real minValue=1, real maxValue=0, QImage* legend=NULL);

    template <typename real>
    void setAngleImage(const real* data, unsigned int width, unsigned int height, bool showImageNow=true);

    void showImage() { show(); rescaleImage(); update();}
    
    void markPoint(float x, float y, const QPen& pen=QPen(Qt::green, 2)) { markedPoints.push_back(ImagePoint(x,y, pen));}

    void markLine (float x1, float y1, float x2, float y2, const QPen& pen=QPen(Qt::blue, 2)) { markedLines.push_back(ImageLine(x1,y1,x2,y2,pen));}

    void removeMarkedPoints() { markedPoints.clear();}

    void removeMarkedLines() { markedLines.clear();}
    
    void savePng(const char* fileName="imageWidget.png") const { image.save(fileName, "PNG");}
    bool save(const char* fileName) const;
    
    template <typename real>  // value has to be in [0.0, 1.0]
    static inline void getColorForReal(real value, unsigned char& r, unsigned char& g, unsigned char& b, bool grayScale);
    
    template <typename real>  // value has to be in (-pi,pi]
    static inline void getColorForAngle(real value, unsigned char& r, unsigned char& g, unsigned char& b);

    const QImage& getImage() { return image; }
    const QImage& getScaledImage() { return imageScaled; }

    //! The return value is created with new[] and the user has to make sure to delete[] it again.
    unsigned char* getRGB(int& width, int& height) const;
    
    void setShowCurrentPixelPosition(bool showCurrentPixelPosition);

    bool useOriginalSize;
    bool keepAspectRatio;
    bool showRealImagesInGrayScales;
    bool useFastScaling;
    
    bool printSelectedPoint;
    float clickedPixelX, clickedPixelY;
    bool mouseClickHappened;

  public slots:
    void resizeEvent(QResizeEvent *event);
    void paintEvent(QPaintEvent *event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);

  signals:
    void mouseLeftClicked(float x, float y);
    
  protected:
    void rescaleImage();
    void drawMarkedPoints();
    bool showCurrentPixelPosition;
    
    QImage image, imageScaled;
    unsigned char* imageData;
    uint lastImageSize;
    
    struct ImagePoint {
      ImagePoint(float x, float y, const QPen& pen=QPen(Qt::green, 2)) : x(x), y(y), pen(pen) {}
      float x,y;
      QPen pen;
    };
    vector<ImagePoint> markedPoints;
    
    struct ImageLine 
    {
      ImageLine (float x1, float y1, float x2, float y2, const QPen& pen=QPen(Qt::green, 1)) : x1(x1), y1(y1), x2(x2), y2(y2), pen(pen) {}
      float x1, y1, x2, y2;
      QPen pen;
    };
    vector<ImageLine> markedLines;
};

} // namespace

#include "imageWidget.hpp"  // Definitions of inline&templated functions

#endif
