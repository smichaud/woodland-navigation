#include "imageWidget.h"
#include <QMouseEvent>
#include <QToolTip>
#include "basics/macros.h"

namespace Ais3dTools {

ImageWidget::ImageWidget(QWidget* parent, Qt::WindowFlags fl) :
  QWidget(parent, fl), useOriginalSize(false), keepAspectRatio(true), showRealImagesInGrayScales(false),
  useFastScaling(false), printSelectedPoint(false), showCurrentPixelPosition(false), imageData(NULL), lastImageSize(0)
{
  reset();
  hide();
}

ImageWidget::~ImageWidget() {
  reset();
}

void ImageWidget::reset() {
  resetAllButImages();
  delete[] imageData; imageData=NULL;
  image = imageScaled = QImage();
  lastImageSize = 0;
}

void ImageWidget::resetAllButImages() {
  mouseClickHappened = false;
  clickedPixelX = clickedPixelY = -1.0f;
  markedPoints.clear();
  markedLines.clear();
}

void ImageWidget::rescaleImage() {
  if (!image.isNull()) {
    if (useOriginalSize) {
      imageScaled = image;
      setFixedSize(imageScaled.width(), imageScaled.height());
    }
    else {
      Qt::AspectRatioMode aspectRatioMode = (keepAspectRatio ? Qt::KeepAspectRatio : Qt::IgnoreAspectRatio);
      Qt::TransformationMode transformMode = (useFastScaling ? Qt::FastTransformation : Qt::SmoothTransformation);
      imageScaled = image.scaled(width(), height(), aspectRatioMode, transformMode);
      if (width()!=imageScaled.width() || height()!=imageScaled.height()) {
        // keep aspect ration of window
        resize(imageScaled.width(), imageScaled.height());
      }
      //cout << PVARC(this->frameSize().width()) << PVARC(this->frameSize().height())
           //<< PVARC(this->minimumWidth()) << PVARC(this->minimumHeight())
           //<< PVARC(parentWidget()->width()) << PVARC(parentWidget()->height())
           //<< PVARC(imageScaled.width()) << PVARC(imageScaled.height())
           //<< PVARC(this->width()) << PVARN(this->height());
    }
  }
}

void ImageWidget::resizeEvent(QResizeEvent *event) {
  rescaleImage();
  update();
  QWidget::resizeEvent(event);
}

void ImageWidget::paintEvent(QPaintEvent *event)
{
  QWidget::paintEvent(event);
  if (!image.isNull()) {
    QPainter painter(this);
    painter.drawImage(0,0, imageScaled);
  }
  drawMarkedPoints();
}

void ImageWidget::setGreyscaleImage(const unsigned char* data, unsigned int width, unsigned int height, bool showImageNow) {
  if (data == NULL) {
    reset();
    return;
  }
  resetAllButImages();
  unsigned int size = width*height;
  if (size != lastImageSize) {
    delete[] imageData; imageData = new unsigned char[4*size];
    lastImageSize = size;
  }

  const unsigned char* srcPointer = data;
  unsigned char* dstPointer = imageData;
  
  for (unsigned int i=0; i<size; i++) {
    *(dstPointer++) = *srcPointer;
    *(dstPointer++) = *srcPointer;
    *(dstPointer++) = *(srcPointer++);
    *(dstPointer++) = 255;
  }
  
  image = QImage(imageData, width, height, QImage::Format_RGB32);
  if (showImageNow) showImage();
}

void ImageWidget::setBoolImage(const bool* data, unsigned int width, unsigned int height, bool showImageNow) {
  if (data == NULL) {
    reset();
    return;
  }
  resetAllButImages();
  unsigned int size = width*height;
  if (size != lastImageSize) {
    delete[] imageData; imageData = new unsigned char[4*size];
    lastImageSize = size;
  }
  
  const bool* srcPointer = data;
  unsigned char* dstPointer = imageData;
  
  for (unsigned int i=0; i<size; i++) {
    if (*(srcPointer++)) {
      *(dstPointer++) = 0; *(dstPointer++) = 0; *(dstPointer++) = 0;
    }
    else {
      *(dstPointer++) = 255; *(dstPointer++) = 255; *(dstPointer++) = 255;
    }
    *(dstPointer++) = 255;
  }
  
  image = QImage(imageData, width, height, QImage::Format_RGB32);
  if (showImageNow) showImage();
}

void ImageWidget::setRGBImage(const unsigned char* data, unsigned int width, unsigned int height, bool showImageNow) {
  if (data == NULL) {
    reset();
    return;
  }
  resetAllButImages();
  unsigned int size = width*height;
  if (size != lastImageSize) {
    delete[] imageData; imageData = new unsigned char[4*size];
    lastImageSize = size;
  }
  
  const unsigned char* srcPointer = data;
  unsigned char* dstPointer = imageData;
  
  for (unsigned int i=0; i<size; i++) {
    *(dstPointer++) = *(srcPointer+2);
    *(dstPointer++) = *(srcPointer+1);
    *(dstPointer++) = *srcPointer;
    *(dstPointer++) = 255;
    srcPointer += 3;
  }
  
  image = QImage(imageData, width, height, QImage::Format_RGB32);
  if (showImageNow) showImage();
}

void ImageWidget::setBayerGRBGImage(const unsigned char* data, unsigned int width, unsigned int height, bool showImageNow) {
 if (data == NULL) {
    reset();
    return;
  }
  resetAllButImages();
  unsigned int size = width*height;
  if (size != lastImageSize) {
    delete[] imageData; imageData = new unsigned char[4*size];
    lastImageSize = size;
  }
  
  for (unsigned int y=0; y<height-2; y+=2) {
    for (unsigned int x=0; x<width-2; x+=2) {
      const unsigned char* input  = data+y*width+x;
      unsigned char* output = imageData+4*(y*width+x);
      *(output++)=input[width], *(output++)=(int(input[0])+int(input[width+1]))/2, *(output++)=input[1], *(output++)=255;
      *(output++)=input[width+2], *(output++)=(int(input[2])+int(input[width+1]))/2, *(output++)=input[1], *(output++)=255;
      output += 4*width-4;
      *(output++)=input[width], *(output++)=(int(input[width+1])+int(input[2*width]))/2, *(output++)=input[2*width+1], *(output++)=255;
      *(output++)=input[width+2], *(output++)=(int(input[width+1])+int(input[2*width+3]))/2, *(output++)=input[2*width+1], *(output++)=255;
    }
  }
  // last 2 columns
  for (unsigned int y=0; y<height; y+=2) {
    for (unsigned int x=width-2; x<width; x+=2) {
      const unsigned char* input  = data+y*width+x;
      unsigned char* output = imageData+4*(y*width+x);
      unsigned char r=input[1], g=(int(input[0])+int(input[width+1]))/2, b=input[width];
      *(output++)=b, *(output++)=g, *(output++)=r, *(output++)=255;
      *(output++)=b, *(output++)=g, *(output++)=r, *(output++)=255;
      output += 4*width-4;
      *(output++)=b, *(output++)=g, *(output++)=r, *(output++)=255;
      *(output++)=b, *(output++)=g, *(output++)=r, *(output++)=255;
    }
  }
  // last 2 lines
  for (unsigned int y=height-2; y<height; y+=2) {
    for (unsigned int x=0; x<width-2; x+=2) {
      const unsigned char* input  = data+y*width+x;
      unsigned char* output = imageData+4*(y*width+x);
      unsigned char r=input[1], g=(int(input[0])+int(input[width+1]))/2, b=input[width];
      *(output++)=b, *(output++)=g, *(output++)=r, *(output++)=255;
      *(output++)=b, *(output++)=g, *(output++)=r, *(output++)=255;
      output += 4*width-4;
      *(output++)=b, *(output++)=g, *(output++)=r, *(output++)=255;
      *(output++)=b, *(output++)=g, *(output++)=r, *(output++)=255;
    }
  }

  image = QImage(imageData, width, height, QImage::Format_RGB32);
  if (showImageNow) showImage();
  
  //unsigned char* rgbData = getRGBFromBayerGRBG(data, width, height);
  //setRGBImage(rgbData, width, height, showImageNow);
  //delete[] rgbData;
}

inline unsigned char clipChar(int value) {
  return (value<0 ? 0 : (value>255 ? 255 : value));
}
void ImageWidget::setYuv422Image(const unsigned char* data, unsigned int width, unsigned int height, bool showImageNow) {
 if (data == NULL) {
    reset();
    return;
  }
  resetAllButImages();
  unsigned int size = width*height;
  if (size != lastImageSize) {
    delete[] imageData; imageData = new unsigned char[4*size];
    lastImageSize = size;
  }
  
  const unsigned char* srcPointer = data;
  unsigned char* dstPointer = imageData;
  
  for (unsigned int i=0; i<size; i+=2) {
    int u  = *(srcPointer++)-128,
        y1 = *(srcPointer++),
        v  = *(srcPointer++)-128,
        y2 = *(srcPointer++);
    unsigned char& b1 = *(dstPointer++),
                 & g1 = *(dstPointer++),
                 & r1 = *(dstPointer++),
                 & a1 = *(dstPointer++),
                 & b2 = *(dstPointer++),
                 & g2 = *(dstPointer++),
                 & r2 = *(dstPointer++),
                 & a2 = *(dstPointer++);
    a1 = a2 = 255;
    r1 = clipChar(y1 + ((v*18678 + 8192 ) >> 14));
    g1 = clipChar(y1 + ((v * -9519 - u * 6472 + 8192 ) >> 14));
    b1 = clipChar(y1 + ((u * 33292 + 8192 ) >> 14));
    r2 = clipChar(y2 + ((v*18678 + 8192 ) >> 14));
    g2 = clipChar(y2 + ((v * -9519 - u * 6472 + 8192 ) >> 14));
    b2 = clipChar(y2 + ((u * 33292 + 8192 ) >> 14));
  }
  image = QImage(imageData, width, height, QImage::Format_RGB32);
  if (showImageNow) showImage();
}

void ImageWidget::setRGBPointersImage(const unsigned char** data, unsigned int width, unsigned int height, bool showImageNow) {
  if (data == NULL) {
    reset();
    return;
  }
  resetAllButImages();
  unsigned int size = width*height;
  if (size != lastImageSize) {
    delete[] imageData; imageData = new unsigned char[4*size];
    lastImageSize = size;
  }
  
  const unsigned char** srcPointer = data;
  const unsigned char* srcPointer2 = NULL;
  unsigned char* dstPointer = imageData;
  
  for (unsigned int i=0; i<size; ++i) {
    if (*srcPointer != NULL) {
      srcPointer2 = *srcPointer;
      *(dstPointer++) = *(srcPointer2+2);
      *(dstPointer++) = *(srcPointer2+1);
      *(dstPointer++) = *srcPointer2;
      *(dstPointer++) = 255;
    }
    else {
      *(dstPointer++) = 200;
      *(dstPointer++) = 150;
      *(dstPointer++) = 150;
      *(dstPointer++) = 255;
    }
    ++srcPointer;
  }
  
  image = QImage(imageData, width, height, QImage::Format_RGB32);
  if (showImageNow) showImage();
}

void ImageWidget::setQImage(const QImage& qImage, bool showImageNow) {
  reset();
  delete[] imageData;  imageData=NULL;  lastImageSize=0;  // These are not used if the source is another QImage
  image = qImage;
  if (showImageNow) showImage();
}

void ImageWidget::drawMarkedPoints() {
  QPainter painter(this);
  int rectSize = 3;
  if (!image.isNull()) {
    for (unsigned int i=0; i<markedPoints.size(); ++i) {
      painter.setPen(markedPoints[i].pen);
      painter.drawRect(lrint((markedPoints[i].x+0.5)*imageScaled.width()/image.width())-rectSize/2,
                        lrint((markedPoints[i].y+0.5)*imageScaled.height()/image.height())-rectSize/2, rectSize, rectSize);
    }
    for (unsigned int i=0; i<markedLines.size(); ++i) 
    {
      const ImageLine& line = markedLines[i];
      painter.setPen(markedLines[i].pen);
      painter.drawLine(lrint((line.x1+0.5)*imageScaled.width()/image.width()),
                       lrint((line.y1+0.5)*imageScaled.height()/image.height()),
                       lrint((line.x2+0.5)*imageScaled.width()/image.width()),
                       lrint((line.y2+0.5)*imageScaled.height()/image.height()));
    }
  }
}

void ImageWidget::mouseReleaseEvent(QMouseEvent* event)
{
  float tmpClickedPixelX = (float)event->x() * (float)image.width()/(float)imageScaled.width() - 0.5f,
        tmpClickedPixelY = (float)event->y() * (float)image.height()/(float)imageScaled.height() - 0.5f;
  if (tmpClickedPixelX<=-0.5f || tmpClickedPixelX >=float(image.width())-0.5f ||
      tmpClickedPixelY<=-0.5f || tmpClickedPixelY >=float(image.height())-0.5f)
  {
    return;
  }
  clickedPixelX = tmpClickedPixelX;
  clickedPixelY = tmpClickedPixelY;
  mouseClickHappened = true;
  
  if (printSelectedPoint)
    cout << "ImageWidget: Clicked image point is " << clickedPixelX<<", "<<clickedPixelY<<".\n";
  emit mouseLeftClicked(clickedPixelX, clickedPixelY);
}

void ImageWidget::mouseMoveEvent(QMouseEvent* event)
{
  //cout << "Mouse moved to "<<event->x()<<", "<<event->y()<<".\n";
  if (showCurrentPixelPosition) {
    int imagePixelX = lrintf((float)event->x() * (float)image.width()/(float)imageScaled.width() - 0.5f),
        imagePixelY = lrintf((float)event->y() * (float)image.height()/(float)imageScaled.height() - 0.5f);
    if (imagePixelX>=0 && imagePixelX<image.width() &&
        imagePixelY>=0 && imagePixelY<image.height())
    {
      setToolTip(QString::number(imagePixelX)+","+QString::number(imagePixelY));
    }
  }
}


bool ImageWidget::save(const char* fileName) const {
  return image.save(fileName);
}

void ImageWidget::setShowCurrentPixelPosition(bool showCurrentPixelPosition) {
  setMouseTracking(showCurrentPixelPosition);
  this->showCurrentPixelPosition=showCurrentPixelPosition;
  if (!showCurrentPixelPosition)
    setToolTip("");
}

unsigned char* ImageWidget::getRGB(int& width, int& height) const
{
  width = image.width();
  height = image.height();
  if (imageData == NULL || width==0 || height==0)
    return NULL;
  
  unsigned int size = width*height;
  unsigned char* ret = new unsigned char[3*size];
  
  const unsigned char* srcPointer = imageData;
  unsigned char* dstPointer = ret;
  
  for (unsigned int i=0; i<size; i++) {
    *(dstPointer++) = *(srcPointer+2);
    *(dstPointer++) = *(srcPointer+1);
    *(dstPointer++) = *srcPointer;
    srcPointer += 4;
  }
  return ret;
}

} // namespace

