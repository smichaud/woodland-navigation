template <typename real>
inline void Ais3dTools::ImageWidget::getColorForReal(real value, unsigned char& r, unsigned char& g, unsigned char& b, bool grayScale) {
  if (grayScale) {
    r = g = b = lrint(value*255);
    return;
  }
  
  r = g = b = 0;
  value *= 10;
  if (value <= 1.0) 
  {  // black -> purple
    b = lrint(value*200.0);
    r = lrint(value*120.0);
  }
  else if (value <= 2.0) 
  {  // purple -> blue
    b = 200 + lrint((value-1.0)*55.0);
    r = 120 - lrint((value-1.0)*120.0);
  }
  else if (value <= 3.0) 
  {  // blue -> turquoise
    b = 255 - lrint((value-2.0)*55.0);
    g = lrint((value-2.0)*200.0);
  }
  else if (value <= 4.0) 
  {  // turquoise -> green
    b = 200 - lrint((value-3.0)*200.0);
    g = 200 + lrint((value-3.0)*55.0);
  }
  else if (value <= 5.0) 
  {  // green -> yellow
    g = 255 - lrint((value-4.0)*55.0);
    r = lrint((value-4.0)*200.0);
  }
  else if (value <= 6.0) 
  { // yellow -> orange
    r = 200;
    g = 200 - lrint((value-5.0)*100);
  }
  else if (value <= 7.0) 
  {  // orange -> red
    r = 200 + lrint((value-6.0)*55);
    g = 100  - lrint((value-6.0)*100);
  }
  else 
  {  // red -> white
    r = 255;
    g = lrint((value-7.0)*255.0/3.0);
    b = lrint((value-7.0)*255.0/3.0);
  }
  
  //cout << PVARC(value)<<PVARC((int)r)<<PVARC((int)g)<<PVARN((int)b);
}


template <typename real>
void Ais3dTools::ImageWidget::setRealImage(const real* data, unsigned int width, unsigned int height, bool showImageNow,
                               real minValue, real maxValue, QImage* legend)
{
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
  
  const real* srcPointer = data;
  unsigned char* dstPointer = imageData;
  
  real minNonInfValue=std::numeric_limits<real>::max(), maxNonInfValue=std::numeric_limits<real>::min();
  if (maxValue>minValue && std::isfinite(minValue) && std::isfinite(maxValue)) {
    minNonInfValue = minValue;
    maxNonInfValue = maxValue;
  }
  else {
    for (unsigned int i=0; i<size; i++) {
      if (isfinite(*srcPointer)) {
        minNonInfValue = std::min(minNonInfValue, *srcPointer);
        maxNonInfValue = std::max(maxNonInfValue, *srcPointer);
      }
      ++srcPointer;
    }
  }
  //cout << __PRETTY_FUNCTION__<<": "<<PVAR(minNonInfValue)<<", "<<PVAR(maxNonInfValue)<<"\n";
  srcPointer = data;
  float factor = real(1.0) / real(maxNonInfValue-minNonInfValue);
  
  unsigned char r, g, b;
  for (unsigned int i=0; i<size; i++) {
    if (isinf(*srcPointer)) {
      if (*srcPointer > 0.0) {
        *(dstPointer++) = 200;
        *(dstPointer++) = 150;
        *(dstPointer++) = 150;
        *(dstPointer++) = 255;
      }
      else {
        *(dstPointer++) = 150;
        *(dstPointer++) = 200;
        *(dstPointer++) = 150;
        *(dstPointer++) = 255;
      }
    }
    else if (isnan(*srcPointer)) {
      *(dstPointer++) = 100;
      *(dstPointer++) = 100;
      *(dstPointer++) = 200;
      *(dstPointer++) = 255;
    }
    else {
      real value = std::max(real(0.0), std::min(real(1.0), real((*srcPointer - minNonInfValue)*factor)));
      getColorForReal(value, r, g, b, showRealImagesInGrayScales);
      *(dstPointer++) = b;
      *(dstPointer++) = g;
      *(dstPointer++) = r;
      *(dstPointer++) = 255;
    }
    ++srcPointer;
  }
  
  image = QImage(imageData, width, height, QImage::Format_RGB32);
  
  // Draw legend
  if (legend != NULL) {
    int colorBarWidth=30, legendWidth=70, legendHeight=500;
    *legend = QImage(legendWidth, legendHeight, QImage::Format_RGB32);
    legend->fill(0xFFFFFFFF);

    //std::cout << PVARN(real(0.1)*(maxNonInfValue-minNonInfValue));
    //std::cout << PVARN(log10(real(0.1)*(maxNonInfValue-minNonInfValue)));
    float significantValueStep = std::pow(float(10), round(float(log10(float(0.1)*float(maxNonInfValue-minNonInfValue)))));
    //std::cout << PVARN(significantValueStep);
    for (int y=0; y<legendHeight; ++y) {
      real value = real(legendHeight-1-y)/real(legendHeight-1);
      getColorForReal(value, r, g, b, showRealImagesInGrayScales);
      int color = qRgb(r,g,b);
      for (int x=0; x<colorBarWidth; ++x) {
        legend->setPixel(x, y, color);
      }
    }
    
    QPainter painter(legend);
    for (real textPos=significantValueStep*std::ceil(float(minNonInfValue)/float(significantValueStep)); textPos<maxNonInfValue; textPos+=significantValueStep) {
      int y = legendHeight-1-round(float(((textPos - minNonInfValue)*factor) * real(legendHeight-1))) - painter.font().pixelSize()/2;
      painter.drawText(QPoint(colorBarWidth+2, y), QString::number(textPos));
    }
  }
  
  if (showImageNow) showImage();
}
  
template <typename real>
inline void Ais3dTools::ImageWidget::getColorForAngle(real value, unsigned char& r, unsigned char& g, unsigned char& b)
{
  r = g = b = 0;
  if (value < -M_PI/2.0f) 
  {  // black -> blue
    b = (lrint(255*(value+float(M_PI))/(float(M_PI)/2.0f)));
  }
  else if (value <= 0.0f) 
  {  // blue -> turquoise
    b = 255;
    g = (lrint(255*(value+float(M_PI/2))/(float(M_PI)/2.0f)));
  }
  else if (value <= M_PI/2.0f) 
  {  // turquoise -> green
    g = 255;
    b = (255-lrint(255*(value)/(float(M_PI)/2.0f)));
  }
  //else if (value <= 0.0f) 
  //{  // blue -> white
    //b = 255;
    //r = g = (lrint(255*(value+float(M_PI/2))/(float(M_PI)/2.0f)));
  //}
  //else if (value <= M_PI/2.0f) 
  //{  // white -> green
    //g = 255;
    //r = b = (255-lrint(255*(value)/(float(M_PI)/2.0f)));
  //}
  else 
  {  // green -> black
    g = (255-lrint(255*(value-M_PI/2.0f)/(float(M_PI)/2.0f)));
  }
  //std::cout << 180.0f*value/M_PI<<"deg => "<<(int)r<<", "<<(int)g<<", "<<(int)b<<"\n";
}


template <typename real>
void Ais3dTools::ImageWidget::setAngleImage(const real* data, unsigned int width, unsigned int height, bool showImageNow)
{
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
  
  const real* srcPointer = data;
  unsigned char* dstPointer = imageData;
  
  srcPointer = data;
  
  for (unsigned int i=0; i<size; i++) {
    if (!isfinite(*srcPointer)) {
      *(dstPointer++) = 255;
      *(dstPointer++) = 255;
      *(dstPointer++) = 255;
      //*(dstPointer++) = 150;
      //*(dstPointer++) = 200;
      //*(dstPointer++) = 150;
      *(dstPointer++) = 255;
    }
    else {
      unsigned char r, g, b;
      getColorForAngle(*srcPointer, r, g, b);
      *(dstPointer++) = b;
      *(dstPointer++) = g;
      *(dstPointer++) = r;
      *(dstPointer++) = 255;
    }
    ++srcPointer;
  }
  
  image = QImage(imageData, width, height, QImage::Format_RGB32);
  
  if (showImageNow) showImage();
}
