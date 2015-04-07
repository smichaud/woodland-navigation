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
#include "alu_glviewer_event_filter.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include "visualization/aluGLViewer/alu_glviewer.h"
using namespace std;

namespace Ais3dTools {

ALUGLViewerEventFilter::ALUGLViewerEventFilter() : QObject(),
  widget_(0), upEvent(0), downEvent(0), leftEvent(0), rightEvent(0),
  pgUpEvent(0), pgDownEvent(0), homeEvent(0), endEvent(0), spaceEvent(0),
  plusEvent(0), minusEvent(0),
  zeroEvent(false), oneEvent(false), twoEvent(false), threeEvent(false), fourEvent(false), 
  fiveEvent(false), sixEvent(false),sevenEvent(false), eightEvent(false), nineEvent(false), 
  fOneEvent(false), fTwoEvent(false), fThreeEvent(false), fFourEvent(false), fFiveEvent(false),
  fSixEvent(false), fSevenEvent(false), fEightEvent(false), fNineEvent(false), fTenEvent(false), fElevenEvent(false), fTwelveEvent(false),
  aEvent(0), bEvent(0), cEvent(0), dEvent(0), eEvent(0), fEvent(0), gEvent(0), hEvent(0),
  iEvent(0), jEvent(0), kEvent(0), lEvent(0), mEvent(0), nEvent(0), oEvent(0), pEvent(0),
  qEvent(0), rEvent(0), sEvent(0), tEvent(0), uEvent(0), vEvent(0), wEvent(0), xEvent(0),
  yEvent(0), zEvent(0), mouseLeftClickEvent(0), mouseRightClickEvent(0), mouseLeftAndShiftEvent(0), mouseRightAndShiftEvent(0),
  validPoint(0), x(0.0f), y(0.0f), z(0.0f)
{

}

ALUGLViewerEventFilter::~ALUGLViewerEventFilter()
{
}

bool ALUGLViewerEventFilter::eventFilter(QObject* o, QEvent* e)
{
  (void) o;
  if (e->type() == QEvent::KeyPress) {
    QKeyEvent* k = static_cast<QKeyEvent*>(e);

    /** 1 to 0 keys */
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_0) {
      zeroEvent = true;
      emit zeroPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_1) {
      oneEvent = true;
      emit onePressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_2) {
      twoEvent = true;
      emit twoPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_3) {
      threeEvent = true;
      emit threePressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_4) {
      fourEvent = true;
      emit fourPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_5) {
      fiveEvent = true;
      emit fivePressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_6) {
      sixEvent = true;
      emit sixPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_7) {
      sevenEvent = true;
      emit sevenPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_8) {
      eightEvent = true;
      emit eightPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_9) {
      nineEvent = true;
      emit ninePressed();
      return true;
    }

    /** F1 to F12 keys */
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F1) {
      fOneEvent = true;
      emit fOnePressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F2) {
      fTwoEvent = true;
      emit fTwoPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F3) {
      fThreeEvent = true;
      emit fThreePressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F4) {
      fFourEvent = true;
      emit fFourPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F5) {
      fFiveEvent = true;
      emit fFivePressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F6) {
      fSixEvent = true;
      emit fSixPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F7) {
      fSevenEvent = true;
      emit fSevenPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F8) {
      fEightEvent = true;
      emit fEightPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F9) {
      fNineEvent = true;
      emit fNinePressed();
      return true;
    }

    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F10) {
      fTenEvent = true;
      emit fTenPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F11) {
      fElevenEvent = true;
      emit fElevenPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F12) {
      fTwelveEvent = true;
      emit fTwelvePressed();
      return true;
    }

    /** a to z */
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_A) {
      aEvent = true;
      emit aPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_B) {
      bEvent = true;
      emit bPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_C) {
      cEvent = true;
      emit cPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_D) {
      dEvent = true;
      emit dPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_E) {
      eEvent = true;
      emit ePressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_F) {
      fEvent = true;
      emit fPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_G) {
      gEvent = true;
      emit gPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_H) {
      hEvent = true;
      emit hPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_I) {
      iEvent = true;
      emit iPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_J) {
      jEvent = true;
      emit jPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_K) {
      kEvent = true;
      emit kPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_L) {
      lEvent = true;
      emit lPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_M) {
      mEvent = true;
      emit mPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_N) {
      nEvent = true;
      emit nPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_O) {
      oEvent = true;
      emit oPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_P) {
      pEvent = true;
      emit pPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_Q) {
      qEvent = true;
      emit qPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_R) {
      rEvent = true;
      emit rPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_S) {
      sEvent = true;
      emit sPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_T) {
      tEvent = true;
      emit tPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_U) {
      uEvent = true;
      emit uPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_V) {
      vEvent = true;
      emit vPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_W) {
      wEvent = true;
      emit wPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_X) {
      xEvent = true;
      emit xPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_Y) {
      yEvent = true;
      emit yPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_Z) {
      zEvent = true;
      emit zPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_Up) 
    {
      upEvent = true;
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_Down) 
    {
      downEvent = true;
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_Left) 
    {
      leftEvent = true;
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_Right) 
    {
      rightEvent = true;
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_PageUp) 
    {
      pgUpEvent = true;
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_PageDown) 
    {
      pgDownEvent = true;
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_Home) 
    {
      homeEvent = true;
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_End) 
    {
      endEvent = true;
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_Space) 
    {
      spaceEvent = true;
      emit spacePressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_Plus)
    {
      plusEvent = true;
      emit plusPressed();
      return true;
    }
    if ((k->modifiers() & Qt::NoModifier) == 0 && k->key() == Qt::Key_Minus)
    {
      minusEvent = true;
      emit minusPressed();
      return true;
    }    

  }

  if(e->type() == QEvent::MouseButtonPress){
    QMouseEvent* k = static_cast<QMouseEvent*>(e);
    QPoint clickPose = k->pos();
    validPoint = widget_->getPoseFromGl(clickPose.x(), clickPose.y(), x, y, z);
    if(k->button() & Qt::LeftButton){
      mouseLeftClickEvent = true;
      emit mouseLeftClicked();
      if((k->modifiers() & Qt::SHIFT)){
        mouseLeftAndShiftEvent = true;
        emit mouseLeftClickedAndShiftPressed();
      }
    }
    if(k->button() & Qt::RightButton){
      mouseRightClickEvent = true;
      emit mouseRightClicked();      
      if((k->modifiers() & Qt::SHIFT)){
        mouseRightAndShiftEvent = true;
        emit mouseRightClickedAndShiftPressed();
      }
    }
  }
  
  // we do not handle anything specific, the widget may still receive the events
  return false;
}

void ALUGLViewerEventFilter::watchViewer(Ais3dTools::ALUGLViewer* v)
{
  widget_ = v;
  v->installEventFilter(this);
}

}
