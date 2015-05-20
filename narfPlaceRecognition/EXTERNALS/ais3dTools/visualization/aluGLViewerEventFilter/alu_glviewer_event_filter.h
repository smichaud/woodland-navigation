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
#ifndef ALU_GLVIEWER_EVENT_FILTER_H
#define ALU_GLVIEWER_EVENT_FILTER_H

#include <QObject>

namespace Ais3dTools {
  class ALUGLViewer;

class ALUGLViewerEventFilter : public QObject
{
  Q_OBJECT
  public:
     ALUGLViewerEventFilter();
    ~ALUGLViewerEventFilter();

    void watchViewer(Ais3dTools::ALUGLViewer* v);

    bool getClickPose(double& px, double& py, double& pz) { if(validPoint){ px = x; py = y; pz = z;} return validPoint;}

    Ais3dTools::ALUGLViewer* widget_;

  public: // the public variables
    bool upEvent;
    bool downEvent;
    bool leftEvent;
    bool rightEvent;
    bool pgUpEvent;
    bool pgDownEvent;
    bool homeEvent;
    bool endEvent;
    bool spaceEvent;

    bool plusEvent;
    bool minusEvent;

    /** 1 to 0 keys */
    bool zeroEvent;
    bool oneEvent;
    bool twoEvent;
    bool threeEvent;
    bool fourEvent;
    bool fiveEvent;
    bool sixEvent;
    bool sevenEvent;
    bool eightEvent;
    bool nineEvent;

    /** F1 to F12 keys */
    bool fOneEvent;
    bool fTwoEvent;
    bool fThreeEvent;
    bool fFourEvent;
    bool fFiveEvent;
    bool fSixEvent;
    bool fSevenEvent;
    bool fEightEvent;
    bool fNineEvent;
    bool fTenEvent;
    bool fElevenEvent;
    bool fTwelveEvent;

    /** a to z */
    bool aEvent;
    bool bEvent;
    bool cEvent;
    bool dEvent;
    bool eEvent;
    bool fEvent;
    bool gEvent;
    bool hEvent;
    bool iEvent;
    bool jEvent;
    bool kEvent;
    bool lEvent;
    bool mEvent;
    bool nEvent;
    bool oEvent;
    bool pEvent;
    bool qEvent;
    bool rEvent;
    bool sEvent;
    bool tEvent;
    bool uEvent;
    bool vEvent;
    bool wEvent;
    bool xEvent;
    bool yEvent;
    bool zEvent;

    bool mouseLeftClickEvent;
    bool mouseRightClickEvent;

    bool mouseLeftAndShiftEvent;
    bool mouseRightAndShiftEvent;
    
    signals:
      void spacePressed();

      void plusPressed();
      void minusPressed();

      /** 1 to 0 keys */
      void zeroPressed();
      void onePressed();
      void twoPressed();
      void threePressed();
      void fourPressed();
      void fivePressed();
      void sixPressed();
      void sevenPressed();
      void eightPressed();
      void ninePressed();

      /** F1 to F12 keys */
      void fOnePressed();
      void fTwoPressed();
      void fThreePressed();
      void fFourPressed();
      void fFivePressed();
      void fSixPressed();
      void fSevenPressed();
      void fEightPressed();
      void fNinePressed();
      void fTenPressed();
      void fElevenPressed();
      void fTwelvePressed();

      /** a to z */
      void aPressed();
      void bPressed();
      void cPressed();
      void dPressed();
      void ePressed();
      void fPressed();
      void gPressed();
      void hPressed();
      void iPressed();
      void jPressed();
      void kPressed();
      void lPressed();
      void mPressed();
      void nPressed();
      void oPressed();
      void pPressed();
      void qPressed();
      void rPressed();
      void sPressed();
      void tPressed();
      void uPressed();
      void vPressed();
      void wPressed();
      void xPressed();
      void yPressed();
      void zPressed();

      /** Mouse click handling */
      void mouseLeftClicked();
      void mouseRightClicked();

      void mouseLeftClickedAndShiftPressed();
      void mouseRightClickedAndShiftPressed();
      
  protected:

    bool eventFilter(QObject* o, QEvent* e);
    void togglePoseInit();

    bool validPoint;
    double x,y,z;
};

}

#endif
