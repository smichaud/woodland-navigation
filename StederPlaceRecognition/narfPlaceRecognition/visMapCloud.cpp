#include "visMapCloud.h"
#include "EXTERNALS/ais3dTools/visualization/aluGLViewerObjects/glo_primitives.h"
#include "EXTERNALS/ais3dTools/visualization/imageWidget/imageWidget.h"
#include "EXTERNALS/ais3dTools/visualization/aluGLViewer/alu_glviewer.h"
using namespace std;

namespace Vis {

MapCloud::MapCloud(SparsePointCloud* map, const Vis::MapCloud::PoseVector* trajectory) :
  BaseClass(), flattenMap(false), showCloud(true), showNumbers(true), map_(map), trajectory_(trajectory)
{
  // Set default values:
  setNodeSize();
  setFontSize();
  setWantedNoOfShownNumbers();
  setCoordinateSystem();
}

void MapCloud::draw() const {

  
  float nodeSize = nodeSize_;
  if (nodeSize < 0.0f) {
    if (trajectory_->size() <= 1)
      nodeSize = 1.0f;
    else {
      float averageDistanceBetweenNodes = 0.0f;
      for (int poseIdx=1; poseIdx<int(trajectory_->size()); ++poseIdx) {
        Eigen::Vector3f lastPose = trajectory_->at(poseIdx-1).translation(),
                        pose     = trajectory_->at(poseIdx).translation();
        averageDistanceBetweenNodes += (pose-lastPose).norm();
      }
      averageDistanceBetweenNodes /= float(trajectory_->size())-1.0f;
      nodeSize = 0.2f*averageDistanceBetweenNodes;
    }
  }
  
  //glDisable(GL_DEPTH_TEST);
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glDisable(GL_LIGHTING);
  
  float minHeight=std::numeric_limits<float>::max(),
        maxHeight=std::numeric_limits<float>::min();
  if (!flattenMap) {
    if (coordinateSystem_==xToRight_yDownwards_zForward) {
      for (std::map<int, std::map<int, std::set<int> > >::const_iterator itX=map_->points.begin(); itX!=map_->points.end(); ++itX) {
        if (itX->second.empty())
          continue;
        minHeight = std::min(minHeight, float(-itX->second.rbegin()->first)*map_->cellSize);
        maxHeight = std::max(maxHeight, float(-itX->second.begin()->first)*map_->cellSize);
      }
    }
    else {
      for (std::map<int, std::map<int, std::set<int> > >::const_iterator itX=map_->points.begin(); itX!=map_->points.end(); ++itX) {
        for (std::map<int, std::set<int> >::const_iterator itY=itX->second.begin(); itY!=itX->second.end(); ++itY) {
          if (itY->second.empty())
            continue;
          minHeight = std::min(minHeight, float(*itY->second.begin())*map_->cellSize);
          maxHeight = std::max(maxHeight, float(*itY->second.rbegin())*map_->cellSize);
        }
      }
    }
  }
  
  glColor3f(0.0f, 0.0f, 0.0f);
  if (showCloud) {
    glBegin(GL_POINTS);
      for (std::map<int, std::map<int, std::set<int> > >::const_iterator itX=map_->points.begin(); itX!=map_->points.end(); ++itX)
        for (std::map<int, std::set<int> >::const_iterator itY=itX->second.begin(); itY!=itX->second.end(); ++itY)
          for (std::set<int>::const_iterator itZ=itY->second.begin(); itZ!=itY->second.end(); ++itZ) {
            float x=itX->first*map_->cellSize, y=itY->first*map_->cellSize, z=*itZ*map_->cellSize;
            if (flattenMap) {
              if (coordinateSystem_==xToRight_yDownwards_zForward)
                y = 0.0f;
              else
                z = 0.0f;
            }
            else {
              float distanceUsedForColor = (coordinateSystem_==xToRight_yDownwards_zForward ? -y : z);
              distanceUsedForColor = (distanceUsedForColor-minHeight) / (maxHeight-minHeight);
              distanceUsedForColor = std::max(0.0f, std::min(1.0f, distanceUsedForColor));
              unsigned char r, g, b;
              Ais3dTools::ImageWidget::getColorForReal(distanceUsedForColor, r, g, b, false);
              glColor3ub(r, g, b);
            }
            glVertex3f(x, y, z);
          }
    glEnd();
  }
  
  Eigen::Isometry3f coordTrans = Eigen::Isometry3f::Identity();
  if (coordinateSystem_==xToRight_yDownwards_zForward) {
    coordTrans(0,0)= 0.0f; coordTrans(0,1)= 0.0f; coordTrans(0,2)=1.0f; coordTrans(0,3)=0.0f;
    coordTrans(1,0)=-1.0f; coordTrans(1,1)= 0.0f; coordTrans(1,2)=0.0f; coordTrans(1,3)=0.0f;
    coordTrans(2,0)= 0.0f; coordTrans(2,1)=-1.0f; coordTrans(2,2)=0.0f; coordTrans(2,3)=0.0f;
    coordTrans(3,0)= 0.0f; coordTrans(3,1)= 0.0f; coordTrans(3,2)=0.0f; coordTrans(3,3)=1.0f;
  }
  //std::cout <<PVARN(coordinateSystem_);
  
  for (int poseIdx=0; poseIdx<int(trajectory_->size()); ++poseIdx) {
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    Eigen::Isometry3f pose = trajectory_->at(poseIdx);
    if (flattenMap) {
      if (coordinateSystem_==xToRight_yDownwards_zForward)
        pose(1,3) = 0.0f;
      else
        pose(2,3) = 0.0f;
    }
    glMultMatrixf(pose.data());
    if (poseIdx<int(trajectory_->size())-1)
      glScalef(nodeSize, nodeSize, nodeSize);
    else
      glScalef(3*nodeSize, 3*nodeSize, 3*nodeSize);
    glPushMatrix();
    glMultMatrixf(coordTrans.inverse().data());
    Ais3dTools::drawPoseBox();
    glPopMatrix();
    glPopMatrix();
  }
  glColor3f(0.5f, 0.5f, 0.5f);
  glLineWidth(2.0f);
  glBegin(GL_LINE_STRIP);
    for (size_t poseIdx=0; poseIdx<trajectory_->size(); ++poseIdx) {
      float x=trajectory_->at(poseIdx)(0,3), y=trajectory_->at(poseIdx)(1,3), z=trajectory_->at(poseIdx)(2,3);
      if (flattenMap) {
        if (coordinateSystem_==xToRight_yDownwards_zForward)
          y = 0.0f;
        else
          z = 0.0f;
      }
      glVertex3f(x, y, z);
    }
  glEnd();
  glColor3f(0.0f, 0.0f, 1.0f);
  glLineWidth(5.0f);
  glBegin(GL_LINES);
    for (size_t connectingLinesIdx=0; connectingLinesIdx<connectingLines_.size(); ++connectingLinesIdx) {
      int index1 = connectingLines_[connectingLinesIdx].index1,
          index2 = connectingLines_[connectingLinesIdx].index2;
      if (index1>=int(trajectory_->size()) || index2>=int(trajectory_->size()))
        break;
      float x=trajectory_->at(index1)(0,3), y=trajectory_->at(index1)(1,3), z=trajectory_->at(index1)(2,3);
      if (flattenMap) {
        if (coordinateSystem_==xToRight_yDownwards_zForward)
          y = 0.0f;
        else
          z = 0.0f;
      }
      glVertex3f(x, y, z);
      x=trajectory_->at(index2)(0,3), y=trajectory_->at(index2)(1,3), z=trajectory_->at(index2)(2,3);
      if (flattenMap) {
        if (coordinateSystem_==xToRight_yDownwards_zForward)
          y = 0.0f;
        else
          z = 0.0f;
      }
      glVertex3f(x, y, z);
    }
  glEnd();
  
  glPopAttrib();
  
  //glEnable(GL_DEPTH_TEST);
  
  if (showNumbers) {
    int numberingStepSize = std::max(1, int(lrintf(float(trajectory_->size())/float(wantedNoOfShownNumbers_))));
    if (numberingStepSize > 5)
      numberingStepSize = lrintf(float(numberingStepSize)/5.0f)*5;
    
    QFont font;
    font.setPixelSize(fontSize_);
    font.setBold(true);
    for (int poseIdx=0; poseIdx<int(trajectory_->size()); ++poseIdx) {
      if ((numberingStepSize < 5 && (poseIdx)%numberingStepSize == 0) ||
          (numberingStepSize >= 5 && (poseIdx==0 || (poseIdx+1)%numberingStepSize==0)))
      {
        glColor3f(0.3, 0.3, 1.0);
        Eigen::Vector3f textPos = trajectory_->at(poseIdx).translation();
        if (flattenMap) {
          if (coordinateSystem_==xToRight_yDownwards_zForward)
            textPos[1] = 0.0f;
          else
            textPos[2] = 0.0f;
        }
        textPos.x() += nodeSize;
        //if (coordinateSystem_==xToRight_yDownwards_zForward)
          //textPos.y() -= nodeSize;
        //else
          //textPos.z() += nodeSize;
        QString text = QString::number(poseIdx+1);
        for (size_t viewerIdx=0; viewerIdx<viewer_.size(); ++viewerIdx) {
          Ais3dTools::ALUGLViewer& v = *viewer_[viewerIdx];
          int pixelX, pixelY;
          v.project3dTo2D(textPos.x(), textPos.y(), textPos.z(), pixelX, pixelY);
          v.drawText(pixelX, pixelY, text, font);
        }
      }
    }
  }
}


}  // Namespace end
