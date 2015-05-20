#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

#include <iostream>
using std::cout;
using std::pair;
#include <string>
using std::string;
#include <qfiledialog.h>
#include <qinputdialog.h>
#include <Eigen/Core>
#include "ui_baseMainWidget.h"

class ObjectDataBaseElement;

class MainWidget : public QWidget, public Ui::BaseMainWidget
{
  Q_OBJECT
  
  public:
    MainWidget(QWidget* parent = 0, Qt::WindowFlags fl = Qt::Window);
    ~MainWidget();

    Eigen::Vector3f selectedPoint;
    bool selectedPointChanged;
    //, loadPointCloud, savePointCloud;
    string fileName;

  public:
    //! Pops up a file dialog to ask for a file to open. fileFilter e.g. "All files (*.*)"
    bool loadUsingQFileDialog(std::string defaultFileName, std::string fileDescription, std::string& fileName, bool chooseDirectory=false);
    bool saveUsingQFileDialog(std::string defaultFileName, std::string fileDescription, std::string& fileName);

  public slots:
    //void resizeEvent(QResizeEvent* event);
    //void paintEvent(QPaintEvent* event);
    //void on_MenuPointCloudLoad_activated() { loadUsingQFileDialog("scene.pointCloud", "Point cloud (*.pointCloud);;All files (*.*)", fileName, loadPointCloud);}
    //void on_MenuPointCloudSave_activated() { saveUsingQFileDialog("scene.pointCloud", "Point cloud (*.pointCloud)", fileName, savePointCloud);}
    void newSelectedPoint();
    virtual void on_menu_Config_useGlobalFeaturesToSortScans_toggled(bool on);
    virtual void on_menu_Config_useBowToSortScans_toggled(bool on);
};

#endif
