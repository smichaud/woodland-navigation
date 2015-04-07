#include "mainWidget.h"

MainWidget::MainWidget(QWidget* parent, Qt::WindowFlags fl) : QWidget(parent, fl)
{
  setupUi(this);
  //QObject::connect(this->viewer,  SIGNAL(newSelectedPoint()), this, SLOT(newSelectedPoint()));
  selectedPointChanged = false;
}

MainWidget::~MainWidget() {
}

//void MainWidget::resizeEvent(QResizeEvent *event) {
//}

//void MainWidget::paintEvent(QPaintEvent *event) {
//}

bool MainWidget::loadUsingQFileDialog(std::string defaultFileName, std::string fileDescription, std::string& fileName, bool chooseDirectory)
{
  QFileDialog fileDialog(this, QString("Open File"), QString("."), QString(fileDescription.c_str()));
  fileDialog.setAcceptMode(QFileDialog::AcceptOpen);  //AcceptSave
  if (!chooseDirectory)
  {
    fileDialog.setFileMode(QFileDialog::ExistingFile);
  }
  else
  {
    fileDialog.setFileMode(QFileDialog::Directory);
    fileDialog.setOption(QFileDialog::ShowDirsOnly, true);
  }
  fileDialog.selectFile(QString(defaultFileName.c_str()));
  if (fileDialog.exec())
  {
    fileName = fileDialog.selectedFiles()[0].toStdString();
    cout << "Selected file is \""<<fileName<<"\".\n";
    return fileName!="";
  }
  return false;
}

bool MainWidget::saveUsingQFileDialog(std::string defaultFileName, std::string fileDescription, std::string& fileName) {
  QFileDialog fileDialog(this, QString("Save File"), QString("."), QString(fileDescription.c_str()));
  fileDialog.setAcceptMode(QFileDialog::AcceptSave);
  fileDialog.selectFile(QString(defaultFileName.c_str()));
  if (fileDialog.exec())
  {
    fileName = fileDialog.selectedFiles()[0].toStdString();
    cout << "Selected file is \""<<fileName<<"\".\n";
    return fileName!="";
  }
  return false;
}

void MainWidget::newSelectedPoint() {
  selectedPointChanged = true;
  qglviewer::Vec qglviewerPoint;
  viewer->getSelectedPoint(qglviewerPoint);
  selectedPoint = Eigen::Vector3f(qglviewerPoint[0], qglviewerPoint[1], qglviewerPoint[2]);
}


void MainWidget::on_menu_Config_useGlobalFeaturesToSortScans_toggled(bool on) {
  //std::cout << __PRETTY_FUNCTION__ << ": Switching to "<<(on ? "on" : "off")<<".\n";
  if (on) {
    menu_Config_useBowToSortScans->setChecked(false);
  }
}

void MainWidget::on_menu_Config_useBowToSortScans_toggled(bool on) {
  //std::cout << __PRETTY_FUNCTION__ << ": Switching to "<<(on ? "on" : "off")<<".\n";
  if (on) {
    menu_Config_useGlobalFeaturesToSortScans->setChecked(false);
  }
}
