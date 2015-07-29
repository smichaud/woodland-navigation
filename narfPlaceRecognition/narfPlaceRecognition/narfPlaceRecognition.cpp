#include <iostream>
using namespace std;

#pragma GCC diagnostic ignored "-Wunused-parameter"  // Do not show warnings from ROS
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/poses_from_matches.h>
//#include <pcl/features/range_image_border_extractor.h>
//#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf.h>
//#include <pcl/object_recognition/false_positives_filter.h>
#undef MEASURE_FUNCTION_TIME
#pragma GCC diagnostic warning "-Wunused-parameter"

#include <boost/filesystem.hpp>
#include <omp.h>

#include <QApplication>
#include "mainWidget.h"
#include "narfPlaceRecognitionLib/scanDatabase.h"
#include "narfPlaceRecognitionLib/rangeImageMatching.h"
#include "visMapCloud.h"
#include "Results.hpp"

#include "EXTERNALS/ais3dTools/visualization/imageWidget/imageWidget.h"
#include "EXTERNALS/ais3dTools/visualization/aluGLViewer/alu_glviewer.h"
#include "EXTERNALS/ais3dTools/visualization/aluGLViewerObjects/glo_pcl_pointcloud.h"

#include "EXTERNALS/ais3dTools/basics/misc.h"
#include "EXTERNALS/ais3dTools/basics/timeutil.h"
#include "EXTERNALS/ais3dTools/basics/filesys_tools.h"
#include "EXTERNALS/ais3dTools/basics/parameters_manager.h"
using namespace Ais3dTools;

#include "EXTERNALS/spm/spmBase/spModel.h"
#include "EXTERNALS/spm/spmBase/narfWord.h"
#include "EXTERNALS/spm/spmBase/narfInstance.h"
#include "EXTERNALS/spm/spmBase/narfImport.h"
#include "EXTERNALS/spm/spmDictionarySelector/spmDictionarySelection.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/types/slam3d/types_slam3d.h"
G2O_USE_OPTIMIZATION_LIBRARY(csparse);


MainWidget window;
RangeImageMatching rangeImageMatching;
ScanDatabase& scanDatabase = rangeImageMatching.scanDatabase;

std::vector<float> confusionMatrixScores, confusionMatrixFastScores;
std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> >
    confusionMatrixTransformations, confusionMatrixFastTransformations;

Results results(scanDatabase, window, confusionMatrixScores); // Let's get dirty/dangerous

std::string statisticsDirectory = "!statistics";

bool& useRotationInvariance = ScanDatabaseElement::useRotationInvariance;

unsigned int noOfShownTransformations = 1000000;

float defaultAngularResolution = deg2rad(0.5),
      defaultNoiseLevel = 0.0,
      defaultSupportSize = 1.0,
      defaultSupportSizeDictionary = 0.5*defaultSupportSize,
      defaultMaxDistanceToConsiderScansOverlapping = 3*defaultSupportSize,
      defaultMaxValidationPointError = 0.2*defaultSupportSize;

float angularResolution = -1,
      noiseLevel = -1,
      supportSize = -1,
      supportSizeDictionary = -1,
      maxDistanceToConsiderScansOverlapping = -1;
pcl::RangeImage::CoordinateFrame coordinateFrame(pcl::RangeImage::CAMERA_FRAME),
    defaultCoordinateFrame(pcl::RangeImage::CAMERA_FRAME);

float averageDistanceBetweenConsecutiveScans = -1.0f,
      averageRangeMeasurement = -1.0f,
      maximumRangeInDataset = -1.0f;

int descriptorSize = 36,
    noOfValidationPoints = 200;
//float minimumDistanceBetweenKeypoints = 0.5f;  // Factor regarding support size

double timeLimitForDatabaseQuery = -1;

//float maxDescrDistForClustering=0.08;

typedef ScanDatabaseElement::PointCloudType PointCloudType;

QString windowTitle = "Place Recognition GUI";


std::string directoryName;

int dictionarySize = 100;


////////////////////////////////////////////////////////////////////////////////
template <typename T> std::string toString(const T& t);
void printUsage (const char *progName);
int getMatrixIndex(int scan1Index, int scan2Index);
std::pair<int,int> getScanIndices(int matrixIndex);

void recalculateAndSaveRangeImages(QApplication &application, const int &maxNoOfThreads, const float &maximumRange);
void createDatasetStatistics(MyPcl::NarfssKeypoint &keyPointDetector);
void recalculateAndSaveAll(QApplication &application, MyPcl::NarfssKeypoint &keyPointDetector, const int &maxNoOfThreads, const float &maximumRange);
void calculateConfusionMatrix(QApplication &application, ImageWidget &confusionMatrixWidget, vector<vector<double> > &neededTimesPerScan, const int &maxNoOfThreads, const float &maximumRange, const bool &useSlamHack);
template <typename ValueType> void saveResult(const std::string& valueName, ValueType value, bool datasetSpecific=true);
////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv) {
    setlocale (LC_NUMERIC,"C");  // HACK to reset numeric locale, we need C, else parsing ASCII numbers fails

    for (char c; (c = getopt (argc, argv, "h")) != -1;) {
        switch (c) {
            case 'h': {
                          printUsage (argv[0]);
                          exit (0);
                      }
        }
    }
    if (optind < argc) {
        directoryName = argv[optind];
    }

    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > trajectory;
    SparsePointCloud mapCloud;
    Vis::MapCloud visMapCloud(&mapCloud, &trajectory);
    visMapCloud.setDrawColor(0.8, 0.8, 0.0);

    int maxNoOfThreads = 1;
    rangeImageMatching.parameters.minValidationPointScore = 0.2f;
    rangeImageMatching.parameters.maxDescriptorDistance = 0.05f;
    rangeImageMatching.parameters.icpNumIterations = 10;
    rangeImageMatching.parameters.icpPixelSearchRadius = 3;
    //float minSurfaceChangeForValidationPoints = 0.3f;
    float mapCellSize = 0.5f;
    visMapCloud.flattenMap = false;
    visMapCloud.showCloud = true;
    visMapCloud.showNumbers = true;

    rangeImageMatching.parameters.maxValidationPointError = 1.0f;
    rangeImageMatching.parameters.minDistanceBetweenMatches = 0.5f*supportSize;
    //rangeImageMatching.getParams(argc, argv);

    //pcl::RangeImageBorderExtractor borderExtractor;
    ////borderExtractor.getParameters().pixel_radius_principal_curvature = 4;
    //pcl::NarfKeypoint keyPointDetector(&borderExtractor);
    //keyPointDetector.getParameters().max_no_of_threads = 1;
    //keyPointDetector.getParameters().support_size = supportSize;
    //keyPointDetector.getParameters().min_distance_between_interest_points = minimumDistanceBetweenKeypoints;
    //keyPointDetector.getParameters().min_interest_value = 0.25f;
    //keyPointDetector.getParameters().min_surface_change_score = 0.25f;
    //keyPointDetector.getParameters().max_no_of_interest_points = 200;

    MyPcl::NarfssKeypoint keyPointDetector;
    keyPointDetector.getParameters().min_interest_value = 0.0125f;

    //pcl::RangeImageBorderExtractor borderExtractorForDictionary;
    //pcl::NarfKeypoint keyPointDetectorForDictionary(&borderExtractorForDictionary);
    //keyPointDetectorForDictionary.getParameters().max_no_of_threads = 1;
    //keyPointDetectorForDictionary.getParameters().support_size = supportSizeDictionary;
    //keyPointDetectorForDictionary.getParameters().min_distance_between_interest_points = minimumDistanceBetweenKeypoints;
    //keyPointDetectorForDictionary.getParameters().min_interest_value = 0.1f;
    //keyPointDetectorForDictionary.getParameters().min_surface_change_score = 0.1f;
    //keyPointDetectorForDictionary.getParameters().add_points_on_straight_edges = true;
    //keyPointDetectorForDictionary.getParameters().do_non_maximum_suppression = false;
    //keyPointDetectorForDictionary.getParameters().max_no_of_interest_points = 1000;

    MyPcl::NarfssKeypoint keyPointDetectorForDictionary;
    keyPointDetectorForDictionary.getParameters().min_interest_value = 0.05f;
    keyPointDetectorForDictionary.getParameters().use_max_descr_dist_instead_of_min = true;
    keyPointDetectorForDictionary.getParameters().do_non_maximum_suppression = false;
    //keyPointDetectorForDictionary.getParameters().min_dist_between_keypoints_factor = 0.5;

    float minScoreToAddToGraph = 0.2f;

    float maximumRange = -1;

    rangeImageMatching.parameters.maxNoOfPoseEstimatesPerScanPair = 2000;
    rangeImageMatching.parameters.maxNoOfPoseEstimatesPerScanPairRotInv = 5000;
    timeLimitForDatabaseQuery = -1;

    std::string parametersFilename = "currentPlaceRecognitionParameters.ini";
    Ais3dTools::ParametersManager parametersManager(parametersFilename);
    parametersManager.updateParameters();  // Read file for the first time

    // Default parameters
    parametersManager.addParameterValue("narfPlaceRecognition", "maxNoOfThreads",                           1, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "useRotationInvariance",                false, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "maximumRange",                            -1, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "minValidationPointScoreBeforeICP",       0.2, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "minValidationPointScore",                0.0, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "maxDescriptorDistance",                 0.05, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "icpNumIterations",                        20, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "icpPixelSearchRadius",                     2, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "mapCellSize",                            0.5, true);
    //parametersManager.addParameterValue("narfPlaceRecognition", "maxDescrDistForClustering",             0.11, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "max_no_of_keypoints",                    200, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "max_no_of_dictionary_keypoints",        2000, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "doSelfSimilarityScoreAdjustment",       true, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "minScoreToAddToGraph",                   0.2, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "maxNoOfPoseEstimatesPerScanPair",       2000, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "maxNoOfPoseEstimatesPerScanPairRotInv", 5000, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "timeLimitForDatabaseQuery",               -1, true);
    parametersManager.addParameterValue("narfPlaceRecognition", "dictionarySize",                          40, true);
    parametersManager.writeToFile(parametersFilename);

    QApplication application(argc,argv);
    setlocale (LC_NUMERIC,"C");  // HACK to reset numeric locale, we need C, else parsing ASCII numbers fails
    window.imagesFrame->hide();
    QListWidget& visTransformationsList = *window.transformationsList;
    visTransformationsList.setSelectionMode(QAbstractItemView::SingleSelection);
    visTransformationsList.hide();
    window.confusionMatrixManipulationFrame->hide();
    window.statusFrame->hide();
    window.currentScanLabel->hide();
    QListWidget& visScanDatabaseList = *window.scanDatabaseList;
    visScanDatabaseList.setSelectionMode(QAbstractItemView::SingleSelection);
    visScanDatabaseList.hide();
    window.show();

    //window.imageWidget1->showRealImagesInGrayScales = true;

    bool viewPointCloud=0, viewPointCloudFromRangeImage=0, viewFeatures=0, viewValidationPoints=0;
    bool scan1Changed=false, scan2Changed=false, databaseChanged=false, poseEstimatesChanged=false,
         setConfusionMatrixPoseEstimate=false;
    int poseEstimatesType = 0; // 1:leftRight, 2:scanAgainstDatabase, 3:selectedInConfusionMatrix

    // Instantiate the viewer.
    Ais3dTools::ALUGLViewer& viewer = *window.viewer,
        & viewer2 = *window.viewer2;
    viewer.setUpVector(0.0, -1.0, 0.0);    viewer.setPosition(0.0, -3.0, -10.0);  viewer.lookAt(0.0, 0.0, 3.0);
    viewer2.setUpVector(0.0, -1.0, 0.0);   viewer2.setPosition(0.0, -3.0, -10.0); viewer2.lookAt(0.0, 0.0, 3.0);
    viewer.setBackgroundColor(Qt::white);  viewer2.setBackgroundColor(Qt::white);

    ScanDatabaseElement scan1, scan2;

    Ais3dTools::PclPointCloudObjectT<PointCloudType> visPointCloud(&scan1.pointCloud), visPointCloud2(&scan2.pointCloud);
    visPointCloud.setDrawColor(0.7, 0.7, 0.2);  visPointCloud2.setDrawColor(0.7, 0.7, 0.2);

    Ais3dTools::PclPointCloudObjectT<pcl::RangeImage> visRangeImageCloud(&scan1.rangeImage), visRangeImageCloud2(&scan2.rangeImage);
    visRangeImageCloud.setDrawColor(0.2, 0.7, 0.2);  visRangeImageCloud2.setDrawColor(0.2, 0.7, 0.2);

    pcl::PointCloud<pcl::PointXYZ> featurePositions1, featurePositions2;
    Ais3dTools::PclPointCloudObjectT<pcl::PointCloud<pcl::PointXYZ> > visFeaturePositions1(&featurePositions1),
        visFeaturePositions2(&featurePositions2);
    visFeaturePositions1.setDrawColor(0.7, 0.2, 0.2); visFeaturePositions2.setDrawColor(0.7, 0.2, 0.2);
    visFeaturePositions1.setPointSize(5.0);       visFeaturePositions2.setPointSize(5.0);

    pcl::PointCloud<pcl::PointXYZ> validationPoints1, validationPoints2;
    Ais3dTools::PclPointCloudObjectT<pcl::PointCloud<pcl::PointXYZ> > visValidationPoints1(&validationPoints1),
        visValidationPoints2(&validationPoints2);
    visValidationPoints1.setDrawColor(0.2, 0.2, 0.7); visValidationPoints2.setDrawColor(0.2, 0.2, 0.7);
    visValidationPoints1.setPointSize(7.0);       visValidationPoints2.setPointSize(7.0);

    PointCloudType transformedPointCloud, transformedPointCloudSlam;
    Ais3dTools::PclPointCloudObjectT<PointCloudType> visTransformedRangeImageCloud(&transformedPointCloud),
        visTransformedRangeImageCloudSlam(&transformedPointCloudSlam);
    visTransformedRangeImageCloud.setDrawColor(0.0, 0.0, 0.8);
    visTransformedRangeImageCloudSlam.setDrawColor(0.5, 0.5, 0.5);
    viewer.add(&visTransformedRangeImageCloud);
    viewer.add(&visTransformedRangeImageCloudSlam);


    /////////////// SCREENSHOT FUN//////////////////
    visTransformedRangeImageCloud.setDrawColor(0.6, 0.6, 1.0);  visTransformedRangeImageCloud.setPointSize(2.0);
    //visPointCloud.setDrawColor(1.0, 0.1, 0.1);
    visPointCloud.setPointSize(2.0);
    visMapCloud.setNodeSize(-1.0f);  // Set to automatic
    //visMapCloud.setDrawColor(0.0, 0.0, 0.0);
    visMapCloud.setDrawColor(1.0, 1.0, 0.0);
    ///////////////////////////////////////////////

    Ais3dTools::ALUGLViewer mapViewer;
    mapViewer.setUpVector(-1.0, 0.0, 0.0);  mapViewer.setPosition(0.0, -50.0, 0.0);  mapViewer.lookAt(0.0, 0.0, 0.0);
    mapViewer.setDrawCoordinateSystem(true);

    pcl::RangeImage markedScan;
    Ais3dTools::PclPointCloudObjectT<pcl::RangeImage> visMarkedScan(&markedScan);
    visMarkedScan.setDrawColor(0.4, 1.0, 0.4);
    visMarkedScan.setPointSize(2.0f);
    mapViewer.add(&visMarkedScan);

    mapViewer.add(&visMapCloud);
    mapViewer.updateGL();
    mapViewer.setBackgroundColor(Qt::white);
    mapViewer.hide();

    pcl::PointCloud<pcl::PointXYZ> usedFeatures1, usedFeatures2;
    Ais3dTools::PclPointCloudObjectT<pcl::PointCloud<pcl::PointXYZ> > visUsedFeatures1(&usedFeatures1),
        visUsedFeatures2(&usedFeatures2);
    visUsedFeatures1.setDrawColor(1.0, 1.0, 1.0); visUsedFeatures2.setDrawColor(1.0, 1.0, 1.0);
    visUsedFeatures1.setPointSize(10.0);      visUsedFeatures2.setPointSize(10.0);
    viewer.add(&visUsedFeatures1);      viewer2.add(&visUsedFeatures2);

    bool needDraw=true;
    bool clearTransformations=false, ignoreClearTransformationsCall=false;

    pcl::PosesFromMatches::PoseEstimatesVector poseEstimates;
    pcl::PointCorrespondences6DVector featureMatches;

    int markedTransformationIdx=-1, markedScanIdx=-1;

    enum ConfusionMatrixFastSource {CMFS_EMPTY, CMFS_BAG_OF_WORDS, CMFS_GLOBAL_FEATURES};
    ConfusionMatrixFastSource confusionMatrixFastSource = CMFS_EMPTY;
    (void)confusionMatrixFastSource;
    ImageWidget confusionMatrixWidget;
    confusionMatrixWidget.setWindowTitle("Place recognition confusion matrix");
    confusionMatrixWidget.showRealImagesInGrayScales = true;
    confusionMatrixWidget.setShowCurrentPixelPosition(true);
    confusionMatrixWidget.useFastScaling = true;
    ImageWidget confusionMatrixFastWidget;
    confusionMatrixFastWidget.setWindowTitle("Bag-of-words confusion matrix");
    confusionMatrixFastWidget.showRealImagesInGrayScales = true;
    confusionMatrixFastWidget.setShowCurrentPixelPosition(true);
    confusionMatrixFastWidget.useFastScaling = true;
    ImageWidget confusionMatrixGroundTruthWidget;
    confusionMatrixGroundTruthWidget.setWindowTitle("Ground truth confusion matrix");
    confusionMatrixGroundTruthWidget.showRealImagesInGrayScales = true;
    confusionMatrixGroundTruthWidget.useFastScaling = true;

    vector<vector<double> > neededTimesPerScan;

    SPModel dictionary;

    std::string showMap = "SLAM";

    // Run main loop.
    while (window.isVisible()) {
        application.processEvents();
        usleep(100000);

        bool updateParams = false;
        DO_EVERY(1.0, updateParams=true;);
        if (updateParams) {
            parametersManager.updateParameters();
            parametersManager.getParameterValue("narfPlaceRecognition", "maxNoOfThreads", maxNoOfThreads);
            parametersManager.getParameterValue("narfPlaceRecognition", "useRotationInvariance", useRotationInvariance);
            parametersManager.getParameterValue("narfPlaceRecognition", "maximumRange", maximumRange);
            parametersManager.getParameterValue("narfPlaceRecognition", "minValidationPointScoreBeforeICP",
                    rangeImageMatching.parameters.minValidationPointScoreBeforeICP);
            parametersManager.getParameterValue("narfPlaceRecognition", "minValidationPointScore",
                    rangeImageMatching.parameters.minValidationPointScore);
            parametersManager.getParameterValue("narfPlaceRecognition", "maxDescriptorDistance",
                    rangeImageMatching.parameters.maxDescriptorDistance);
            parametersManager.getParameterValue("narfPlaceRecognition", "icpNumIterations",
                    rangeImageMatching.parameters.icpNumIterations);
            parametersManager.getParameterValue("narfPlaceRecognition", "icpPixelSearchRadius",
                    rangeImageMatching.parameters.icpPixelSearchRadius);
            parametersManager.getParameterValue("narfPlaceRecognition", "mapCellSize", mapCellSize);
            //parametersManager.getParameterValue("narfPlaceRecognition", "maxDescrDistForClustering", maxDescrDistForClustering);
            parametersManager.getParameterValue("narfPlaceRecognition", "max_no_of_keypoints",
                    keyPointDetector.getParameters().max_no_of_keypoints);
            parametersManager.getParameterValue("narfPlaceRecognition", "max_no_of_dictionary_keypoints",
                    keyPointDetectorForDictionary.getParameters().max_no_of_keypoints);
            parametersManager.getParameterValue("narfPlaceRecognition", "doSelfSimilarityScoreAdjustment",
                    rangeImageMatching.parameters.doSelfSimilarityScoreAdjustment);
            parametersManager.getParameterValue("narfPlaceRecognition", "minScoreToAddToGraph", minScoreToAddToGraph);
            parametersManager.getParameterValue("narfPlaceRecognition", "maxNoOfPoseEstimatesPerScanPair",
                    rangeImageMatching.parameters.maxNoOfPoseEstimatesPerScanPair);
            parametersManager.getParameterValue("narfPlaceRecognition", "maxNoOfPoseEstimatesPerScanPairRotInv",
                    rangeImageMatching.parameters.maxNoOfPoseEstimatesPerScanPairRotInv);
            parametersManager.getParameterValue("narfPlaceRecognition", "timeLimitForDatabaseQuery", timeLimitForDatabaseQuery);
            parametersManager.getParameterValue("narfPlaceRecognition", "dictionarySize", dictionarySize);
        }

        // Menu->View
        if (viewPointCloud != window.menu_View_PointCloud->isChecked()) {
            viewPointCloud = window.menu_View_PointCloud->isChecked();
            if (viewPointCloud) {
                viewer.add(&visPointCloud);
                viewer2.add(&visPointCloud2);
                //viewer2.add(&visTransformedPointCloud);
            }
            else {
                viewer.remove(&visPointCloud);
                viewer2.remove(&visPointCloud2);
            }
        }
        if (viewPointCloudFromRangeImage != window.menu_View_PointCloudFromRangeImage->isChecked()) {
            viewPointCloudFromRangeImage = window.menu_View_PointCloudFromRangeImage->isChecked();
            if (viewPointCloudFromRangeImage) {
                viewer.add(&visRangeImageCloud);
                viewer2.add(&visRangeImageCloud2);
            }
            else {
                viewer.remove(&visRangeImageCloud);
                viewer2.remove(&visRangeImageCloud2);
            }
        }
        if (viewFeatures != window.menu_View_Features->isChecked()) {
            viewFeatures = window.menu_View_Features->isChecked();
            if (viewFeatures) {
                viewer.add(&visFeaturePositions1);
                viewer2.add(&visFeaturePositions2);
            }
            else {
                viewer.remove(&visFeaturePositions1);
                viewer2.remove(&visFeaturePositions2);
            }
        }
        if (viewValidationPoints != window.menu_View_ValidationPoints->isChecked()) {
            viewValidationPoints = window.menu_View_ValidationPoints->isChecked();
            if (viewValidationPoints) {
                viewer.add(&visValidationPoints1);
                viewer2.add(&visValidationPoints2);
            }
            else {
                viewer.remove(&visValidationPoints1);
                viewer2.remove(&visValidationPoints2);
            }
        }

        // Reset scan
        if (window.menu_PointCloud_Reset->isChecked()) {
            window.menu_PointCloud_Reset->setChecked(false);
            scan1.reset();
            scan1Changed = true;
        }
        if (window.menu_PointCloud2_Reset->isChecked()) {
            window.menu_PointCloud2_Reset->setChecked(false);
            scan2.reset();
            scan2Changed = true;
        }

        // Load new point cloud from harddisk
        if (window.menu_PointCloud_Load->isChecked() || window.menu_PointCloud2_Load->isChecked()) {
            ScanDatabaseElement& scan = (window.menu_PointCloud_Load->isChecked() ? scan1 : scan2);
            scan1Changed=window.menu_PointCloud_Load->isChecked();
            scan2Changed=window.menu_PointCloud2_Load->isChecked();
            window.menu_PointCloud_Load->setChecked(false);  window.menu_PointCloud2_Load->setChecked(false);
            std::string fileName;
            if (!window.loadUsingQFileDialog("scene.pcd", "Point Cloud (*.pcd)", fileName))
                continue;
            scan.reset();
            scan.pointCloudFileName = fileName;
        }

        // PointCloud changed
        while (scan1Changed || scan2Changed) {
            ScanDatabaseElement& scan = (scan1Changed ? scan1 : scan2);
            ImageWidget& imageWidget = *(scan1Changed ? window.imageWidget1 : window.imageWidget2);
            pcl::PointCloud<pcl::PointXYZ>& featurePositions = (scan1Changed ? featurePositions1 : featurePositions2);
            pcl::PointCloud<pcl::PointXYZ>& validationPoints = (scan1Changed ? validationPoints1 : validationPoints2);
            if (scan1Changed && scan2Changed)
                scan1Changed = false;
            else
                scan1Changed = scan2Changed = false;

            window.statusFrame->show();
            window.progressBar->setMaximum(3);
            window.progressBar->setValue(0);
            window.statusLabel->setText("Loading point cloud from disc...");
            application.processEvents();

            // Load point cloud from disc - if not already existing
            if (!scan.pointCloudFileName.empty() && scan.pointCloud.points.empty())
                scan.loadPointCloud();
            //else
            //std::cout << "Did not need to load point cloud, since it already exists.\n";

            window.progressBar->setValue(1);
            window.statusLabel->setText("Creating range image...");
            application.processEvents();

            // Create range image from the point cloud - if not already existing
            if (!scan.pointCloud.points.empty() && scan.rangeImage.points.empty())
                scan.createRangeImage(angularResolution, noiseLevel, coordinateFrame, maximumRange);
            //else
            //std::cout << "Did not need to calculate range image, since it already exists.\n";

            window.progressBar->setValue(2);
            window.statusLabel->setText("Extracting features...");
            application.processEvents();

            //borderExtractor.setRangeImage(&scan.rangeImage);
            //static ImageWidget* surfaceChangeWidget = new ImageWidget;
            //surfaceChangeWidget->showRealImagesInGrayScales = true;
            //surfaceChangeWidget->setWindowTitle("Surface changes");
            //surfaceChangeWidget->setRealImage(borderExtractor.getSurfaceChangeScores(), scan.rangeImage.width, scan.rangeImage.height);
            //borderExtractor.setRangeImage(NULL);

            //keyPointDetector.setRangeImage(&scan.rangeImage);
            //static ImageWidget* interestValueWidget = new ImageWidget;
            //interestValueWidget->setWindowTitle("Interest values");
            //interestValueWidget->setRealImage(keyPointDetector.getInterestImage(), scan.rangeImage.width, scan.rangeImage.height);
            //keyPointDetector.setRangeImage(NULL);

            // Extract features from the range image - if not already existing
            keyPointDetector.getParameters().max_no_of_threads = maxNoOfThreads;

            if (scan.features.empty())
                scan.extractNARFs(supportSize, descriptorSize, useRotationInvariance, keyPointDetector, true);
            //else
            //std::cout << "Did not need to extract features, since they already exist.\n";

            //TEST
            //featureScores1.resize(scan1.features.size());
            //for (size_t featureIdx=0; featureIdx<scan1.features.size(); ++featureIdx) {
            //float & featureScore = featureScores1[featureIdx];
            //featureScore = 0.0f;
            //for (size_t featureIdx2=0; featureIdx2<scan1.features.size(); ++featureIdx2)
            //featureScore += scan1.features[featureIdx]->getDescriptorDistance(*scan1.features[featureIdx2]);
            //featureScore /= scan1.features.size();
            //cout << PVARC(featureIdx)<<PVARN(featureScore);
            //}

            // Extract validation points - if not already existing
            window.progressBar->setValue(3);
            window.statusLabel->setText("Extracting validation points...");
            application.processEvents();
            if (scan.validationPoints.empty())
                //scan.extractValidationPoints(noOfValidationPoints, keyPointDetector, minSurfaceChangeForValidationPoints);
                scan.extractValidationPoints(noOfValidationPoints, keyPointDetector, true);
            //else
            //std::cout << "Did not need to extract validation points, since they already exist.\n";
            keyPointDetector.setRangeImage(NULL);

            scan.selfSimilarity = rangeImageMatching.getSelfSimilarity(scan);

            keyPointDetector.getParameters().max_no_of_threads = 1;

            window.progressBar->setValue(4);
            window.statusLabel->setText("Done.");
            application.processEvents();

            if (scan.rangeImage.points.empty()) {
                imageWidget.reset();
                imageWidget.hide();
            }
            else {
                float* rangesArray = scan.rangeImage.getRangesArray();
                imageWidget.setRealImage(rangesArray, scan.rangeImage.width, scan.rangeImage.height);
                //(new ImageWidget)->setRealImage(rangesArray, scan.rangeImage.width, scan.rangeImage.height);
                delete[] rangesArray;
            }

            if (scan1.rangeImage.points.empty() && scan2.rangeImage.points.empty())
                window.imagesFrame->hide();
            else
                window.imagesFrame->show();

            featurePositions.points.clear();
            featurePositions.points.resize(scan.features.size());
            for (unsigned int feature_idx=0; feature_idx<scan.features.size(); ++feature_idx)
                featurePositions.points[feature_idx].getVector3fMap() = scan.features[feature_idx]->getPosition();

            validationPoints.points.clear();
            validationPoints.points.resize(scan.validationPoints.size());
            for (unsigned int vp_idx=0; vp_idx<scan.validationPoints.size(); ++vp_idx)
                validationPoints.points[vp_idx].getVector3fMap() = scan.validationPoints[vp_idx];

            window.statusFrame->hide();

            clearTransformations = true;
            needDraw = true;

            //featureList.clearAndEraseFeatures();
            //featurePoints.clear();

            //if (!scan1.pointCloud.empty()) {
            //extractFeatures = true;
            //}
            //else {
            //window.imageWidget1->hide();
            //}
        }

        // Load scan database from disc
        if (window.menu_Database_Load->isChecked() || !directoryName.empty()) {
            window.menu_Database_Load->setChecked(false);
            if (directoryName.empty())
                if (!window.loadUsingQFileDialog("", "Dataset directory (*)", directoryName, true))
                    continue;
            if (directoryName[directoryName.size()-1]=='/')
                directoryName.resize(directoryName.size()-1);
            cout << "Loading database from disc: \""<<directoryName<<"\".\n";
            scanDatabase.load(directoryName);

            string statisticsDirectoryPath = scanDatabase.databaseDirectory+"/"+statisticsDirectory;
            boost::filesystem::create_directory(statisticsDirectoryPath);

            saveResult("noOfScans", scanDatabase.size());

            cout << "\nconfigFileParams:\n-----------------\n";
            const std::map<std::string, std::string>& configFileParams = scanDatabase.configFileParams;

            float maxValidationPointError;
            coordinateFrame = static_cast<pcl::RangeImage::CoordinateFrame>(-1);
            noiseLevel = angularResolution = supportSize = supportSizeDictionary =
                maxValidationPointError = maxDistanceToConsiderScansOverlapping = -1;

            averageDistanceBetweenConsecutiveScans = averageRangeMeasurement = maximumRangeInDataset = -1;

            for (std::map<std::string, std::string>::const_iterator it=configFileParams.begin(); it!=configFileParams.end(); ++it) {
                const std::string& paramName  = it->first,
                      & paramValue = it->second;
                //cout << paramName << " = " << paramValue << "\n";
                if (paramName == "angularResolution") {
                    angularResolution = strtod(paramValue.c_str(), NULL);
                    angularResolution = deg2rad(angularResolution);
                    cout << PVARAN(angularResolution);
                }
                if (paramName == "coordinateFrame") {
                    coordinateFrame = static_cast<pcl::RangeImage::CoordinateFrame>(strtol(paramValue.c_str(), NULL, 0));
                    cout << PVARN(coordinateFrame);
                }
                if (paramName == "noiseLevel") {
                    noiseLevel = strtod(paramValue.c_str(), NULL);
                    cout << PVARN(noiseLevel);
                }
                else if (paramName == "averageDistanceBetweenConsecutiveScans") {
                    averageDistanceBetweenConsecutiveScans = strtod(paramValue.c_str(), NULL);
                    cout << PVAR(averageDistanceBetweenConsecutiveScans)<<"m\n";
                }
                else if (paramName == "averageRangeMeasurement") {
                    averageRangeMeasurement = strtod(paramValue.c_str(), NULL);
                    cout << PVAR(averageRangeMeasurement)<<"m\n";
                }
                else if (paramName == "maximumRangeInDataset") {
                    maximumRangeInDataset = strtod(paramValue.c_str(), NULL);
                    cout << PVAR(maximumRangeInDataset)<<"m\n";
                }
                else if (paramName == "supportSize") {
                    supportSize = strtod(paramValue.c_str(), NULL);
                    cout << PVAR(supportSize)<<"m\n";
                    keyPointDetector.getParameters().support_size = supportSize;
                    rangeImageMatching.parameters.minDistanceBetweenMatches = 0.5f*supportSize;
                }
                else if (paramName == "supportSizeDictionary") {
                    supportSizeDictionary = strtod(paramValue.c_str(), NULL);
                    cout << PVAR(supportSizeDictionary)<<"m\n";
                    keyPointDetectorForDictionary.getParameters().support_size = supportSizeDictionary;
                }
                else if (paramName == "maxValidationPointError") {
                    maxValidationPointError = strtod(paramValue.c_str(), NULL);
                    cout << PVAR(maxValidationPointError)<<"m\n";
                    rangeImageMatching.parameters.maxValidationPointError = maxValidationPointError;
                }
                else if (paramName == "maxDistanceToConsiderScansOverlapping") {
                    maxDistanceToConsiderScansOverlapping = strtod(paramValue.c_str(), NULL);
                    cout << PVAR(maxDistanceToConsiderScansOverlapping)<<"m\n";
                }

                saveResult(paramName, paramValue);
            }
            cout << "\n";

            if (angularResolution < 0) {
                angularResolution = defaultAngularResolution;
                scanDatabase.configFileParams["angularResolution"] = toString(rad2deg(angularResolution));
                cout << "angularResolution is not set in param config file. Setting it to the default value: "<<rad2deg(angularResolution)<<"deg.\n";
            }
            if (coordinateFrame < 0) {
                coordinateFrame = defaultCoordinateFrame;
                scanDatabase.configFileParams["coordinateFrame"] = toString(coordinateFrame);
                cout << "coordinateFrame is not set in param config file. Setting it to the default value: "<<coordinateFrame<<".\n";
            }

            if (noiseLevel < 0) {
                noiseLevel = defaultNoiseLevel;
                scanDatabase.configFileParams["noiseLevel"] = toString(noiseLevel);
                cout << "noiseLevel is not set in param config file. Setting it to the default value: "<<noiseLevel<<"m.\n";
            }
            if (supportSize < 0) {
                supportSize = defaultSupportSize;
                scanDatabase.configFileParams["supportSize"] = toString(supportSize);
                keyPointDetector.getParameters().support_size = supportSize;
                rangeImageMatching.parameters.minDistanceBetweenMatches = 0.5f*supportSize;
                cout << "supportSize is not set in param config file. Setting it to the default value: "<<supportSize<<"m.\n";
            }
            if (supportSizeDictionary < 0) {
                supportSizeDictionary = defaultSupportSizeDictionary;
                scanDatabase.configFileParams["supportSizeDictionary"] = toString(supportSizeDictionary);
                keyPointDetectorForDictionary.getParameters().support_size = supportSizeDictionary;
                cout << "supportSizeDictionary is not set in param config file. Setting it to the default value: "<<supportSizeDictionary<<"m.\n";
            }
            if (maxValidationPointError < 0) {
                maxValidationPointError = defaultMaxValidationPointError;
                scanDatabase.configFileParams["maxValidationPointError"] = toString(maxValidationPointError);
                rangeImageMatching.parameters.maxValidationPointError = maxValidationPointError;
                cout << "maxValidationPointError is not set in param config file. Setting it to the default value: "<<maxValidationPointError<<"m.\n";
            }
            if (maxDistanceToConsiderScansOverlapping < 0) {
                maxDistanceToConsiderScansOverlapping = defaultMaxDistanceToConsiderScansOverlapping;
                scanDatabase.configFileParams["maxDistanceToConsiderScansOverlapping"] = toString(maxDistanceToConsiderScansOverlapping);
                cout << "maxDistanceToConsiderScansOverlapping is not set in param config file."
                    << " Setting it to the default value: "<<maxDistanceToConsiderScansOverlapping<<"m.\n";
            }
            if (scanDatabase.configFileParams["shortName"].empty())
                scanDatabase.configFileParams["shortName"] = "scanDatabase";
            scanDatabase.saveConfigFile();

            saveResult("weightForSeeThroughValidationPoint", rangeImageMatching.parameters.weightForSeeThroughValidationPoint, false);
            saveResult("weightForKnownObstacleValidationPoint", rangeImageMatching.parameters.weightForKnownObstacleValidationPoint, false);
            saveResult("weightForUnknownObstacleValidationPoint", rangeImageMatching.parameters.weightForUnknownObstacleValidationPoint, false);
            saveResult("weightForFarRangeValidationPoint", rangeImageMatching.parameters.weightForFarRangeValidationPoint, false);
            saveResult("pixelSearchRadiusForValidationPointsScore", rangeImageMatching.parameters.pixelSearchRadiusForValidationPointsScore, false);
            saveResult("noOfValidationPoints", noOfValidationPoints, false);
            saveResult("minNoOfTestedValidationPoints", rangeImageMatching.parameters.minNoOfTestedValidationPoints, false);
            saveResult("descriptorSize", descriptorSize, false);
            saveResult("maxNoOfKeypoints", keyPointDetector.getParameters().max_no_of_keypoints, false);
            saveResult("maxNoOfDictionaryKeypoints", keyPointDetectorForDictionary.getParameters().max_no_of_keypoints, false);
            saveResult("maxDescriptorDistance", rangeImageMatching.parameters.maxDescriptorDistance, false);
            saveResult("dictionarySize", dictionarySize, false);
            saveResult("maxNoOfPoseEstimatesPerScanPair", rangeImageMatching.parameters.maxNoOfPoseEstimatesPerScanPair, false);
            saveResult("maxNoOfPoseEstimatesPerScanPairRotInv", rangeImageMatching.parameters.maxNoOfPoseEstimatesPerScanPairRotInv, false);
            saveResult("timeLimitForDatabaseQuery", timeLimitForDatabaseQuery, false);

            dictionary.clear();
            string rotInv = (useRotationInvariance ? "RotInv" : "");
            std::string dictionaryFileName = scanDatabase.databaseDirectory + "/../dictionary"+rotInv+".dat";
            std::ifstream dictionaryFile(dictionaryFileName.c_str());
            if (!dictionaryFile)
                std::cerr << "Could not open dictionary file \""<<dictionaryFileName<<"\".\n";
            else {
                dictionary.readBinary(dictionaryFile);
            }
            dictionaryFile.close();

            directoryName.clear();
            databaseChanged = true;
        }

        // Clear the scan database
        if (window.menu_Database_Reset->isChecked()) {
            window.menu_Database_Reset->setChecked(false);
            cout << "Clearing scan database.\n";
            scanDatabase.clear();
            databaseChanged = true;
        }

        if (databaseChanged) {
            databaseChanged = false;

            visScanDatabaseList.clear();
            window.setWindowTitle(windowTitle);
            if (scanDatabase.empty()) {
                visScanDatabaseList.hide();
                window.update();
                continue;
            }
            window.setWindowTitle(  windowTitle+" - "+scanDatabase.name.c_str()
                    +" ("+QString::number(scanDatabase.size())+" scans)");

            for (unsigned int i=0; i<scanDatabase.size(); ++i)
                visScanDatabaseList.insertItem(i, QString::fromAscii("Scan ")+QString::number(i+1));
            visScanDatabaseList.setCurrentRow(0);
            visScanDatabaseList.show();
            markedScanIdx = -1;

            confusionMatrixScores.clear();
            confusionMatrixFastScores.clear();
            confusionMatrixFastSource = CMFS_EMPTY;
            confusionMatrixTransformations.clear();
            confusionMatrixFastTransformations.clear();
            confusionMatrixWidget.hide();
            confusionMatrixFastWidget.hide();
            confusionMatrixGroundTruthWidget.hide();
            //scanDatabase.updateFeatureSourceIds();
            //scanDatabase.updateCombinedFeatureList();
            trajectory.clear();
            mapCloud.points.clear();
            markedScan.points.clear();

            //window.menu_Database_LoadConfusionMatrix->setChecked(true);

            window.update();
        }

        // Copy a scan from the database to the viewer
        if (window.menu_Database_CopySelectedScanToLeft->isChecked()) {
            window.menu_Database_CopySelectedScanToLeft->setChecked(false);
            scan1.reset();
            scan1Changed = true;
            if (scanDatabase.empty())
                continue;
            int scanIdx = visScanDatabaseList.currentRow();
            scan1 = *scanDatabase[scanIdx];
        }
        // Copy a scan from the database to the right point cloud
        if (window.menu_Database_CopySelectedScanToRight->isChecked()) {
            window.menu_Database_CopySelectedScanToRight->setChecked(false);
            scan2.reset();
            scan2Changed = true;
            if (scanDatabase.empty())
                continue;
            int scanIdx = visScanDatabaseList.currentRow();
            scan2 = *scanDatabase[scanIdx];
        }

        // Recalculate range images
        if (window.menu_Database_RecalculateAndSaveRangeImages->isChecked()) {
            recalculateAndSaveRangeImages(application, maxNoOfThreads, maximumRange);
        }

        // Recalculate database data
        if (window.menu_Database_RecalculateAndSaveAll->isChecked()) {
            recalculateAndSaveAll(application, keyPointDetector, maxNoOfThreads, maximumRange);
        }

        // Recalculate dictionary features
        if (window.menu_Database_RecalculateDictionaryFeatures->isChecked()) {
            window.menu_Database_RecalculateDictionaryFeatures->setChecked(false);

            int noOfScans = scanDatabase.size();

            window.statusFrame->show();
            window.progressBar->setMaximum(scanDatabase.size());
            window.statusLabel->setText("Extracting dictionary features...");
            application.processEvents();

            keyPointDetectorForDictionary.getParameters().max_no_of_threads = maxNoOfThreads;
            pcl::Narf::max_no_of_threads = maxNoOfThreads;

            double startTime = get_time();
            int noOfDoneScans = 0;

            bool done = false;
            omp_set_nested(1);
#     pragma omp parallel num_threads(2) default(shared)
            {
                if(omp_get_thread_num() == 0) {
                    int lastNoOfDoneScans = -1;
                    while (!done) {
                        int currentNoOfDoneScans = noOfDoneScans;
                        if (noOfDoneScans != lastNoOfDoneScans) {
                            double eta = (currentNoOfDoneScans>0 ? (noOfScans-currentNoOfDoneScans) * (get_time()-startTime)/currentNoOfDoneScans : -1.0),
                                   etaHours = floor(eta/3600), etaMinutes = floor((eta-3600*etaHours)/60), etaSeconds = round(eta - 3600*etaHours - 60*etaMinutes);
                            QString etaString = (eta>0 ? "  ETA: "+QString::number(etaHours)+"h "+QString::number(etaMinutes)+"m "+QString::number(etaSeconds)+"s" : "");
                            window.progressBar->setValue(currentNoOfDoneScans);
                            window.statusLabel->setText("Extracting dictionary features - Done scans: "+QString::number(noOfDoneScans)+"."+etaString);
                            application.processEvents();
                            lastNoOfDoneScans = currentNoOfDoneScans;
                        }
                        usleep(100000);
                    }
                }
                else {
#         pragma omp parallel for num_threads(maxNoOfThreads) default(shared) schedule(dynamic, 1)
                    for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx)
                    {
                        ScanDatabaseElement& scan = *scanDatabase[scanIdx];
                        //pcl::RangeImageBorderExtractor tmpBorderExtractor;
                        //tmpBorderExtractor.getParameters() = borderExtractorForDictionary.getParameters();
                        //pcl::NarfKeypoint tmpKeyPointDetector(&tmpBorderExtractor);
                        //tmpKeyPointDetector.getParameters() = keyPointDetectorForDictionary.getParameters();
                        MyPcl::NarfssKeypoint tmpKeyPointDetector;  // For multithreading
                        tmpKeyPointDetector.getParameters() = keyPointDetectorForDictionary.getParameters();
                        scan.extractDictionaryNARFs(supportSizeDictionary, descriptorSize, useRotationInvariance, tmpKeyPointDetector);
                        scan.saveDictionaryNARFs();
                        scan.resetDictionaryFeatures();

#           pragma omp critical
                        {
                            ++noOfDoneScans;
                        }
                    }
                    done = true;
                }
            }
            omp_set_nested(0);

            double timeForDictionaryFeatureExtraction = getTime()-startTime;

            window.statusFrame->hide();

            keyPointDetectorForDictionary.getParameters().max_no_of_threads = 1;
            pcl::Narf::max_no_of_threads = 1;

            cout << "Extracting dictionary features for all scans took "<<timeForDictionaryFeatureExtraction<<"s.\n";
            saveResult("timeForDictionaryFeatureExtractionPerScan", round(1000.0f*timeForDictionaryFeatureExtraction/scanDatabase.size()));
        }

        if (window.menu_Database_SaveRangeImages->isChecked()) {
            window.menu_Database_SaveRangeImages->setChecked(false);

            window.statusFrame->show();
            window.progressBar->setMaximum(scanDatabase.size());
            window.statusLabel->setText("Saving range images...");
            application.processEvents();

            for (unsigned int scanIdx=0; scanIdx<scanDatabase.size(); ++scanIdx) {
                window.progressBar->setValue(scanIdx);
                application.processEvents();
                scanDatabase[scanIdx]->saveRangeImage();
            }
            window.statusFrame->hide();
        }

        if (window.menu_Database_SaveFeaturesAndValidationPoints->isChecked()) {
            window.menu_Database_SaveFeaturesAndValidationPoints->setChecked(false);

            window.statusFrame->show();
            window.progressBar->setMaximum(scanDatabase.size());
            window.statusLabel->setText("Saving features...");

            for (unsigned int scanIdx=0; scanIdx<scanDatabase.size(); ++scanIdx) {
                window.progressBar->setValue(scanIdx);
                application.processEvents();

                scanDatabase[scanIdx]->saveNARFs();
                scanDatabase[scanIdx]->saveValidationPoints();
            }

            window.statusFrame->hide();
        }
        if (window.menu_Database_UpdateAndSaveSelfSimilarities->isChecked()) {
            window.menu_Database_UpdateAndSaveSelfSimilarities->setChecked(false);

            window.statusFrame->show();
            window.progressBar->setMaximum(scanDatabase.size());
            window.statusLabel->setText("Updating self similarity values...");

            rangeImageMatching.parameters.maxNoOfThreadsPerScanComparison = maxNoOfThreads;
            for (unsigned int scanIdx=0; scanIdx<scanDatabase.size(); ++scanIdx)
            {
                window.progressBar->setValue(scanIdx);
                application.processEvents();
                ScanDatabaseElement& scan = *scanDatabase[scanIdx];
                scan.selfSimilarity = rangeImageMatching.getSelfSimilarity(scan);
                scan.saveInfoFile();
            }
            rangeImageMatching.parameters.maxNoOfThreadsPerScanComparison = 1;
            window.statusFrame->hide();
        }

        //// A new point was selected in viewer
        //if (window.selectedPointChanged) {
        //window.selectedPointChanged = false;
        ////cout << "New selected point: "<<window.selectedPoint<<"\n";
        //}

        // Recalculate range image, features etc. for point cloud
        if (window.menu_PointCloud_Recalculate->isChecked()) {
            window.menu_PointCloud_Recalculate->setChecked(false);
            scan1.resetRangeImage();
            scan1.resetFeatures();
            scan1.resetValidationPoints();
            scan1Changed = true;
        }
        // Recalculate range image, features etc. for point cloud 2
        if (window.menu_PointCloud2_Recalculate->isChecked()) {
            window.menu_PointCloud2_Recalculate->setChecked(false);
            scan2.resetClouds();
            scan2.resetRangeImage();
            scan2.resetFeatures();
            scan2.resetValidationPoints();
            scan2Changed = true;
        }

        if (clearTransformations) {
            clearTransformations = false;
            if (!ignoreClearTransformationsCall) {
                transformedPointCloud.points.clear();
                transformedPointCloudSlam.points.clear();
                markedTransformationIdx = -1;
                visTransformationsList.clear();
                visTransformationsList.hide();
                window.confusionMatrixManipulationFrame->hide();
                poseEstimates.clear();
                usedFeatures1.points.clear();
                usedFeatures2.points.clear();
                needDraw = true;
            }
            ignoreClearTransformationsCall = false;
        }

        //// Copy left point cloud to right point cloud
        //if (window.rightArrowButton->isChecked()) {
        //window.rightArrowButton->setChecked(false);
        //scan2.pointCloud = scan1.pointCloud;
        //scan2Changed = true;
        //}

        //// Copy right point cloud to left point cloud
        //if (window.leftArrowButton->isChecked()) {
        //window.leftArrowButton->setChecked(false);
        //scan1.pointCloud = scan2.pointCloud;
        //scan1Changed = true;
        //}

        // Match the two range images
        if (window.menu_Process_Match->isChecked()) {
            window.menu_Process_Match->setChecked(false);
            rangeImageMatching.getBestPoseEstimates(scan1, scan2, featureMatches, poseEstimates);
            poseEstimatesType = 1;  // leftRight
            //cout << PVARN(featureMatches.size());
            poseEstimatesChanged = true;
        }

        int scanToMatchAgainstDatabaseIdx = 0;
        bool matchScanAgainstDatabase = false;
        if (window.menu_Database_MatchLeft->isChecked()) {
            window.menu_Database_MatchLeft->setChecked(false);
            scanToMatchAgainstDatabaseIdx = -1;
            matchScanAgainstDatabase = true;
        }
        if (window.menu_Database_MatchRight->isChecked()) {
            window.menu_Database_MatchRight->setChecked(false);
            scanToMatchAgainstDatabaseIdx = -2;
            matchScanAgainstDatabase = true;
        }
        if (window.menu_Database_MatchSelectedScan->isChecked()) {
            window.menu_Database_MatchSelectedScan->setChecked(false);
            scanToMatchAgainstDatabaseIdx = visScanDatabaseList.currentRow();
            matchScanAgainstDatabase = true;
        }

        // Match a single scan against the database
        if (matchScanAgainstDatabase) {
            matchScanAgainstDatabase = false;
            int noOfScans = scanDatabase.size();
            ScanDatabaseElement& scan = (scanToMatchAgainstDatabaseIdx >= 0  ?
                    *scanDatabase[scanToMatchAgainstDatabaseIdx] :
                    (scanToMatchAgainstDatabaseIdx == -1 ? scan1 : scan2));
            std::vector<pcl::PosesFromMatches::PoseEstimatesVector> poseEstimatesPerScan;
            poseEstimatesPerScan.resize(scanDatabase.size());

            rangeImageMatching.parameters.maxNoOfThreadsPerDatabaseSearch = maxNoOfThreads;
            double timeForMatching = -get_time();
            //rangeImageMatching.getBestPoseEstimates(scan, poseEstimatesPerScan);
#pragma omp parallel for default(shared) num_threads(maxNoOfThreads) schedule(dynamic, 1)
            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                rangeImageMatching.getBestPoseEstimates(*scanDatabase[scanIdx], scan, poseEstimatesPerScan[scanIdx]);
            }
            timeForMatching += get_time();
            rangeImageMatching.parameters.maxNoOfThreadsPerDatabaseSearch = 1;

            cout << "Matching ";
            if (scanToMatchAgainstDatabaseIdx==-1)
                cout << "left scan";
            else if (scanToMatchAgainstDatabaseIdx==-2)
                cout << "right scan";
            else
                cout << "scan "<<scanToMatchAgainstDatabaseIdx;
            cout << " against the database took "<<timeForMatching<<"s.\n";

            poseEstimates.clear();
            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                if (scanIdx==scanToMatchAgainstDatabaseIdx)
                    continue;
                //cout << PVARC(scanIdx1)<<PVARC(scanIdx)<<PVARN(poseEstimatesPerScan[scanIdx].size());
                if (!poseEstimatesPerScan[scanIdx].empty()) {
                    pcl::PosesFromMatches::PoseEstimate poseEstimate = poseEstimatesPerScan[scanIdx][0];
                    poseEstimate.correspondence_indices.clear();
                    poseEstimate.correspondence_indices.push_back(scanToMatchAgainstDatabaseIdx);
                    poseEstimate.correspondence_indices.push_back(scanIdx);
                    poseEstimates.push_back(poseEstimate);
                }
            }
            std::sort(poseEstimates.begin(), poseEstimates.end(), pcl::PosesFromMatches::PoseEstimate::IsBetter());

            poseEstimatesChanged = true;
            poseEstimatesType = 2;  // Scan against database
        }

        if (poseEstimatesChanged) {
            poseEstimatesChanged = false;
            visTransformationsList.clear();
            for (size_t poseIdx=0; poseIdx<poseEstimates.size(); ++poseIdx) {
                if (poseIdx >= noOfShownTransformations)
                    break;
                const pcl::PosesFromMatches::PoseEstimate& poseEstimate = poseEstimates[poseIdx];
                if (poseEstimatesType==1)
                    visTransformationsList.insertItem(poseIdx, QString::fromAscii("Score ")+QString::number(poseEstimate.score,'g', 3));
                else if (poseEstimatesType==2 || poseEstimatesType==3)
                    visTransformationsList.insertItem(poseIdx, QString::number(poseEstimate.correspondence_indices[0]+1) +
                            QString::fromAscii(",") +
                            QString::number(poseEstimate.correspondence_indices[1]+1) +
                            QString::fromAscii(": Score ")+QString::number(poseEstimate.score,'g', 3));
                else
                    visTransformationsList.insertItem(poseIdx, QString::fromAscii("Unknown, Score ")+QString::number(poseEstimate.score,'g', 3));
            }
            visTransformationsList.setCurrentRow(0);
            visTransformationsList.show();

            markedTransformationIdx = -1;

            window.update();
        }

        if (confusionMatrixWidget.mouseClickHappened) {
            int scanIdx1 = lrintf(confusionMatrixWidget.clickedPixelX),
                scanIdx2 = lrintf(confusionMatrixWidget.clickedPixelY);
            int confusionMatrixIdx = scanIdx2*scanDatabase.size()+scanIdx1;
            cout << "Confusion matrix element "<<scanIdx1+1<<","<<scanIdx2+1<<" was selected "
                << "(score "<<confusionMatrixScores[confusionMatrixIdx]<<").\n";
            confusionMatrixWidget.mouseClickHappened = false;
            pcl::PosesFromMatches::PoseEstimate poseEstimate;
            poseEstimate.score = confusionMatrixScores[confusionMatrixIdx];
            poseEstimate.transformation = confusionMatrixTransformations[confusionMatrixIdx];
            poseEstimate.correspondence_indices.clear();
            poseEstimate.correspondence_indices.push_back(scanIdx1);
            poseEstimate.correspondence_indices.push_back(scanIdx2);

            poseEstimates.clear();
            poseEstimates.push_back(poseEstimate);
            poseEstimatesChanged = true;
            poseEstimatesType = 3;  // selected from matrix

            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(poseEstimate.transformation, x, y, z, roll, pitch, yaw);
            cout << "Our:  "<<scanIdx1+1<<" "<<scanIdx2+1<<" "<<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
            Eigen::Isometry3f transformation = scanDatabase[scanIdx2]->knownPoses["SLAM"].inverse(Eigen::Isometry) *
                scanDatabase[scanIdx1]->knownPoses["SLAM"];
            pcl::getTranslationAndEulerAngles(transformation, x, y, z, roll, pitch, yaw);
            cout << "SLAM: "<<scanIdx1+1<<" "<<scanIdx2+1<<" "<<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
        }

        if (confusionMatrixFastWidget.mouseClickHappened) {
            int scanIdx2 = lrintf(confusionMatrixFastWidget.clickedPixelX),
                scanIdx1 = lrintf(confusionMatrixFastWidget.clickedPixelY);
            int confusionMatrixIdx = scanIdx2*scanDatabase.size()+scanIdx1;
            if (!setConfusionMatrixPoseEstimate) {
                cout << "Fast confusion matrix element "<<scanIdx1+1<<","<<scanIdx2+1<<" was selected "
                    << "(score "<<confusionMatrixFastScores[confusionMatrixIdx]<<").\n";
                scan1 = *scanDatabase[scanIdx1];
                scan2 = *scanDatabase[scanIdx2];
                scan1Changed = scan2Changed = true;
                setConfusionMatrixPoseEstimate = true;
            }
            else {  // Second run
                setConfusionMatrixPoseEstimate = false;
                confusionMatrixFastWidget.mouseClickHappened = false;

                if (!confusionMatrixFastTransformations.empty()) {
                    pcl::PosesFromMatches::PoseEstimate poseEstimate;
                    poseEstimate.score = confusionMatrixFastScores[confusionMatrixIdx];
                    poseEstimate.transformation = confusionMatrixFastTransformations[confusionMatrixIdx];
                    poseEstimate.correspondence_indices.clear();
                    poseEstimate.correspondence_indices.push_back(scanIdx1);
                    poseEstimate.correspondence_indices.push_back(scanIdx2);
                    if (poseEstimate.score > 0.0f) {
                        poseEstimates.push_back(poseEstimate);
                        poseEstimatesChanged = true;
                        poseEstimatesType = 3;  // selected from matrix
                    }
                }
                if (scan1.knownPoses.find("SLAM")!=scan1.knownPoses.end() && scan2.knownPoses.find("SLAM")!=scan2.knownPoses.end()) {
                    Eigen::Isometry3f slamTransformation = (scan2.knownPoses["SLAM"].inverse(Eigen::Isometry) * scan1.knownPoses["SLAM"]).inverse(Eigen::Isometry);
                    pcl::transformPointCloud(scan2.pointCloud, transformedPointCloudSlam, slamTransformation);
                }
            }
        }

        // Clear Confusion matrix
        if (window.menu_Database_ClearConfusionMatrix->isChecked()) {
            window.menu_Database_ClearConfusionMatrix->setChecked(false);
            int noOfScans = scanDatabase.size();
            int confusionMatrizSize = pow(noOfScans, 2);
            confusionMatrixScores.clear();
            confusionMatrixScores.resize(confusionMatrizSize, 0.0f);
            confusionMatrixTransformations.clear();
            confusionMatrixTransformations.resize(confusionMatrizSize, Eigen::Isometry3f::Identity());

            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx)
                confusionMatrixScores[scanIdx*noOfScans+scanIdx] = 1.0f;

            confusionMatrixWidget.setRealImage(&confusionMatrixScores[0], noOfScans, noOfScans, true, 0.0f, 1.0f);
        }


        // Calculate Confusion matrix (SLAM hack)
        bool useSlamHack = false;
        if (window.menu_Database_CalculateConfusionMatrixSlamHack->isChecked()) {
            window.menu_Database_CalculateConfusionMatrixSlamHack->setChecked(false);
            window.menu_Database_CalculateConfusionMatrix->setChecked(true);
            useSlamHack = true;
        }


        // Calculate Confusion matrix
        if (window.menu_Database_CalculateConfusionMatrix->isChecked()) {
            calculateConfusionMatrix(application, confusionMatrixWidget, neededTimesPerScan, maxNoOfThreads, maximumRange, useSlamHack);
        }


        // Read confusion matrix from file
        if (window.menu_Database_LoadConfusionMatrix->isChecked()) {
            window.menu_Database_LoadConfusionMatrix->setChecked(false);

            int noOfScans = scanDatabase.size();
            int confusionMatrizSize = pow(noOfScans, 2);
            confusionMatrixScores.clear();
            confusionMatrixScores.resize(confusionMatrizSize, 0.0f);
            confusionMatrixTransformations.clear();
            confusionMatrixTransformations.resize(confusionMatrizSize, Eigen::Isometry3f::Identity());

            string rotInv = (useRotationInvariance ? "RotInv" : "");
            string settings = rotInv;

            std::string confusionMatrixFileName = scanDatabase.databaseDirectory+"/"+statisticsDirectory+"/confusionMatrix"+settings+"_"+scanDatabase.name+".dat";
            std::ifstream confusionMatrixFile(confusionMatrixFileName.c_str());
            if (!confusionMatrixFile) {
                cerr << "Could not open file \""<<confusionMatrixFileName<<"\".\n";
                continue;
            }
            while (true) {
                unsigned int scanIdx1, scanIdx2;
                float score, x, y, z, roll, pitch, yaw;
                confusionMatrixFile >> scanIdx1 >> scanIdx2 >> score >> x >> y >> z >> roll >> pitch >> yaw;
                //cout << scanIdx1<<"x"<<scanIdx2<<"\n";
                if (confusionMatrixFile.eof() || confusionMatrixFile.fail()) break;
                if (scanIdx1 > scanDatabase.size() || scanIdx1 > scanDatabase.size()) {
                    cerr << PVARC(scanIdx1) << PVARC(scanIdx2) << PVAR(scanDatabase.size())<<": Something's wrong...\n";
                    break;
                }
                --scanIdx1; --scanIdx2;

                int confusionMatrixIdx = scanIdx2*noOfScans+scanIdx1;
                confusionMatrixScores[confusionMatrixIdx] = score;
                confusionMatrixTransformations[confusionMatrixIdx].matrix() =
                    Ais3dTools::TransformationRepresentation::getMatrixFromTranslationAndEuler(x, y, z, roll, pitch, yaw);

                //cout << scanIdx1+1<<"<->"<<scanIdx2+1<<": "<<score<<"\n";
            }
            confusionMatrixWidget.setRealImage(&confusionMatrixScores[0], noOfScans, noOfScans, true, 0.0f, 1.0f);
            cout << "Loading confusion matrix done.\n";
            confusionMatrixFile.close();

            // Read timings from file
            std::string timingsFileName = scanDatabase.databaseDirectory+"/"+statisticsDirectory+"/confusionMatrixTimings"+settings+"_"+scanDatabase.name+".dat";
            std::ifstream timingsFile(timingsFileName.c_str());
            neededTimesPerScan.resize(scanDatabase.size());
            for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
                neededTimesPerScan[scanIdx1].resize(scanDatabase.size());
                for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
                    timingsFile >> neededTimesPerScan[scanIdx1][scanIdx2];
                }
            }
            confusionMatrixFile.close();
        }

        // Create and optimize graph
        bool createAndOptimizeGraph = false,
             useRobustKernel = false;
        if (window.menu_Database_CreateAndOptimizeGraph->isChecked()) {
            window.menu_Database_CreateAndOptimizeGraph->setChecked(false);
            createAndOptimizeGraph = true;
        }
        if (window.menu_Database_CreateAndOptimizeGraphRobust->isChecked()) {
            window.menu_Database_CreateAndOptimizeGraphRobust->setChecked(false);
            createAndOptimizeGraph = true;
            useRobustKernel = true;
        }

        if (createAndOptimizeGraph) {
            if (confusionMatrixScores.empty()) {
                std::cerr << "Confusion matrix is empty!\n";
                continue;
            }

            std::string incrementalPoseType = "IncrementalScanMatching";

            int noOfScans = scanDatabase.size();;
            std::stringstream graphStream;
            bool posesNotAvailable = false;
            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                ScanDatabaseElement& scan = *scanDatabase[scanIdx];
                if (scan.knownPoses.find(incrementalPoseType)==scan.knownPoses.end()) {
                    posesNotAvailable = true;
                    break;
                }
                const Eigen::Isometry3f& poseGuess = scan.knownPoses[incrementalPoseType];
                Eigen::Vector3f translation = poseGuess.translation();
                Eigen::Quaternionf orientation = Eigen::Quaternionf(poseGuess.rotation());
                graphStream << "VERTEX_SE3:QUAT "<<scanIdx<<" "<<translation[0]<<" "<<translation[1]<<" "<<translation[2]<<" "
                    <<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z()<<" "<<orientation.w()<<"\n";
            }
            if (posesNotAvailable) {
                cerr << "Known poses do not contain \""<<incrementalPoseType<<"\"!\n";
                continue;
            }

            for (int scanIdx1=0; scanIdx1<noOfScans-1; ++scanIdx1) {
                ScanDatabaseElement& scan1 = *scanDatabase[scanIdx1];
                const Eigen::Isometry3f& poseGuess1 = scan1.knownPoses[incrementalPoseType];
                for (int scanIdx2=scanIdx1+1; scanIdx2<noOfScans; ++scanIdx2) {
                    if (scanIdx2-scanIdx1 == 1) {  // Incremental?
                        ScanDatabaseElement& scan2 = *scanDatabase[scanIdx2];
                        const Eigen::Isometry3f& poseGuess2 = scan2.knownPoses[incrementalPoseType];
                        Eigen::Isometry3f incrementalMovement = poseGuess1.inverse(Eigen::Isometry)*poseGuess2;
                        Eigen::Vector3f translation = incrementalMovement.translation();
                        Eigen::Quaternionf orientation = Eigen::Quaternionf(incrementalMovement.rotation());
                        graphStream << "EDGE_SE3:QUAT "<<scanIdx1<<" "<<scanIdx2<<" "<<translation[0]<<" "<<translation[1]<<" "<<translation[2]<<" "
                            <<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z()<<" "<<orientation.w()<<" "
                            <<"1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 100 0 0 100 0 100\n";
                        continue;
                    }
                    int confusionMatrixIdx = scanIdx2*noOfScans+scanIdx1;
                    float confusionMatrixScore = confusionMatrixScores[confusionMatrixIdx];
                    const Eigen::Isometry3f& confusionMatrixTransformation = confusionMatrixTransformations[confusionMatrixIdx];
                    if (confusionMatrixScore < minScoreToAddToGraph)
                        continue;
                    Eigen::Vector3f translation = confusionMatrixTransformation.inverse().translation();
                    Eigen::Quaternionf orientation = Eigen::Quaternionf(confusionMatrixTransformation.inverse().rotation());
                    graphStream << "EDGE_SE3:QUAT "<<scanIdx1<<" "<<scanIdx2<<" "<<translation[0]<<" "<<translation[1]<<" "<<translation[2]<<" "
                        <<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z()<<" "<<orientation.w()<<" "
                        <<"1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 100 0 0 100 0 100\n";
                }
            }
            std::ofstream graphFile((scanDatabase.name+".g2o").c_str());
            graphFile << graphStream.str();
            graphFile.close();
            int maxOptimizerIterations = 30;
            g2o::SparseOptimizer optimizer;
            optimizer.setVerbose(true);
            g2o::OptimizationAlgorithmProperty solverProperty;
            g2o::OptimizationAlgorithmFactory* solverFactory = g2o::OptimizationAlgorithmFactory::instance();
            //solverFactory->listSolvers(cout);  std::cout << std::flush;
            optimizer.setAlgorithm(solverFactory->construct("gn_var", solverProperty));
            optimizer.load(graphStream, false);
            g2o::OptimizableGraph::Vertex* gauge = optimizer.findGauge();
            gauge->setFixed(true);
            cerr << "# graph is fixed by node " << gauge->id() << endl;
            optimizer.initializeOptimization();

            if (useRobustKernel) {
                double huberWidth = 1.0;

                bool useRobustKernel = true;
                if (useRobustKernel) {
                    g2o::AbstractRobustKernelCreator* creator = g2o::RobustKernelFactory::instance()->creator("Cauchy");
                    //g2o::AbstractRobustKernelCreator* creator = g2o::RobustKernelFactory::instance()->creator("Saturated");
                    if (!creator) {
                        cerr << "Not a a valid robust kernel" << endl;
                    }
                    else {
                        for (g2o::SparseOptimizer::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
                            g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*it);
                            e->setRobustKernel(creator->construct());
                            e->robustKernel()->setDelta(huberWidth);
                        }
                    }
                }
                //else {
                //for (SparseOptimizer::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
                //OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
                //e->setRobustKernel(0);
                //}
                //}
            }

            optimizer.computeActiveErrors();
            cerr << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;
            int i=optimizer.optimize(maxOptimizerIterations);
            if (maxOptimizerIterations > 0 && !i){
                cerr << "optimize failed, result might be invalid" << endl;
            }
            for (g2o::OptimizableGraph::VertexIDMap::const_iterator it=optimizer.vertices().begin(); it!=optimizer.vertices().end(); ++it) {
                int scanIdx = it->first;
                g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(it->second);
                Eigen::Isometry3f slamPose = vertex->estimate().cast<float>();
                ScanDatabaseElement& scan = *scanDatabase[scanIdx];
                scan.knownPoses["SLAM"] = slamPose;
            }

            if (useRobustKernel) {
                //// Find active edges
                //optimizer.computeActiveErrors();
                //if (e->robustKernel()!=NULL && e->chi2() > e->robustKernel()->delta())
                //deactivated
            }
        }

        // Save graph with ground truth poses (scanIdx as z-axis) and found edges
        if (window.menu_Database_SaveVisualizationGraph->isChecked()) {
            window.menu_Database_SaveVisualizationGraph->setChecked(false);

            if (confusionMatrixScores.empty()) {
                std::cerr << "Confusion matrix is empty!\n";
                continue;
            }
            std::string incrementalPoseType = "SLAM";
            int noOfScans = scanDatabase.size();;
            std::stringstream graphStream;
            bool posesNotAvailable = false;
            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                ScanDatabaseElement& scan = *scanDatabase[scanIdx];
                if (scan.knownPoses.find(incrementalPoseType)==scan.knownPoses.end()) {
                    posesNotAvailable = true;
                    break;
                }
                const Eigen::Isometry3f& poseGuess = scan.knownPoses[incrementalPoseType];
                Eigen::Vector3f translation = poseGuess.translation();
                translation[1] = scanIdx*0.05f*averageDistanceBetweenConsecutiveScans;  // For visualization
                Eigen::Quaternionf orientation = Eigen::Quaternionf(poseGuess.rotation());
                graphStream << "VERTEX_SE3:QUAT "<<scanIdx<<" "<<translation[0]<<" "<<translation[1]<<" "<<translation[2]<<" "
                    <<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z()<<" "<<orientation.w()<<"\n";
            }
            if (posesNotAvailable) {
                cerr << "Known poses do not contain \""<<incrementalPoseType<<"\"!\n";
                continue;
            }
            for (int scanIdx1=0; scanIdx1<noOfScans-1; ++scanIdx1) {
                ScanDatabaseElement& scan1 = *scanDatabase[scanIdx1];
                const Eigen::Isometry3f& poseGuess1 = scan1.knownPoses[incrementalPoseType];
                for (int scanIdx2=scanIdx1+1; scanIdx2<noOfScans; ++scanIdx2) {
                    if (scanIdx2-scanIdx1 == 1) {  // Incremental?
                        ScanDatabaseElement& scan2 = *scanDatabase[scanIdx2];
                        const Eigen::Isometry3f& poseGuess2 = scan2.knownPoses[incrementalPoseType];
                        Eigen::Isometry3f incrementalMovement = poseGuess1.inverse(Eigen::Isometry)*poseGuess2;
                        Eigen::Vector3f translation = incrementalMovement.translation();
                        Eigen::Quaternionf orientation = Eigen::Quaternionf(incrementalMovement.rotation());
                        graphStream << "EDGE_SE3:QUAT "<<scanIdx1<<" "<<scanIdx2<<" "<<translation[0]<<" "<<translation[1]<<" "<<translation[2]<<" "
                            <<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z()<<" "<<orientation.w()<<" "
                            <<"1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 100 0 0 100 0 100\n";
                        continue;
                    }
                    int confusionMatrixIdx = scanIdx2*noOfScans+scanIdx1;
                    float confusionMatrixScore = confusionMatrixScores[confusionMatrixIdx];
                    const Eigen::Isometry3f& confusionMatrixTransformation = confusionMatrixTransformations[confusionMatrixIdx];
                    if (confusionMatrixScore < minScoreToAddToGraph)
                        continue;
                    Eigen::Vector3f translation = confusionMatrixTransformation.translation();
                    Eigen::Quaternionf orientation = Eigen::Quaternionf(confusionMatrixTransformation.rotation());
                    graphStream << "EDGE_SE3:QUAT "<<scanIdx1<<" "<<scanIdx2<<" "<<translation[0]<<" "<<translation[1]<<" "<<translation[2]<<" "
                        <<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z()<<" "<<orientation.w()<<" "
                        <<"1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 100 0 0 100 0 100\n";
                }
            }
            std::ofstream graphFile((scanDatabase.name+"_visualization.g2o").c_str());
            graphFile << graphStream.str();
            graphFile.close();
        }

        // ===============================================
        // Create statistics regarding the current dataset
        // ===============================================
        if (window.menu_Database_CreateDatasetStatistics->isChecked()) {
            createDatasetStatistics(keyPointDetector);
        }

        // ================================================
        // Create statistics regarding the confusion matrix
        // ================================================
        if (window.menu_Database_CreateConfusionMatrixStatistics->isChecked()) {
            window.menu_Database_CreateConfusionMatrixStatistics->setChecked(false);

            if (confusionMatrixScores.empty())
                continue;

            int noOfScans = scanDatabase.size();

            vector<float> distancesToCheck;
            for (float minDistance=0.0; minDistance<=40; minDistance+=0.1)
                distancesToCheck.push_back(minDistance);
            vector<int> truePositives;
            truePositives.resize(distancesToCheck.size(), 0);
            vector<int> falseNegatives;
            falseNegatives.resize(distancesToCheck.size(), 0);

            vector<float> minScoresForFalsePositives;
            for (float minScoreForFalsePositives=0.01; minScoreForFalsePositives<=1.0; minScoreForFalsePositives+=0.01)
                minScoresForFalsePositives.push_back(minScoreForFalsePositives);

            vector<float> maxTimes;
            for (float maxTime=0.01; maxTime<=100.0; maxTime+=0.01)
                maxTimes.push_back(maxTime);
            vector<int> truePositivesPerMaxTime;
            truePositivesPerMaxTime.resize(maxTimes.size(), 0);

            vector<int> falsePositivesPerScore;
            falsePositivesPerScore.resize(minScoresForFalsePositives.size(), 0);
            vector<int> falseTransformationsPerScore;
            falseTransformationsPerScore.resize(minScoresForFalsePositives.size(), 0);

            vector<int> truePositivesForFixedDistance;
            truePositivesForFixedDistance.resize(minScoresForFalsePositives.size(), 0);

            float maximumScoreForFalsePositives = 0.0f,
                  maximumScoreForFalseTransformation = 0.0f,
                  maximumScoreForFalsePositivesLoopOnly = 0.0f;

            int minLoopSize = 30;

            // Now analyse confusion matrix
            vector<int> isFalsePositiveImage;
            isFalsePositiveImage.resize(noOfScans*noOfScans);
            vector<float> trueDistanceImage;
            trueDistanceImage.resize(noOfScans*noOfScans);
            for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
                ScanDatabaseElement& scan1 = *scanDatabase[scanIdx1];
                const Eigen::Isometry3f& truePose1 = scan1.knownPoses["SLAM"];
                for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
                    ScanDatabaseElement& scan2 = *scanDatabase[scanIdx2];
                    const Eigen::Isometry3f& truePose2 = scan2.knownPoses["SLAM"];
                    int confusionMatrixIdx = scanIdx2*noOfScans+scanIdx1;
                    int& isFalsePositive = isFalsePositiveImage[confusionMatrixIdx];
                    float& trueDistance = trueDistanceImage[confusionMatrixIdx];

                    Eigen::Isometry3f truePoseDiff = truePose2.inverse(Eigen::Isometry)*truePose1;
                    Eigen::Vector3f truePosDiff = truePoseDiff.translation();
                    //float trueRoll, truePitch, trueYaw;
                    //pcl::getEulerAngles(truePoseDiff, trueRoll, truePitch, trueYaw);
                    trueDistance = truePosDiff.norm();

                    float score = confusionMatrixScores[confusionMatrixIdx];
                    const Eigen::Isometry3f& estimatedPoseDiff = confusionMatrixTransformations[confusionMatrixIdx];

                    float errorTranslation, errorOrientation;
                    rangeImageMatching.getEstimationError(estimatedPoseDiff, truePoseDiff, errorTranslation, errorOrientation);

                    //float estimatedRoll, estimatedPitch, estimatedYaw;
                    //pcl::getEulerAngles(estimatedPoseDiff, estimatedRoll, estimatedPitch, estimatedYaw);
                    //float estimatedDistance = estimatedPosDiff.norm();
                    //float posError = (estimatedPosDiff-truePosDiff).norm(),
                    //angleError = std::max(anglesDiff(estimatedRoll,trueRoll),
                    //std::max(fabsf(anglesDiff(estimatedPitch,truePitch)), fabsf(anglesDiff(estimatedYaw,trueYaw))));

                    bool isFalseTransformation = errorTranslation > 0.5f*averageRangeMeasurement || errorOrientation > deg2rad(15.0f);
                    isFalsePositive = isFalseTransformation;

                    if (isFalseTransformation) {
                        if (score > maximumScoreForFalseTransformation) {
                            maximumScoreForFalseTransformation = score;
                            //cout << "False transformation ("<<PVARC(errorTranslation)<<PVARA(errorOrientation)<<") with score "<<score<<" at "
                            //<< scanIdx1+1<<","<<scanIdx2+1<<"\n";
                        }

                        if (isFalsePositive && score > maximumScoreForFalsePositives) {
                            maximumScoreForFalsePositives = score;
                            cout << "False positive ("<<PVARC(errorTranslation)<<PVARA(errorOrientation)<<") with score "<<score<<" at "
                                << scanIdx1+1<<","<<scanIdx2+1<<"\n";
                            //cout << "Inverse score is "<<confusionMatrixScores[scanIdx1*scanDatabase.size()+scanIdx2]<<".\n";
                            confusionMatrixWidget.mouseClickHappened = true;
                            confusionMatrixWidget.clickedPixelX = scanIdx1;
                            confusionMatrixWidget.clickedPixelY = scanIdx2;
                        }

                        if (isFalsePositive && abs(scanIdx2-scanIdx1)>=minLoopSize)
                            maximumScoreForFalsePositivesLoopOnly = std::max(score, maximumScoreForFalsePositivesLoopOnly);
                    }
                }
            }

            int noOfOverlappingScans=0, noOfOverlappingScansLoopOnly=0;
            int truePositivesForOverlappingScans=0, truePositivesForOverlappingScansLoopOnly=0;
            int noOfScanPairs = 0;
            //std::ofstream overlappingScansFile("overlappingScans.txt");
            for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
                for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
                    if (scanIdx1==scanIdx2)
                        continue;
                    int confusionMatrixIdx = scanIdx2*noOfScans+scanIdx1;
                    bool isFalsePositive = isFalsePositiveImage[confusionMatrixIdx];
                    float trueDistance = trueDistanceImage[confusionMatrixIdx];
                    float score = confusionMatrixScores[confusionMatrixIdx];
                    float neededTime = neededTimesPerScan[scanIdx1][scanIdx2];

                    ++noOfScanPairs;

                    for (unsigned int distanceNo=0; distanceNo<distancesToCheck.size(); ++distanceNo) {
                        if (trueDistance >= distancesToCheck[distanceNo])
                            continue;
                        if (!isFalsePositive && score>maximumScoreForFalsePositives) {
                            //cout << "True positive (distance "<<trueDistance<<"m)!\n";
                            ++truePositives[distanceNo];
                        }
                        else {
                            //cout << "False positive (distance "<<trueDistance<<"m)!\n";
                            ++falseNegatives[distanceNo];
                        }
                    }

                    if (trueDistance <= maxDistanceToConsiderScansOverlapping) {
                        //if (scanIdx1 < scanIdx2)
                        //overlappingScansFile << scanIdx1<<" "<<scanIdx2<<"\n"<<std::flush;
                        ++noOfOverlappingScans;
                        if (abs(scanIdx2-scanIdx1) >= minLoopSize)
                            ++noOfOverlappingScansLoopOnly;
                        if (score > maximumScoreForFalsePositives) {
                            ++truePositivesForOverlappingScans;
                            for (unsigned int maxTimeIdx=0; maxTimeIdx<maxTimes.size(); ++maxTimeIdx)
                                if (neededTime < maxTimes[maxTimeIdx])
                                    ++truePositivesPerMaxTime[maxTimeIdx];
                        }
                        if (score > maximumScoreForFalsePositivesLoopOnly  &&  abs(scanIdx2-scanIdx1) >= minLoopSize)
                            ++truePositivesForOverlappingScansLoopOnly;
                    }

                    for (unsigned int minScoreNo=0; minScoreNo<minScoresForFalsePositives.size(); ++minScoreNo) {
                        if (score < minScoresForFalsePositives[minScoreNo])
                            continue;
                        if (isFalsePositive) {
                            ++falsePositivesPerScore[minScoreNo];
                        }
                        else if (trueDistance <= maxDistanceToConsiderScansOverlapping) {
                            //if (minScoresForFalsePositives[minScoreNo] > 0.9)
                            //cout << scanIdx1<<","<<scanIdx2<<"\n";
                            ++truePositivesForFixedDistance[minScoreNo];
                        }
                    }
                }
            }
            //overlappingScansFile.close();

            string rotInv = (useRotationInvariance ? "RotInv" : "");
            string settings = rotInv;

            std::ofstream recallRateFile((scanDatabase.databaseDirectory+"/"+statisticsDirectory+"/recallRate"+settings+"_"+scanDatabase.name+".dat").c_str());
            for (unsigned int distanceNo=0; distanceNo<distancesToCheck.size(); ++distanceNo) {
                //cout << truePositives[distanceNo]<<" true positives and "<<falseNegatives[distanceNo]<<" false negatives for distance "<<distancesToCheck[distanceNo]<<"m.\n";
                recallRateFile << distancesToCheck[distanceNo]<<" "<<truePositives[distanceNo]<<" "<<falseNegatives[distanceNo]<<"\n";
            }
            recallRateFile.close();

            std::ofstream falsePositivesAndRecallRateFile((scanDatabase.databaseDirectory+"/"+statisticsDirectory+"/falsePositivesAndRecallRate"+settings+"_"
                        +scanDatabase.name+".dat").c_str());
            for (unsigned int minScoreNo=0; minScoreNo<minScoresForFalsePositives.size(); ++minScoreNo) {
                falsePositivesAndRecallRateFile << minScoresForFalsePositives[minScoreNo] << " "
                    << falsePositivesPerScore[minScoreNo] << " " << noOfScanPairs << " "
                    << truePositivesForFixedDistance[minScoreNo] << " " << noOfOverlappingScans<<"\n";
            }
            falsePositivesAndRecallRateFile.close();

            std::ofstream recallRatePerMaxTimeFile((scanDatabase.databaseDirectory+"/"+statisticsDirectory+"/recallRatePerMaxTime"+settings+"_"
                        +scanDatabase.name+".dat").c_str());
            for (size_t maxTimeIdx=0; maxTimeIdx<maxTimes.size(); ++maxTimeIdx) {
                recallRatePerMaxTimeFile << maxTimes[maxTimeIdx] << " " << truePositivesPerMaxTime[maxTimeIdx] << " " << noOfOverlappingScans<<"\n";
            }
            recallRatePerMaxTimeFile.close();

            maximumScoreForFalsePositives = round(maximumScoreForFalsePositives*100.0f)/100.0f;
            cout << PVARN(maximumScoreForFalsePositives);
            maximumScoreForFalsePositivesLoopOnly = round(maximumScoreForFalsePositivesLoopOnly*100.0f)/100.0f;
            cout << PVARN(maximumScoreForFalsePositivesLoopOnly);
            //cout << PVARN(maximumScoreForFalseTransformation);

            float bestRecallRateForOverlappingScans = float(truePositivesForOverlappingScans)/float(noOfOverlappingScans);
            bestRecallRateForOverlappingScans = round(bestRecallRateForOverlappingScans*1000.0f)/1000.0f;
            cout << PVARN(bestRecallRateForOverlappingScans);

            float bestRecallRateForOverlappingScansLoopOnly = float(truePositivesForOverlappingScansLoopOnly)/float(noOfOverlappingScansLoopOnly);
            bestRecallRateForOverlappingScansLoopOnly = round(bestRecallRateForOverlappingScansLoopOnly*1000.0f)/1000.0f;
            cout << PVARN(bestRecallRateForOverlappingScansLoopOnly);

            saveResult("maximumScoreForFalseTransformation", maximumScoreForFalseTransformation);
            saveResult("maximumScoreForFalsePositives", maximumScoreForFalsePositives);
            saveResult("maximumScoreForFalsePositivesLoopOnly", maximumScoreForFalsePositivesLoopOnly);
            saveResult("bestRecallRateForOverlappingScans", bestRecallRateForOverlappingScans);
            saveResult("bestRecallRateForOverlappingScansLoopOnly", bestRecallRateForOverlappingScansLoopOnly);

            cout << "Analysis of confusion matrix is done.\n";
        }

        // A new transformation was selected in list
        if (markedTransformationIdx != visTransformationsList.currentRow() && visTransformationsList.currentRow() < (int)poseEstimates.size()) {
            markedTransformationIdx = visTransformationsList.currentRow();
            const pcl::PosesFromMatches::PoseEstimate& poseEstimate = poseEstimates[markedTransformationIdx];
            usedFeatures1.points.clear();
            usedFeatures2.points.clear();
            transformedPointCloud.clear();

            if (poseEstimatesType==1) {
                pcl::transformPointCloud(scan2.pointCloud, transformedPointCloud, poseEstimate.transformation);
                for (unsigned int i=0; i<poseEstimate.correspondence_indices.size(); ++i) {
                    pcl::PointXYZ point;
                    point.getVector3fMap() = featureMatches[poseEstimate.correspondence_indices[i]].point1;
                    usedFeatures1.points.push_back(point);
                    point.getVector3fMap() = featureMatches[poseEstimate.correspondence_indices[i]].point2;
                    usedFeatures2.points.push_back(point);
                }
            }
            else if (poseEstimatesType==2 || poseEstimatesType==3) {
                int scanIdx1 = poseEstimate.correspondence_indices[0],
                    scanIdx2 = poseEstimate.correspondence_indices[1];
                //scan1Changed = scan2Changed = true;
                if (scanIdx1 >= 0) {
                    scan1 = *scanDatabase[scanIdx1];
                    scan1Changed = true;
                }
                scan2 = *scanDatabase[scanIdx2];
                if (!scan2.pointCloudFileName.empty() && scan2.pointCloud.points.empty())
                    scan2.loadPointCloud();
                scan2Changed = true;

                if (scan1.knownPoses.find("SLAM")!=scan1.knownPoses.end() && scan2.knownPoses.find("SLAM")!=scan2.knownPoses.end()) {
                    Eigen::Isometry3f slamTransformation = (scan2.knownPoses["SLAM"].inverse() * scan1.knownPoses["SLAM"]).inverse();
                    transformedPointCloudSlam.clear();
                    pcl::transformPointCloud(scan2.pointCloud, transformedPointCloudSlam, slamTransformation);
                }

                ignoreClearTransformationsCall = true;
                pcl::transformPointCloud(scan2.pointCloud, transformedPointCloud, poseEstimate.transformation.inverse(Eigen::Isometry));
                window.confusionMatrixManipulationFrame->show();
            }
            needDraw = true;
        }

        window.confusionMatrixAddButton->setEnabled(poseEstimatesType==2&&!confusionMatrixScores.empty());
        window.confusionMatrixRemoveButton->setEnabled(poseEstimatesType==3&&!confusionMatrixScores.empty());

        // The "Add to matrix"-button was pressed
        if (window.confusionMatrixAddButton->isChecked()) {
            window.confusionMatrixAddButton->setChecked(false);
            if (markedTransformationIdx>=0 && poseEstimatesType==2) {
                const pcl::PosesFromMatches::PoseEstimate& poseEstimate = poseEstimates[markedTransformationIdx];
                int scanIdx1 = poseEstimate.correspondence_indices[0],
                    scanIdx2 = poseEstimate.correspondence_indices[1];
                int noOfScans = scanDatabase.size();
                confusionMatrixScores[scanIdx2*noOfScans + scanIdx1] = confusionMatrixScores[scanIdx1*noOfScans + scanIdx2] = 1.0f;
                Eigen::Isometry3f transformation;
                transformation.matrix() = poseEstimate.transformation.matrix();
                confusionMatrixTransformations[scanIdx2*noOfScans + scanIdx1] = transformation;
                confusionMatrixTransformations[scanIdx1*noOfScans + scanIdx2] = transformation.inverse();
                confusionMatrixWidget.setRealImage(&confusionMatrixScores[0], noOfScans, noOfScans, true, 0.0f, 1.0f);
            }

            if (markedTransformationIdx >= 0) {
                const pcl::PosesFromMatches::PoseEstimate& poseEstimate = poseEstimates[markedTransformationIdx];
                int scanIdx1 = poseEstimate.correspondence_indices[0],
                    scanIdx2 = poseEstimate.correspondence_indices[1];
                std::cout << PVARC(scanIdx1)<<PVARN(scanIdx2);
            }
        }

        // The "Remove from matrix"-button was pressed
        if (window.confusionMatrixRemoveButton->isChecked()) {
            window.confusionMatrixRemoveButton->setChecked(false);
            const pcl::PosesFromMatches::PoseEstimate& poseEstimate = poseEstimates[markedTransformationIdx];
            int scanIdx1 = poseEstimate.correspondence_indices[0],
                scanIdx2 = poseEstimate.correspondence_indices[1];
            int noOfScans = scanDatabase.size();
            int confusionMatrixIdx = scanIdx2*noOfScans+scanIdx1;
            confusionMatrixScores[confusionMatrixIdx] = 0.0f;
            confusionMatrixWidget.setRealImage(&confusionMatrixScores[0], noOfScans, noOfScans, true, 0.0f, 1.0f);
        }


        // A new scan was selected in list
        if (markedScanIdx != visScanDatabaseList.currentRow() && visScanDatabaseList.currentRow() < (int)scanDatabase.size()) {
            markedScanIdx = visScanDatabaseList.currentRow();

            const ScanDatabaseElement& scan = *scanDatabase[markedScanIdx];
            //cout << "Scan "<<markedScanIdx+1<<" has self similarity "<<scan.selfSimilarity<<".\n";

            //int noOfScans = scanDatabase.size();
            //if (confusionMatrixFastScores != NULL) {
            //std::multimap<float, int> orderedScans;
            //for (int i=0; i<noOfScans; ++i)
            //orderedScans.insert(std::make_pair(confusionMatrixFastScores[markedScanIdx*noOfScans+i],i));
            //for (std::multimap<float, int>::const_iterator it=orderedScans.begin(); it!=orderedScans.end(); ++it)
            //cout << it->second+1 << " ";
            //cout << "\n";
            //}

            if (!scan.narfsForCompleteScan.empty()) {
                pcl::Narf& narf = *scan.narfsForCompleteScan[0];
                static ImageWidget* surfacePatchWidget = new ImageWidget;
                surfacePatchWidget->useFastScaling = true;
                surfacePatchWidget->showRealImagesInGrayScales = true;
                surfacePatchWidget->setRealImage(narf.getSurfacePatch(), narf.getSurfacePatchPixelSize(), narf.getSurfacePatchPixelSize(), true, -10.0f, 10.0f);
                static ImageWidget* descriptorWidget = new ImageWidget;
                descriptorWidget->useFastScaling = true;
                descriptorWidget->showRealImagesInGrayScales = true;
                descriptorWidget->setRealImage(narf.getDescriptor(), narf.getDescriptorSize(), 1, true);
            }

            if (window.menu_View_SelectedScanInMap->isChecked()) {
                ScanDatabaseElement& scan = *scanDatabase[markedScanIdx];
                markedScan.points.clear();
                if (scan.knownPoses.find(showMap)==scan.knownPoses.end()) {
                    cerr << "\""<<showMap<<"\" pose is not available.\n";
                    continue;
                }
                Eigen::Isometry3f pose = scan.knownPoses[showMap];
                pcl::transformPointCloud(scan.rangeImage, markedScan, pose);
                mapViewer.show();
            }

            window.currentScanLabel->show();
            window.currentScanLabel->setText("Scan "+QString::number(markedScanIdx+1)+": Self-similarity "+QString::number(scan.selfSimilarity));

            needDraw = true;
        }

        if (!window.menu_View_SelectedScanInMap->isChecked())
            markedScan.points.clear();

        // Recalculate the SPModel dictionary
        if (window.menu_Database_RecalculateDictionary->isChecked()) {
            window.menu_Database_RecalculateDictionary->setChecked(false);

            dictionary.clear();

            int noOfScans = scanDatabase.size();

            window.statusFrame->show();
            window.progressBar->setMaximum(noOfScans);
            window.progressBar->setValue(0);
            window.statusLabel->setText("Starting to calculate and save SPModel...");
            application.processEvents();

            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                ScanDatabaseElement& scan = *scanDatabase[scanIdx];
                scan.loadDictionaryNARFs();
                NarfImport import(&scan.featuresForDictionary, &dictionary);
                import.compute();
                scan.resetDictionaryFeatures();

                window.progressBar->setValue(scanIdx);
                window.statusLabel->setText("Done adding scan "+QString::number(scanIdx+1)+" to dictionary.");
                application.processEvents();
            }

            window.progressBar->setValue(0);
            window.statusLabel->setText("Clustering...");
            application.processEvents(); application.processEvents();

            //std::string dictionaryFileName = scanDatabase.name+".spm";
            //ofstream file(dictionaryFileName.c_str());
            //dictionary.writeBinary(file);
            //file.close();

            SPMDictionarySelector selector;
            //selector.clusterDictionaryWithDescriptorDistance(dictionary, maxDescrDistForClustering);
            selector.clusterDictionaryWithkMeans(dictionary, dictionarySize, 1);

            dictionary.clearInstances();

            string rotInv = (useRotationInvariance ? "RotInv" : "");

            std::string dictionaryFileName = scanDatabase.databaseDirectory + "/../dictionary"+rotInv+".dat";
            std::ofstream dictionaryFile(dictionaryFileName.c_str());
            dictionary.writeBinary(dictionaryFile);
            dictionaryFile.close();

            window.statusFrame->hide();
        }

        // Update the scan histograms
        if (window.menu_Database_UpdateScanHistograms->isChecked()) {
            window.menu_Database_UpdateScanHistograms->setChecked(false);

            int noOfScans = scanDatabase.size();

            window.statusFrame->show();
            window.progressBar->setMaximum(noOfScans);
            window.progressBar->setValue(0);

            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                std::cout << "Scan "<<scanIdx<<"\n";
                ScanDatabaseElement& scan = *scanDatabase[scanIdx];
                scan.loadDictionaryNARFs();
                SPModel currentWords;
                NarfImport import(&scan.featuresForDictionary, &currentWords);
                import.compute();
                scan.dictionaryHistogram = currentWords.getWordHistogram(dictionary.dictionary());
                scan.resetDictionaryFeatures();
                scan.saveDictionaryHistogram();

                window.progressBar->setValue(scanIdx);
                window.statusLabel->setText("Done computing histogram of scan "+QString::number(scanIdx+1)+".");
                application.processEvents();
            }

            //vector<double> weights = dictionary.getWordHistogram(dictionary.dictionary());
            //std::ofstream file("histograms.dat");
            //for (size_t histogramIdx=0; histogramIdx<weights.size(); ++histogramIdx)
            //file << weights[histogramIdx] << (histogramIdx==weights.size()-1 ? "\n" : " ");
            //for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx)
            //for (size_t histogramIdx=0; histogramIdx<weights.size(); ++histogramIdx)
            //file << histograms[scanIdx][histogramIdx] << (histogramIdx==weights.size()-1 ? "\n" : " ");
            //file.close();

            window.statusFrame->hide();
        }

        // Get the confusion matrix using the SPModel
        if (window.menu_Database_GetConfusionMatrixFromSPModel->isChecked()) {
            window.menu_Database_GetConfusionMatrixFromSPModel->setChecked(false);

            int noOfScans = scanDatabase.size();

            window.statusFrame->show();
            window.progressBar->setMaximum(noOfScans);
            window.progressBar->setValue(0);

            window.statusLabel->setText("Calculating confusion matrix");
            int confusionMatrizSize = pow(noOfScans, 2);
            confusionMatrixFastScores.clear();
            confusionMatrixFastScores.resize(confusionMatrizSize, 0.0f);
            confusionMatrixFastSource = CMFS_BAG_OF_WORDS;
            confusionMatrixFastTransformations.clear();

            float maxDistance = 0.0f;
            for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
                const ScanDatabaseElement& scan1 = *scanDatabase[scanIdx1];
                for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
                    const ScanDatabaseElement& scan2 = *scanDatabase[scanIdx2];
                    float& distance = confusionMatrixFastScores[scanIdx2*noOfScans+scanIdx1];
                    distance = scan1.getHistogramDistance(scan2.dictionaryHistogram);
                    maxDistance = std::max(distance, maxDistance);
                }
            }
            for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
                for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
                    float& score = confusionMatrixFastScores[scanIdx2*noOfScans+scanIdx1];
                    score = 1.0f - score/maxDistance;
                }
            }
            confusionMatrixFastWidget.setRealImage(&confusionMatrixFastScores[0], noOfScans, noOfScans, true, 0.0f, INFINITY);
            confusionMatrixFastWidget.setWindowTitle("Confusion matrix from BoW approach");

            window.statusFrame->hide();
        }

        // Get the confusion matrix using the global features
        if (window.menu_Database_GetConfusionMatrixFromGlobalFeatures->isChecked()) {
            window.menu_Database_GetConfusionMatrixFromGlobalFeatures->setChecked(false);

            int noOfScans = scanDatabase.size();

            window.statusFrame->show();
            window.progressBar->setMaximum(noOfScans);
            window.progressBar->setValue(0);
            window.statusLabel->setText("Extracting global features for each scan");
            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                //cout << "Extracting global feature for scan "<<scanIdx+1<<".\n";
                ScanDatabaseElement& scan = *scanDatabase[scanIdx];
                scan.extractGlobalFeatures(maximumRangeInDataset);
                window.progressBar->setValue(scanIdx+1);
                DO_EVERY(0.1, application.processEvents(););
            }

            //window.progressBar->setValue(0);
            //window.statusLabel->setText("Creating kd-trees");
            //for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
            //scanDatabase[scanIdx]->updateKdTree();
            //window.progressBar->setValue(scanIdx+1);
            //DO_EVERY(0.1, application.processEvents(););
            //}

            window.progressBar->setValue(0);
            window.statusLabel->setText("Calculating descriptor distances");
            int confusionMatrizSize = pow(noOfScans, 2);
            confusionMatrixFastScores.clear();
            confusionMatrixFastScores.resize(confusionMatrizSize, 0.0f);
            confusionMatrixFastSource = CMFS_GLOBAL_FEATURES;
            confusionMatrixFastTransformations.clear();
            confusionMatrixFastTransformations.resize(confusionMatrizSize, Eigen::Isometry3f::Identity());

            pcl::PosesFromMatches::PoseEstimatesVector tmpPoseEstimates;
            tmpPoseEstimates.resize(1);
            float maxDistance = 0.0f;

            for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
                const ScanDatabaseElement& scan1 = *scanDatabase[scanIdx1];
                cout << "Calculating descriptor distances for row "<<scanIdx1+1<<".\n";
                for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
                    const ScanDatabaseElement& scan2 = *scanDatabase[scanIdx2];
                    float& distance = confusionMatrixFastScores[scanIdx2*noOfScans+scanIdx1];
                    distance = 1.0f;
                    int bestIdx1=-1, bestIdx2=-1;
                    for (int narfIdx1=0; narfIdx1<=0; ++narfIdx1) {
                        for (int narfIdx2=0; narfIdx2<int(scan2.narfsForCompleteScan.size()); ++narfIdx2) {
                            if (scan1.narfsForCompleteScan[narfIdx1]->getDescriptorSize()>0 &&
                                    scan1.narfsForCompleteScan[narfIdx1]->getDescriptorSize() == scan2.narfsForCompleteScan[narfIdx2]->getDescriptorSize())
                            {
                                float descrDist = scan1.narfsForCompleteScan[narfIdx1]->getDescriptorDistance(*scan2.narfsForCompleteScan[narfIdx2]);
                                if (descrDist >= distance)
                                    continue;
                                distance = descrDist;
                                bestIdx1 = narfIdx1;
                                bestIdx2 = narfIdx2;
                            }
                        }
                    }
                    Eigen::Isometry3f& transformation = confusionMatrixFastTransformations[scanIdx2*noOfScans+scanIdx1];
                    transformation.matrix() = (scan2.narfsForCompleteScan[bestIdx2]->getTransformation().inverse(Eigen::Isometry) *
                            scan1.narfsForCompleteScan[bestIdx1]->getTransformation()).matrix();

                    //tmpPoseEstimates[0].transformation = transformation;
                    //rangeImageMatching.parameters.icpNumIterations = 10;
                    //rangeImageMatching.parameters.icpPixelSearchRadius = 10;
                    //rangeImageMatching.parameters.maxValidationPointError = 5.0f;
                    //rangeImageMatching.doICP(scan2, scan1, tmpPoseEstimates, true, 20, 10.0f, 1.0f);
                    //rangeImageMatching.doICP(scan2, scan1, tmpPoseEstimates, true);
                    //cout << PVARN(transformation.matrix())<<PVARN(tmpPoseEstimates[0].transformation.matrix());
                    //transformation = tmpPoseEstimates[0].transformation;

                    //cout << PVARC(scanIdx1)<<PVARC(scanIdx2)<<PVARN(distance);
                    maxDistance = std::max(distance, maxDistance);
                }
                //cout << PVARN(scan1.narfsForCompleteScan.size());
                window.progressBar->setValue(scanIdx1+1);
                DO_EVERY(0.1, application.processEvents(););
            }
            window.progressBar->setValue(0);
            window.statusLabel->setText("Setting confusion matrix values");
            for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
                cout << "Calculating scores for row "<<scanIdx1+1<<".\n";
                for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
                    float& score = confusionMatrixFastScores[scanIdx2*noOfScans+scanIdx1];
                    score = 1.0f - score/maxDistance;
                }
                window.progressBar->setValue(scanIdx1+1);
            }
            confusionMatrixFastWidget.setRealImage(&confusionMatrixFastScores[0], noOfScans, noOfScans, true);
            confusionMatrixFastWidget.setWindowTitle("Confusion matrix from global feature approach");

            window.statusFrame->hide();
        }

        // Get the ground truth confusion matrix
        if (window.menu_Database_ShowGroundTruthConfusionMatrix->isChecked()) {
            window.menu_Database_ShowGroundTruthConfusionMatrix->setChecked(false);
            cout << "Showing ground truth cofusion matrix.\n";

            float maxPointError = rangeImageMatching.parameters.maxValidationPointError;

            int noOfScans = scanDatabase.size();
            float* scanOverlapValues = new float[noOfScans*noOfScans];
            int noOfDoneScans = 0;

            window.statusFrame->show();
            window.progressBar->setMaximum(noOfScans);
            window.progressBar->setValue(0);
            window.statusLabel->setText("Calculating ground truth confusion matrix");
            application.processEvents();

#     pragma omp parallel for num_threads(maxNoOfThreads) default(shared) schedule(dynamic, 1)
            for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
                ScanDatabaseElement& scan1 = *scanDatabase[scanIdx1];
                const Eigen::Isometry3f& truePose1 = scan1.knownPoses["SLAM"];
                for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
                    ScanDatabaseElement& scan2 = *scanDatabase[scanIdx2];
                    const Eigen::Isometry3f& truePose2 = scan2.knownPoses["SLAM"];
                    Eigen::Isometry3f truePoseDiff = truePose1.inverse(Eigen::Isometry)*truePose2;

                    float& overlap = scanOverlapValues[scanIdx1*noOfScans+scanIdx2];
                    //overlap = std::min(rangeImageMatching.getRangeImageComparisonScore(scan1, scan2, truePoseDiff, maxPointError, false),
                    //rangeImageMatching.getRangeImageComparisonScore(scan2, scan1, truePoseDiff.inverse(Eigen::Isometry), maxPointError, false));
                    overlap = std::min(rangeImageMatching.getValidationPointsScore(scan1, scan2, truePoseDiff, 0.0f, maxPointError, false, -1),
                            rangeImageMatching.getValidationPointsScore(scan2, scan1, truePoseDiff.inverse(Eigen::Isometry),
                                0.0f, maxPointError, false, -1));
                }
#       pragma omp critical
                ++noOfDoneScans;
                if(omp_get_thread_num() == 0)
                {
                    window.progressBar->setValue(noOfDoneScans);
                    application.processEvents();
                }
            }

            window.progressBar->setValue(0);
            window.statusLabel->setText("Writing ground truth confusion matrix to disc");
            application.processEvents();

            // Write confusion matrix to file
            std::string confusionMatrixFileName = scanDatabase.databaseDirectory+"/"+statisticsDirectory+"/confusionMatrixGroundTruth_"+scanDatabase.name+".dat";
            std::ofstream confusionMatrixFile(confusionMatrixFileName.c_str());
            for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
                for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
                    float score = scanOverlapValues[scanIdx2*noOfScans + scanIdx1];
                    confusionMatrixFile << scanIdx1+1<<" "<<scanIdx2+1<<" "<<score<<"\n";
                }
                window.progressBar->setValue(noOfDoneScans);
                application.processEvents();
            }
            confusionMatrixFile.close();

            confusionMatrixGroundTruthWidget.setRealImage(scanOverlapValues, noOfScans, noOfScans, true, 0.0f, 1.0f);
            delete[] scanOverlapValues;

            window.statusFrame->hide();
        }

        bool updateMap = false;
        // Show Odometry map
        if (window.menu_View_ShowOdometryMap->isChecked()) {
            window.menu_View_ShowOdometryMap->setChecked(false);
            showMap = "Odometry";
            updateMap = true;
        }
        // Show incremental Scan matching map
        if (window.menu_View_ShowIncrementalScanMatchingMap->isChecked()) {
            window.menu_View_ShowIncrementalScanMatchingMap->setChecked(false);
            showMap = "IncrementalScanMatching";
            updateMap = true;
        }
        // Show SLAM map
        if (window.menu_View_ShowSlamMap->isChecked()) {
            window.menu_View_ShowSlamMap->setChecked(false);
            showMap = "SLAM";
            updateMap = true;
        }
        // Show map
        if (updateMap) {
            updateMap = false;

            int noOfScans = scanDatabase.size();

            window.statusFrame->show();
            window.progressBar->setMaximum(noOfScans);
            window.progressBar->setValue(0);
            window.statusLabel->setText("Starting to recalculate data for all scans...");
            application.processEvents();

            mapViewer.show();
            trajectory.clear();
            mapCloud.points.clear();
            markedScanIdx = -1;
            markedScan.points.clear();
            mapCloud.cellSize = mapCellSize;

            visMapCloud.resetConnectingLines();
            Vis::MapCloud::CoordinateSystem mapCloudCoordinateSystem =
                (coordinateFrame==pcl::RangeImage::CAMERA_FRAME ? Vis::MapCloud::xToRight_yDownwards_zForward : Vis::MapCloud::xForward_yToLeft_zUpwards);
            visMapCloud.setCoordinateSystem(mapCloudCoordinateSystem);
            Eigen::Isometry3f moveToOrigin = Eigen::Isometry3f::Identity();
            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                //cout << "Adding scan "<<scanIdx+1<<" to map.\n";
                if (!mapViewer.isVisible()) {
                    break;
                }
                ScanDatabaseElement scan = *scanDatabase[scanIdx];
                if (scan.knownPoses.find(showMap)==scan.knownPoses.end()) {
                    std::cerr << "Scan "<<scanIdx<<" does not have a \""<<showMap<<"\" pose.\n";
                    continue;
                }

                Eigen::Isometry3f pose = scan.knownPoses[showMap];
                if (scanIdx==0)
                    moveToOrigin =  pose.inverse();

                //scanMatchedPose = scanMatchedPose * previousOdoPose.inverse() * odoPose;

                pose = moveToOrigin * pose;
                Eigen::Vector3f sensorPos = scan.rangeImage.getSensorPos();
                trajectory.push_back(pose);
                //scan.loadPointCloud();
                for (size_t pointIdx=0; pointIdx<scan.rangeImage.points.size(); ++pointIdx) {
                    if (!scan.rangeImage.isValid(pointIdx))
                        continue;
                    Eigen::Vector3f point = scan.rangeImage.getPoint(pointIdx).getVector3fMap();
                    if (window.menu_View_MapIn2D->isChecked() &&
                            ((coordinateFrame==pcl::RangeImage::CAMERA_FRAME && (point[1]+0.2<sensorPos[1] || point[1]-0.2>sensorPos[1])) ||
                             (coordinateFrame==pcl::RangeImage::LASER_FRAME  && (point[2]+0.2<sensorPos[2] || point[2]-0.2>sensorPos[2]))))
                        continue;
                    point = pose*point;
                    mapCloud.add(point[0], point[1], point[2]);
                }
                //visMapCloud.resetConnectingLines();
                if (window.menu_View_CurrentCorrespondences->isChecked() && !confusionMatrixScores.empty()) {
                    for (int scanIdx2=0; scanIdx2<scanIdx; ++scanIdx2) {
                        int confusionMatrixIdx = scanIdx2*noOfScans+scanIdx;
                        float confusionMatrixScore = confusionMatrixScores[confusionMatrixIdx];
                        if (confusionMatrixScore > minScoreToAddToGraph)
                            visMapCloud.addConnectingLine(scanIdx, scanIdx2);
                    }
                }

                // Visualize current scan
                markedScan.points.clear();
                pcl::transformPointCloud(scan.rangeImage, markedScan, pose);

                // Update Viewer
                mapViewer.updateGL();
                application.processEvents();

                if (window.menu_View_SaveMapScreenshots->isChecked()) {
                    stringstream fileNameSS;
                    fileNameSS << showMap<<"_map_" << setfill ('0') << setw (6) << scanIdx+1 << ".png";
                    mapViewer.savePng(fileNameSS.str().c_str());
                    cout << "Saved image of current map as \""<<fileNameSS.str()<<"\".\n";
                }

                window.progressBar->setValue(scanIdx);
                window.statusLabel->setText("Done with scan "+QString::number(scanIdx+1)+".");
                application.processEvents();
            }

            bool saveCloudToDisk = false;
            if (saveCloudToDisk) {
                pcl::PointCloud<pcl::PointXYZ> tmpCloud;
                for (std::map<int, std::map<int, std::set<int> > >::const_iterator itX=mapCloud.points.begin(); itX!=mapCloud.points.end(); ++itX) {
                    for (std::map<int, std::set<int> >::const_iterator itY=itX->second.begin(); itY!=itX->second.end(); ++itY) {
                        for (std::set<int>::const_iterator itZ=itY->second.begin(); itZ!=itY->second.end(); ++itZ) {
                            pcl::PointXYZ p;
                            p.x = itX->first*mapCloud.cellSize;
                            p.y = itY->first*mapCloud.cellSize;
                            p.z = *itZ*mapCloud.cellSize;
                            tmpCloud.points.push_back(p);
                        }
                    }
                }
                tmpCloud.width = tmpCloud.points.size();
                tmpCloud.height = 1;
                pcl::io::savePCDFile("mergedCloud.pcd", tmpCloud, true);

            }
            window.statusFrame->hide();

        }
        visMapCloud.flattenMap  = window.menu_View_FlattenMap->isChecked();
        visMapCloud.showCloud   = window.menu_View_CloudInMap->isChecked();
        visMapCloud.showNumbers = window.menu_View_ScanNumbersOnTrajectory->isChecked();

        // Do incremental scan matching
        if (window.menu_Database_DoIncrementalScanMatching->isChecked()) {
            window.menu_Database_DoIncrementalScanMatching->setChecked(false);

            int noOfScans = scanDatabase.size();

            window.statusFrame->show();
            window.progressBar->setMaximum(noOfScans);
            window.progressBar->setValue(0);
            window.statusLabel->setText("Starting to scan match all scans...");
            application.processEvents();

            //mapViewer.show();
            trajectory.clear();
            mapCloud.points.clear();
            markedScan.points.clear();

            Eigen::Isometry3f scanMatchedPose = Eigen::Isometry3f::Identity();
            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                //if (!mapViewer.isVisible()) {
                //break;
                //}
                ScanDatabaseElement& scan = *scanDatabase[scanIdx];
                Eigen::Isometry3f odoPose = Eigen::Isometry3f::Identity();
                if (scan.knownPoses.find("Odometry")!=scan.knownPoses.end())
                    odoPose = scan.knownPoses["Odometry"];
                if (scanIdx > 0) {
                    ScanDatabaseElement previousScan = *scanDatabase[scanIdx-1];
                    Eigen::Isometry3f previousOdoPose = Eigen::Isometry3f::Identity();
                    if (previousScan.knownPoses.find("Odometry")!=previousScan.knownPoses.end())
                        previousOdoPose = previousScan.knownPoses["Odometry"];

                    //scan.loadPointCloud();
                    //scan.createRangeImage(angularResolution, noiseLevel, coordinateFrame, maximumRange);

                    Eigen::Isometry3f odoPoseEstimate, icpPoseEstimate, ourPoseEstimate, bestPoseEstimate;
                    float odoPoseScore, icpPoseScore, ourPoseScore, bestPoseScore;
                    odoPoseEstimate = odoPose.inverse()*previousOdoPose;
                    odoPoseScore = rangeImageMatching.getValidationPointsScore(scan, previousScan, odoPoseEstimate, 0.0f);
                    bestPoseEstimate = odoPoseEstimate;
                    bestPoseScore = odoPoseScore;
                    int source = 0;

                    if (true || bestPoseScore < minScoreToAddToGraph) {
                        icpPoseEstimate = rangeImageMatching.doICP(scan, previousScan.validationPoints, odoPoseEstimate);
                        icpPoseScore = rangeImageMatching.getValidationPointsScore(scan, previousScan, icpPoseEstimate, 0.0f);
                        if (icpPoseScore > bestPoseScore) {
                            bestPoseEstimate = icpPoseEstimate;
                            bestPoseScore = icpPoseScore;
                            source = 1;
                        }

                        if (true || bestPoseScore < minScoreToAddToGraph) {
                            pcl::PosesFromMatches::PoseEstimate tmp;
                            tmp = rangeImageMatching.getBestPoseEstimate(scan, previousScan);
                            ourPoseEstimate.matrix() = tmp.transformation.matrix();
                            ourPoseScore = tmp.score;
                            if (ourPoseScore > bestPoseScore)
                            {
                                //float movedDistanceOdo = odoPoseEstimate.transformation.translation().norm(),
                                //movedDistanceOur = ourPoseEstimate.transformation.translation().norm();
                                //if (movedDistanceOur < 1.2f*movedDistanceOdo && movedDistanceOur > 0.8f*movedDistanceOdo)
                                {
                                    //cout  << PVARC(movedDistanceOur)<<PVARN(movedDistanceOdo);
                                    bestPoseEstimate = ourPoseEstimate;
                                    bestPoseScore = ourPoseScore;
                                    source = 2;
                                }
                            }
                        }
                    }

                    //cout <<": " << PVARC(odoPoseEstimate.score) << PVARC(icpPoseEstimate.score) << PVARN(ourPoseEstimate.score);

                    cout << (source==0 ? "Odo" : (source==1 ? "ICP" : "Our")) << " "<<std::flush;

                    scanMatchedPose = scanMatchedPose * bestPoseEstimate.inverse(Eigen::Isometry);
                }
                scanDatabase[scanIdx]->knownPoses["IncrementalScanMatching"] = scanMatchedPose;
                //for (size_t pointIdx=0; pointIdx<scan.pointCloud.points.size(); ++pointIdx) {
                //mapCloud.points.push_back(pcl::PointXYZ());
                ////mapCloud.points.back().getVector3fMap() = scan.pointCloud.points[pointIdx].getVector3fMap();
                //mapCloud.points.back().getVector3fMap() = scanMatchedPose * scan.pointCloud.points[pointIdx].getVector3fMap();
                //}
                //mapViewer.updateGL();  application.processEvents();

                window.progressBar->setValue(scanIdx);
                window.statusLabel->setText("Done with scan "+QString::number(scanIdx+1)+".");
                application.processEvents();
            }
            cout << "\n";
            window.statusFrame->hide();
        }

        /////////////SEB////////////////////////////////////////////////////////
        if(window.menu_Seb_runAll->isChecked()) {
            window.menu_Seb_runAll->setChecked(false);
            std::cout << "Running the following actions:" << std::endl;
            std::cout << "Database --> Recalculate and save range images" << std::endl;
            recalculateAndSaveRangeImages(application, maxNoOfThreads, maximumRange);
            std::cout << "Database --> Create dataset statistics" << std::endl;
            createDatasetStatistics(keyPointDetector);
            std::cout << "Database --> Recalculate and save all" << std::endl;
            recalculateAndSaveAll(application, keyPointDetector, maxNoOfThreads, maximumRange);
            std::cout << "Database --> Calculate and save confusion matrix" << std::endl;
            calculateConfusionMatrix(application, confusionMatrixWidget, neededTimesPerScan, maxNoOfThreads, maximumRange, useSlamHack);
            std::cout << "Done !" << std::endl;
        }

        if(window.menu_Seb_runOptional->isChecked()) {
            window.menu_Seb_runOptional->setChecked(false);
            std::cout << "The optional steps are:" << std::endl;
            std::cout << "Database --> DoIncrementalScanMatching" << std::endl;
            std::cout << "Database --> CreateAndOptimizeGraph" << std::endl;
            std::cout << "View --> ShowSlamMap" << std::endl;
            std::cout << "Database --> SaveKnownPoses" << std::endl;
            std::cout << "Database --> CreateConfusionMatrixStatistics" << std::endl;
        }

        if(window.menu_Seb_devTest->isChecked()) {
            std::cout << "Running a developpement test..." << std::endl;
            window.menu_Seb_devTest->setChecked(false);

            results.plotStuff();
            std::cout << "Test done !" << std::endl;
        }
        /////////////SEB////////////////////////////////////////////////////////

        // Save know poses
        if (window.menu_Database_SaveKnownPoses->isChecked()) {
            window.menu_Database_SaveKnownPoses->setChecked(false);
            cout << "Saving know poses...\n";
            int noOfScans = scanDatabase.size();
            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                const ScanDatabaseElement& scan = *scanDatabase[scanIdx];
                scan.saveInfoFile();
            }
            cout << "Done.\n";
        }

        // Save know poses
        if (window.menu_Database_SaveForPublishing->isChecked()) {
            window.menu_Database_SaveForPublishing->setChecked(false);
            cout << "Saving the dataset...\n";

            string datasetsForPublishingDirectoryName = "datasetsForPublishing",
                   directory = datasetsForPublishingDirectoryName+"/"+scanDatabase.name;

            createDirectory(datasetsForPublishingDirectoryName.c_str());
            createDirectory(directory.c_str());

            string dirName = datasetsForPublishingDirectoryName + "/" + scanDatabase.name;

            int noOfScans = scanDatabase.size();
            std::ofstream readmeFile((directory+"/README.txt").c_str());
            if (!readmeFile) {
                cerr << "Error writing to disk.\n";
                continue;
            }
            readmeFile << "Dataset: \""<<scanDatabase.name<<"\"\n"
                << "\nGeneral description:\n"
                <<   "--------------------\n"
                << scanDatabase.configFileParams["description"]<<"\n"
                << "It contains "<<noOfScans<<" 3D scans with "<<scanDatabase.configFileParams["averageNoOfPoints"]<<" "
                << "points per scan on average. Each scan has an approximate usable angular resolution of "
                << scanDatabase.configFileParams["angularResolution"]<<"deg. "
                << "The length of the trajectory is " << scanDatabase.configFileParams["trajectoryLength"]<<"m.\n"
                << "\nFile informations:\n"
                <<   "------------------\n"
                << "This directory contains 3 files per scan. The 3D points of the scans themselves are saved in scan_xxx.pcd. "
                << "*.pcd is the file format of the Point Cloud Library (PCL - see http://pointclouds.org). "
                << "The files are in ASCI, so the library is not necessarily needed to parse the file. "
                << "After the header, the points are given with x, y, z, sensorPosX, sensorPosY, sensorPosZ per line. "
                << "The sensor position is the point in space, from which the point was originally observed.\n"
                << "In the second file (scan_xxx_far_ranges.pcd) are the points that returned maximum range readings, "
                << "which is typically a good indicator for free space.\n"
                << "The third file (scan_xxx_info.dat) contains odometry and SLAM poses (x,y,z,roll,pitch,yaw in meters and radians) "
                << "for the scans.\n"
                << "\nLicense:\n"
                <<   "--------\n"
                << "This dataset is made available under the Creative Commons Attribution Licence 3.0: "
                << "the licensor permits others to copy, distribute, display, and perform the work. "
                << "In return, licensees must give the original author credit.\n"
                << "See http://creativecommons.org/licenses/by/3.0\n"
                << "By way of attribution, please include an acknowledgement and/or citation for any results published using this dataset.\n";
            readmeFile.close();
            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                cout << scanIdx << " "<<std::flush;
                ScanDatabaseElement& scan = *scanDatabase[scanIdx];
                copyFile(scan.pointCloudFileName.c_str(), (directory+"/"+getBasename(scan.pointCloudFileName)).c_str());
                string farRangesFileName = getPureFilename(scan.pointCloudFileName)+"_far_ranges.pcd";
                copyFile(farRangesFileName.c_str(), (directory+"/"+getBasename(farRangesFileName)).c_str());
                string infoFileName = directory+"/"+getPureFilename(getBasename(scan.pointCloudFileName))+"_info.dat";
                std::ofstream infoFile(infoFileName.c_str());
                if (!infoFile) {
                    cerr << "Could not write to file \""<<infoFileName<<"\"\n";
                }
                infoFile << "# Known 6DOF positions for this scan (format: x y z roll pitch yaw)\n";
                float x, y, z, roll, pitch, yaw;
                pcl::getTranslationAndEulerAngles(scan.knownPoses["Odometry"], x, y, z, roll, pitch, yaw);
                infoFile << "Odometry: "<<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
                pcl::getTranslationAndEulerAngles(scan.knownPoses["SLAM"], x, y, z, roll, pitch, yaw);
                infoFile << "SLAM: "<<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
                infoFile.close();
            }
            cout << "\n\nDone.\n";
            cout << "Saved in folder \""<<directory<<"\"\n";
        }

        // ==========
        // Update GUI
        // ==========
        if (needDraw) {
            viewer.updateGL();
            viewer2.updateGL();
            mapViewer.updateGL();
        }
    }

    //delete[] confusionMatrixImage;
}

template <typename T>
std::string toString(const T& t) {
    std::stringstream tmpSS;
    tmpSS << t;
    return tmpSS.str();
}

void printUsage (const char *progName) {
    cout << "\n\nUsage: " << progName << " [options] <database folder>\n\n"
        << "Options:\n" << "-------------------------------------------\n"
        << "-h         this help\n" << "\n\n";
}

int getMatrixIndex(int scan1Index, int scan2Index) {
    return scan2Index*scanDatabase.size()+scan1Index;
}

std::pair<int, int> getScanIndices(int matrixIndex) {
    int first = matrixIndex % scanDatabase.size();
    int second = static_cast<int>(matrixIndex/scanDatabase.size());

    std::pair<int, int> scanPair(first, second);
    return scanPair;
}

void recalculateAndSaveRangeImages(
        QApplication &application,
        const int &maxNoOfThreads,
        const float &maximumRange) {
    window.menu_Database_RecalculateAndSaveRangeImages->setChecked(false);

    int noOfScans = scanDatabase.size();

    window.statusFrame->show();
    window.progressBar->setMaximum(noOfScans);
    window.progressBar->setValue(0);
    window.statusLabel->setText("Starting to recalculate range images for all scans...");
    application.processEvents();

    int noOfDoneScans = 0;
#     pragma omp parallel for num_threads(maxNoOfThreads) default(shared) schedule(dynamic, 1)
    for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
        ScanDatabaseElement& scan = *scanDatabase[scanIdx];

        // Load point cloud from disc - if not already existing
        if (!scan.pointCloudFileName.empty() && scan.pointCloud.points.empty()) {
            scan.loadPointCloud();
        }
        scan.createRangeImage(angularResolution, noiseLevel, coordinateFrame, maximumRange);

#       pragma omp critical
        ++noOfDoneScans;

        if(omp_get_thread_num() == 0) {
            window.progressBar->setValue(noOfDoneScans);
            window.statusLabel->setText("Done recalculating "+QString::number(noOfDoneScans)+" scans.");
            application.processEvents();
        }
    }
    window.statusFrame->hide();
    window.menu_Database_SaveRangeImages->setChecked(true);
}

void createDatasetStatistics(MyPcl::NarfssKeypoint &keyPointDetector) {
    window.menu_Database_CreateDatasetStatistics->setChecked(false);

    int noOfScans = scanDatabase.size();
    float trajectoryLength = 0.0f;
    averageRangeMeasurement = 0.0f;
    maximumRangeInDataset = 0.0f;
    int rangeImagePointCounter=0;
    int averageNoOfPoints=0, averageNoOfFarRanges=0;
    for (int scanIdx=1; scanIdx<noOfScans; ++scanIdx) {
        cout << "Analysing scan "<<scanIdx+1<<".\n";
        ScanDatabaseElement scan = *scanDatabase[scanIdx];
        bool trueMovementIsValid = true;
        Eigen::Isometry3f truePose = Eigen::Isometry3f::Identity();
        if (scan.knownPoses.find("SLAM")!=scan.knownPoses.end())
            truePose = scan.knownPoses["SLAM"];
        else
            trueMovementIsValid=false;
        ScanDatabaseElement& previousScan = *scanDatabase[scanIdx-1];
        Eigen::Isometry3f previousTruePose = Eigen::Isometry3f::Identity();
        if (previousScan.knownPoses.find("SLAM")!=previousScan.knownPoses.end())
            previousTruePose = previousScan.knownPoses["SLAM"];
        else
            trueMovementIsValid=false;
        if (!trueMovementIsValid)
            trajectoryLength = -1;
        Eigen::Isometry3f movement = previousTruePose.inverse(Eigen::Isometry)*truePose;
        float movedDistanceSinceLastScan = movement.translation().norm();
        if (trajectoryLength >= 0.0f)
            trajectoryLength += movedDistanceSinceLastScan;
        for (size_t pointIdx=0; pointIdx<scan.rangeImage.points.size(); ++pointIdx) {
            if (!scan.rangeImage.isValid(pointIdx))
                continue;
            const pcl::PointWithRange& point = scan.rangeImage.getPoint(pointIdx);
            averageRangeMeasurement += point.range;
            maximumRangeInDataset = std::max(maximumRangeInDataset, point.range);
            ++rangeImagePointCounter;
        }
        //scan.loadPointCloud();
        averageNoOfPoints += scan.pointCloud.points.size();
        averageNoOfFarRanges += scan.pointCloudFarRanges.points.size();
    }
    averageDistanceBetweenConsecutiveScans = -1;
    if (trajectoryLength >= 0.0f) {
        averageDistanceBetweenConsecutiveScans = trajectoryLength / (noOfScans-1);
        trajectoryLength = round(trajectoryLength*10)/10;
    }

    cout << PVARN(trajectoryLength);
    std::stringstream tmp;  tmp<<trajectoryLength;
    scanDatabase.configFileParams["trajectoryLength"] = tmp.str();

    averageDistanceBetweenConsecutiveScans = round(averageDistanceBetweenConsecutiveScans*100)/100;
    cout << PVARN(averageDistanceBetweenConsecutiveScans);
    tmp.str("");  tmp<<averageDistanceBetweenConsecutiveScans;
    scanDatabase.configFileParams["averageDistanceBetweenConsecutiveScans"] = tmp.str();

    if (rangeImagePointCounter==0)
        averageRangeMeasurement=-1;
    else {
        averageRangeMeasurement /= rangeImagePointCounter;
        averageRangeMeasurement = round(100.0f*averageRangeMeasurement)/100.0f;

        supportSize = round(100.0f*averageRangeMeasurement/5.0f)/100.0f;
        supportSizeDictionary = round(100.0f*averageRangeMeasurement/15.0f)/100.0f;
        float maxValidationPointError = round(100.0f*averageRangeMeasurement/20.0f)/100.0f;
        //noiseLevel = maxValidationPointError;

        keyPointDetector.getParameters().support_size = supportSize;
        rangeImageMatching.parameters.minDistanceBetweenMatches = 0.5f*supportSize;
        rangeImageMatching.parameters.maxValidationPointError = maxValidationPointError;

        tmp.str("");  tmp<<supportSize;
        scanDatabase.configFileParams["supportSize"] = tmp.str();
        tmp.str("");  tmp<<supportSizeDictionary;
        scanDatabase.configFileParams["supportSizeDictionary"] = tmp.str();
        tmp.str("");  tmp<<maxValidationPointError;
        scanDatabase.configFileParams["maxValidationPointError"] = tmp.str();
        //tmp.str("");  tmp<<noiseLevel;
        //scanDatabase.configFileParams["noiseLevel"] = tmp.str();

        cout << PVAR(averageRangeMeasurement)<<" => " << PVARC(supportSize)<<PVARC(supportSizeDictionary)
            << PVARC(maxValidationPointError);
    }
    tmp.str("");  tmp<<averageRangeMeasurement;
    scanDatabase.configFileParams["averageRangeMeasurement"] = tmp.str();

    maximumRangeInDataset = round(maximumRangeInDataset*10)/10;
    cout << PVARN(maximumRangeInDataset);
    tmp.str("");  tmp<<maximumRangeInDataset;
    scanDatabase.configFileParams["maximumRangeInDataset"] = tmp.str();

    averageNoOfPoints /= noOfScans;
    averageNoOfFarRanges /= noOfScans;
    //cout << PVARC(averageNoOfPoints) << PVARN(averageNoOfFarRanges);
    //tmp.str("");  tmp<<averageNoOfPoints;
    //scanDatabase.configFileParams["averageNoOfPoints"] = tmp.str();
    //tmp.str("");  tmp<<averageNoOfFarRanges;
    //scanDatabase.configFileParams["averageNoOfFarRanges"] = tmp.str();

    scanDatabase.saveConfigFile();
}

void recalculateAndSaveAll(QApplication &application, MyPcl::NarfssKeypoint &keyPointDetector, const int &maxNoOfThreads, const float &maximumRange) {
    window.menu_Database_RecalculateAndSaveAll->setChecked(false);

    int noOfScans = scanDatabase.size();

    window.statusFrame->show();
    window.progressBar->setMaximum(noOfScans);
    window.progressBar->setValue(0);
    window.statusLabel->setText("Starting to recalculate data for all scans...");
    application.processEvents();

    double timeForScanCalculations=0.0, timeForLoadingScans=0.0, startTime=get_time();
    int noOfDoneScans = 0;

    bool done = false;
    omp_set_nested(1);
#     pragma omp parallel num_threads(2) default(shared)
    {
        if(omp_get_thread_num() == 0) {
            int lastNoOfDoneScans = -1;
            while (!done) {
                int currentNoOfDoneScans = noOfDoneScans;
                if (noOfDoneScans != lastNoOfDoneScans) {
                    double eta = (currentNoOfDoneScans>0 ? (noOfScans-currentNoOfDoneScans) * (get_time()-startTime)/currentNoOfDoneScans : -1.0),
                           etaHours = floor(eta/3600), etaMinutes = floor((eta-3600*etaHours)/60), etaSeconds = round(eta - 3600*etaHours - 60*etaMinutes);
                    QString etaString = (eta>0 ? "  ETA: "+QString::number(etaHours)+"h "+QString::number(etaMinutes)+"m "+QString::number(etaSeconds)+"s" : "");
                    window.progressBar->setValue(currentNoOfDoneScans);
                    window.statusLabel->setText("Recalculating database - Done scans: "+QString::number(currentNoOfDoneScans)+"."+etaString);
                    application.processEvents();
                    lastNoOfDoneScans = currentNoOfDoneScans;
                }
                usleep(100000);
            }
        }
        else {
#         pragma omp parallel for num_threads(maxNoOfThreads) default(shared) schedule(dynamic, 1)
            //keyPointDetector.getParameters().max_no_of_threads = maxNoOfThreads;
            //pcl::Narf::max_no_of_threads = maxNoOfThreads;
            for (int scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
                ScanDatabaseElement& scan = *scanDatabase[scanIdx];

                // Load point cloud from disc - if not already existing
                double timeForLoadingCurrentScan = -get_time();
                if (!scan.pointCloudFileName.empty() && scan.pointCloud.points.empty()) {
                    scan.loadPointCloud();
                }
                //else
                //{
                //if (!scan.pointCloud.points.empty())
                //std::cerr << "Point cloud of scan "<<scanIdx<<" already exists.\n";
                //else
                //std::cout << "Scan "<<scanIdx<<" has empty file name.\n";
                //}
                timeForLoadingCurrentScan += get_time();

                //pcl::RangeImageBorderExtractor tmpBorderExtractor;
                //tmpBorderExtractor.getParameters() = borderExtractor.getParameters();
                //pcl::NarfKeypoint tmpKeyPointDetector(&tmpBorderExtractor);
                //tmpKeyPointDetector.getParameters() = keyPointDetector.getParameters();
                MyPcl::NarfssKeypoint tmpKeyPointDetector;   // Create temporary objects to be thread safe
                tmpKeyPointDetector.getParameters() = keyPointDetector.getParameters();

                double timeForCurrentScanCalculations = -get_time();
                scan.createRangeImage(angularResolution, noiseLevel, coordinateFrame, maximumRange);
                //scan.extractNARFs(supportSize, descriptorSize, useRotationInvariance, tmpKeyPointDetector);
                //scan.extractValidationPoints(noOfValidationPoints, tmpKeyPointDetector, minSurfaceChangeForValidationPoints);
                scan.extractNARFs(supportSize, descriptorSize, useRotationInvariance, tmpKeyPointDetector, true);
                scan.extractValidationPoints(noOfValidationPoints, tmpKeyPointDetector, true);
                timeForCurrentScanCalculations += get_time();
                //cout << "DONE with "<<scanIdx<<"\n";

                scan.resetClouds();

                //cout << PVARC(scanIdx)<<PVARC(timeForLoadingCurrentScan)<<PVARN(timeForCurrentScanCalculations);

#           pragma omp critical
                {
                    timeForLoadingScans += timeForLoadingCurrentScan;
                    timeForScanCalculations += timeForCurrentScanCalculations;
                    ++noOfDoneScans;
                    //cout << "Doing scan calculations for scan "<<scanIdx1+1<<" took "<<timeForCurrentScanCalculations<<"s.\n"
                    //<< "Matching against the database took "<<timeForMatchingCurrentScan<<"s.\n";
                }
            }
            done = true;
        }
    }
    omp_set_nested(0);

    keyPointDetector.getParameters().max_no_of_threads = 1;
    pcl::Narf::max_no_of_threads = 1;
    cout << "Recalculation for all scans took "<<timeForScanCalculations<<"s.\n";
    saveResult("timeForScanCalculationsPerScan", round(1000*timeForScanCalculations/scanDatabase.size()));

    window.statusFrame->hide();
    window.menu_Database_SaveRangeImages->setChecked(true);
    window.menu_Database_SaveFeaturesAndValidationPoints->setChecked(true);
    window.menu_Database_UpdateAndSaveSelfSimilarities->setChecked(true);
    //window.menu_Database_RecalculateDictionaryFeatures->setChecked(true);
    //window.menu_Database_UpdateScanHistograms->setChecked(true);

    //databaseChanged = true;
}

void calculateConfusionMatrix(QApplication &application, ImageWidget &confusionMatrixWidget, vector<vector<double> > &neededTimesPerScan, const int &maxNoOfThreads, const float &maximumRange, const bool &useSlamHack) {
    window.menu_Database_CalculateConfusionMatrix->setChecked(false);

    if (scanDatabase.empty()) {
        cerr << "Scan database is empty.\n";
        return;
    }

    int noOfScans = scanDatabase.size();

    int confusionMatrizSize = pow(noOfScans, 2);
    confusionMatrixScores.clear();
    confusionMatrixScores.resize(confusionMatrizSize, 0.0f);
    confusionMatrixTransformations.clear();
    confusionMatrixTransformations.resize(confusionMatrizSize, Eigen::Isometry3f::Identity());

    window.statusFrame->show();
    window.progressBar->setMaximum(noOfScans);
    window.progressBar->setValue(0);
    window.statusLabel->setText("Starting calculation of the confusion matrix"+QString(useSlamHack?" (SLAM hack)":"")+"...");
    application.processEvents();

    vector<int> scansToCompareWith;
    for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2)
        scansToCompareWith.push_back(scanIdx2);

    double timeForConfusionMatrixCalculation=0.0, timeForScanCalculations=0.0, startTime = get_time();
    int noOfDoneScans = 0;
    neededTimesPerScan.resize(noOfScans);
    bool interrupted = false;
    //#     pragma omp parallel for num_threads(maxNoOfThreads) default(shared) schedule(dynamic, 1) private(scansToCompareWith)
    for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
        if (window.stopButton->isChecked()) {
            window.stopButton->setChecked(false);
            interrupted = true;
            break;
        }
        ScanDatabaseElement scan = *scanDatabase[scanIdx1];
        double timeForCurrentScanCalculations = -get_time();
        if (scan.rangeImage.points.empty()) {
            if (scan.pointCloud.points.empty())
                scan.loadPointCloud();
            scan.createRangeImage(angularResolution, noiseLevel, coordinateFrame, maximumRange);
        }
        timeForCurrentScanCalculations += get_time();

        std::vector<pcl::PosesFromMatches::PoseEstimatesVector> poseEstimatesPerScan;
        poseEstimatesPerScan.resize(scanDatabase.size());

        vector<double>& neededTimes = neededTimesPerScan[scanIdx1];
        neededTimes.resize(noOfScans);

        double timeForMatchingCurrentScan = -get_time();

        if (timeLimitForDatabaseQuery>0) {
            if (window.menu_Config_useBowToSortScans->isChecked())
                scanDatabase.orderScansRegardingHistogramSimilarity(scan.dictionaryHistogram, noOfScans, scansToCompareWith);
            if (window.menu_Config_useGlobalFeaturesToSortScans->isChecked())
                scanDatabase.orderScansRegardingGlobalFeatureSimilarity(scan, noOfScans, scansToCompareWith);
        }

        //rangeImageMatching.getBestPoseEstimates(scan, poseEstimatesPerScan);
        //rangeImageMatching.parameters.maxNoOfThreadsPerScanComparison = maxNoOfThreads;
        double currentScanStartTime=get_time();

        bool stop = false;
#       pragma omp parallel for num_threads(maxNoOfThreads) default(shared) schedule(dynamic, 1)
        for (size_t scansToCompareWithIdx=0; scansToCompareWithIdx<scansToCompareWith.size(); ++scansToCompareWithIdx) {
            if (stop)
                continue;
            //cout << PVARN(scansToCompareWithIdx);
            int scanIdx2 = scansToCompareWith[scansToCompareWithIdx];

            if (scanIdx2==scanIdx1) {
                pcl::PosesFromMatches::PoseEstimate identityEstimate;
                identityEstimate.transformation = Eigen::Affine3f::Identity();
                identityEstimate.score = 1.0f;
                poseEstimatesPerScan[scanIdx2].push_back(identityEstimate);
                continue;
            }

            if (useSlamHack) {
                if (scanDatabase[scanIdx1]->knownPoses.find("SLAM")!=scanDatabase[scanIdx1]->knownPoses.end() &&
                        scanDatabase[scanIdx2]->knownPoses.find("SLAM")!=scanDatabase[scanIdx2]->knownPoses.end())
                {
                    float distanceBetweenScans = (scanDatabase[scanIdx1]->knownPoses["SLAM"].inverse(Eigen::Isometry)*
                            scanDatabase[scanIdx2]->knownPoses["SLAM"]).translation().norm();
                    if (distanceBetweenScans > (averageRangeMeasurement>0 ? averageRangeMeasurement : 10.0f)) {
                        //cout << "Skipping "<<scanIdx1<<","<<scanIdx2<<" since they have SLAM distance "<<distanceBetweenScans<<"m.\n";
                        continue;
                    }
                }
            }
            rangeImageMatching.getBestPoseEstimates(*scanDatabase[scanIdx2], scan, poseEstimatesPerScan[scanIdx2]);
            double timeSinceCurrentScanStart = get_time()-currentScanStartTime;
            neededTimes[scanIdx2] = timeSinceCurrentScanStart;
            if (timeLimitForDatabaseQuery>0 && timeSinceCurrentScanStart>=timeLimitForDatabaseQuery)
                stop = true;
        }
        //rangeImageMatching.parameters.maxNoOfThreadsPerScanComparison = 1;
        timeForMatchingCurrentScan += get_time();

        for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
            //cout << PVARC(scanIdx1)<<PVARC(scanIdx2)<<PVARN(poseEstimatesPerScan[scanIdx2].size());
            if (!poseEstimatesPerScan[scanIdx2].empty()) {
                int confusionMatrixIdx = scanIdx2*noOfScans+scanIdx1;
                confusionMatrixScores[confusionMatrixIdx] = poseEstimatesPerScan[scanIdx2][0].score;
                confusionMatrixTransformations[confusionMatrixIdx].matrix() = poseEstimatesPerScan[scanIdx2][0].transformation.matrix();
            }
        }

        timeForScanCalculations += timeForCurrentScanCalculations;
        timeForConfusionMatrixCalculation += timeForMatchingCurrentScan;
        ++noOfDoneScans;
        //cout << "Doing scan calculations for scan "<<scanIdx1+1<<" took "<<timeForCurrentScanCalculations<<"s.\n"
        //<< "Matching against the database took "<<timeForMatchingCurrentScan<<"s.\n";

        double eta = (noOfDoneScans>0 ? (noOfScans-noOfDoneScans) * (get_time()-startTime)/noOfDoneScans : -1.0),
               etaHours = floor(eta/3600), etaMinutes = floor((eta-3600*etaHours)/60), etaSeconds = round(eta - 3600*etaHours - 60*etaMinutes);
        QString etaString = (eta>0 ? "  ETA: "+QString::number(etaHours)+"h "+QString::number(etaMinutes)+"m "+QString::number(etaSeconds)+"s" : "");
        window.progressBar->setValue(noOfDoneScans);
        window.statusLabel->setText("Calculating confusion matrix - Done scans: "+QString::number(noOfDoneScans)+"."+etaString);
        application.processEvents();
    }

    window.progressBar->setValue(noOfScans);
    window.statusLabel->setText("Done.");
    application.processEvents();

    if (!interrupted) {
        float timeToMatchScanAgainstDatabase = timeForConfusionMatrixCalculation/noOfScans;
        timeToMatchScanAgainstDatabase = round(1000.0f*timeToMatchScanAgainstDatabase);
        cout << "Calculating the confusion matrix of size "<<noOfScans<<"x"<<noOfScans<<" took "<<timeForConfusionMatrixCalculation<<"s, "
            << "meaning it took "<<timeToMatchScanAgainstDatabase<<"ms to match one scan against the database.\n";
        saveResult("timeToMatchScanAgainstDatabase", timeToMatchScanAgainstDatabase);
        float timeToMatchScanPair = timeToMatchScanAgainstDatabase/noOfScans;
        timeToMatchScanPair = round(timeToMatchScanPair);
        saveResult("timeToMatchScanPair", timeToMatchScanPair);

        string rotInv = (useRotationInvariance ? "RotInv" : "");
        string settings = rotInv;

        // Write confusion matrix to file
        std::string confusionMatrixFileName = scanDatabase.databaseDirectory+"/"+statisticsDirectory+"/confusionMatrix"+settings+"_"+scanDatabase.name+".dat";
        std::ofstream confusionMatrixFile(confusionMatrixFileName.c_str());
        for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
            for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
                int confusionMatrixIdx = scanIdx2*noOfScans+scanIdx1;
                float score = confusionMatrixScores[confusionMatrixIdx];
                const Eigen::Isometry3f& transformation = confusionMatrixTransformations[confusionMatrixIdx];
                float x, y, z, roll, pitch, yaw;
                pcl::getTranslationAndEulerAngles(transformation, x, y, z, roll, pitch, yaw);
                if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||!std::isfinite(roll)||!std::isfinite(pitch)||!std::isfinite(yaw))
                    score=x=y=z=roll=pitch=yaw = 0.0f;
                confusionMatrixFile << scanIdx1+1<<" "<<scanIdx2+1<<" "<<score<<" "<<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
            }
        }
        confusionMatrixFile.close();

        // Write timings to file
        std::string timingsFileName = scanDatabase.databaseDirectory+"/"+statisticsDirectory+"/confusionMatrixTimings"+settings+"_"+scanDatabase.name+".dat";
        std::ofstream timingsFile(timingsFileName.c_str());
        for (int scanIdx1=0; scanIdx1<noOfScans; ++scanIdx1) {
            for (int scanIdx2=0; scanIdx2<noOfScans; ++scanIdx2) {
                timingsFile << neededTimesPerScan[scanIdx1][scanIdx2]<<" ";
            }
        }
        confusionMatrixFile.close();

        confusionMatrixWidget.setRealImage(&confusionMatrixScores[0], noOfScans, noOfScans, true, 0.0f, 1.0f);
        std::string confusionMatrixImageFile = scanDatabase.databaseDirectory+"/"+statisticsDirectory+"/ConfusionMatrix"+settings+"_"+scanDatabase.name+".tiff";
        confusionMatrixWidget.save(confusionMatrixImageFile.c_str());
    }
    else {
        confusionMatrixWidget.setVisible(false);
    }

    window.statusFrame->hide();
}

template <typename ValueType>
void saveResult(const std::string& valueName, ValueType value, bool datasetSpecific=true) {
    std::string token = "\\newcommand{\\"+valueName;
    if (datasetSpecific) {
        if (useRotationInvariance)
            token = token + "RotInv";
        token = token + scanDatabase.configFileParams["shortName"];
    }
    token = token + "}";

    std::stringstream ss;
    std::string fileName = scanDatabase.databaseDirectory+"/"+statisticsDirectory+"/results.tex";
    std::ifstream file(fileName.c_str());
    if (file) {
        std::string line;
        while (!file.eof()) {
            std::getline(file, line);
            bool startsWithToken = !line.compare(0, token.size(), token);
            if (!startsWithToken && !line.empty()) {
                ss << line << "\n";
                continue;
            }
        }
        file.close();
    }
    else {
        std::cerr << "Could not open file \""<<fileName<<"\".\n";
    }

    ss << token << "{"<<value<<"}\n";

    std::ofstream fileOut(fileName.c_str());
    if (!fileOut) {
        std::cerr << "Could not open file \""<<fileName<<"\" to write.\n";
        return;
    }
    fileOut << ss.str();
    fileOut.close();
}

#pragma GCC diagnostic ignored "-Wunused-parameter"  // Do not show warnings from ROS - for some weird reason
// some stuff appears after the end of the code...
