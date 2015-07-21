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
#include "narfPlaceRecognitionLib/scanDatabase.h"
#include "narfPlaceRecognitionLib/rangeImageMatching.h"
#include "visMapCloud.h"
#include "EXTERNALS/ais3dTools/visualization/imageWidget/imageWidget.h"
#include "EXTERNALS/ais3dTools/visualization/aluGLViewer/alu_glviewer.h"
#include "EXTERNALS/ais3dTools/visualization/aluGLViewerObjects/glo_pcl_pointcloud.h"
#include "mainWidget.h"

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

int main(int argc, char *argv[]) {
    std::cout << "Hello world !" << std::endl;
    if(1==1) {

    }
    return 0;
}
