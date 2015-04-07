#include <iostream>
#include <fstream>
#include "spmDictionarySelection.h"
#include "basics/timeutil.h"
#include "../spmBase/narfWord.h"
#include "../spmBase/narfInstance.h"

using namespace std;
using namespace pcl;

void printUsage(const char* progName)
{
  cout << "Usage: " << progName << " [options] inSPModel pcdFile outSPModel" << endl << endl;
  cout << "calculates compressed dictionary by evaluating the BIC" << endl;
  cout << "Options:" << endl;
  cout << "-------------------------------------------" << endl;
  cout << "-k               cluster dictionary with kmeans" << endl;
  //cout << "-t             write total map (instead of just poses to generate the map)" << endl;
  //cout << "-d             show debug outputs and range images" << endl;
  cout << "-i <int>         set max iterations" << endl;
  cout << "-n <int>         set number of iterations without change" << endl;
  cout << "-r <double>      set evaluation resolution" << endl;
  cout << "-s <double>      set sigma" << endl;
  cout << "-c <string>      resume with the dictionary from given fileName" << endl;
  //cout << "-p <string>    " << endl;
  cout << "-h             this help" << endl;
}

int main(int argc, char** argv)
{
   int firstN = 0;
   bool debug = false;

   SelectionParams params;
   params.maxIterations = 1e6;
   params.iterationsWithoutChange = 1000;
   params.evaluationResolution = 0.01;
   params.sigma = 0.0001;
   bool useDicFile = false;
   bool writeDescriptorSimMatrix = false;
   double maxDescriptorDist = 0.05;
   bool clusterWithKMeans = false;
   int k = 20;
   string dicFileName; 
   for(char c; (c = getopt(argc, argv, "k:c:i:n:s:r:tdw:h")) != -1; ) {
      switch(c) {
         case 'k':
          clusterWithKMeans = true;
          k = atoi(optarg);
         break;
         case 'c':
         dicFileName = optarg;
         useDicFile = true;
         break;
         case 't':
         //writeTotalMap = true;
         break;
         case 'i':
         params.maxIterations = atoi(optarg);
         break;
         case 'n':
         params.iterationsWithoutChange = atoi(optarg);
         break;
         case 's':
         params.sigma = atof(optarg);
         break;
         case 'r':
         params.evaluationResolution = atof(optarg);
         break;
         case 'd':
         debug = true;
         break;
         case 'w':
         maxDescriptorDist = atof(optarg);
         writeDescriptorSimMatrix = true;
         break;
         case 'h':
         printUsage(argv[0]);
         return 0;
         default:
         printUsage(argv[0]);
         return 1;
      }
   }
   if (optind != argc - 3) {
      printUsage(argv[0]);
      return 1;
   }

   string inFilename = argv[optind++];
   string cloudFilename = argv[optind++];
   string outFilename = argv[optind++];

   SPModel model;
   ifstream file;
   file.open(inFilename.c_str());
   model.readBinary(file);
   file.close();



   pcl::PointCloud<pcl::PointXYZ> cloud;
   pcl::io::loadPCDFile(cloudFilename, cloud);
   if(cloud.size() == 0){
      cerr << "Error loading input point cloud." << endl;
   }
   cerr << "starting dictionary clustering procedure" << endl;
   //Setting random seed
   srand ( time(NULL) );

   double startTime = Ais3dTools::get_time();
   SPMDictionarySelector selector;
   if(useDicFile)
      selector.readBinary(dicFileName.c_str(), model);
   if(writeDescriptorSimMatrix){
      selector.clusterDictionaryWithDescriptorDistance(model, maxDescriptorDist);
      cerr << "Saving dictionary to file ddDictionary.dic" << endl;
      selector.writeBinary("ddDictionary.dic");

      cerr << "Transfer new _dictionary to model" << endl;
      model.importDictionary(selector._dictionary);
   } else {
      if(clusterWithKMeans){
        selector.clusterDictionaryWithkMeans(model, k, 10);
      } else {
        selector.clusterDictionary(model, cloud, params);
      }
   }
   std::cout << "Dictionary Calculation took " << (Ais3dTools::get_time() - startTime) << " seconds" << std::endl;
   ofstream ofile;
   ofile.open(outFilename.c_str());
   model.writeBinary(ofile);
   ofile.close();

 }
