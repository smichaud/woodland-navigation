#ifndef RESULTS_H
#define RESULTS_H

#include "mainWidget.h"
#include "narfPlaceRecognitionLib/scanDatabase.h"

class Results {
    private:
        enum TestResult {
            TruePositive, TrueNegative, FalsePositive, FalseNegative};

        ScanDatabase& scanDatabase;
        std::vector<float> &confusionMatrixScores;
        std::multimap<float, int> distanceMapMatrixIndex;

        QCustomPlot *customPlot; 

    public:
        Results(ScanDatabase& scanDatabase,
                std::vector<float> &confusionMatrixScores);
        ~Results();

        void compute();
        std::map<float, float> getRecallForDistances(float scoreThreshold);
        void plotStuff();
        void saveDistIndicesScore(
                std::string filename = "/home/smichaud/Desktop/debug.txt");

    private:
        void createDistancesMap();
        TestResult getTestResult(float distance, float distanceThreshold,
                float score, float scoreThreshold);
        float getRecallRate(int TruePositiveCount, int FalseNegativeCount);
        int getMatrixIndex(int scan1Index, int scan2Index);
        std::pair<int,int> getScanIndices(int matrixIndex);
};

#endif
