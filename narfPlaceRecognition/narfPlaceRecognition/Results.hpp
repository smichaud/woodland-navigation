#ifndef RESULTS_H
#define RESULTS_H

#include "mainWidget.h"
#include "narfPlaceRecognitionLib/scanDatabase.h"

class Results {
    private:
        enum TestResult {
            TruePositive, TrueNegative, FalsePositive, FalseNegative};
        ScanDatabase& scanDatabase;
        MainWidget &window;
        std::vector<float> &confusionMatrixScores;

    public:
        Results(ScanDatabase& scanDatabase, MainWidget &window,
                std::vector<float> &confusionMatrixScores);
        void compute();
        void plotStuff();

    private:
        int getMatrixIndex(int scan1Index, int scan2Index);
        std::pair<int,int> getScanIndices(int matrixIndex);
};

#endif
