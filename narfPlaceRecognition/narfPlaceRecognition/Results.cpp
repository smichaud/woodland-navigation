#include "Results.hpp"
#include <fstream>

Results::Results(ScanDatabase& scanDatabase,
        std::vector<float> &confusionMatrixScores):
    scanDatabase(scanDatabase),
    confusionMatrixScores(confusionMatrixScores) {
    }

void Results::compute() {
    // [TODO]: Get the odom and create the ROC curve - 2015-07-23 04:46pm
    // Compute the true/false positive/negative --> Recall rate(TP / TP+FN)
    // Article use recall vs dist. and vs scores to match
    this->createDistancesMap();
    this->getRecallForDistances(0.05);
    //this->plotStuff();
}

void Results::createDistancesMap() {
    int scansCount = scanDatabase.size();

    for (int scan1Index = 0; scan1Index < scansCount; ++scan1Index) {
        for (int scan2Index = 0; scan2Index < scansCount; ++scan2Index) {
            int confusionMatrixIdx = getMatrixIndex(scan1Index, scan2Index);

            ScanDatabaseElement& scan1 = *scanDatabase[scan1Index];
            const Eigen::Isometry3f& poseGuess1 = scan1.knownPoses["Odometry"];
            Eigen::Vector3f position1 = poseGuess1.translation();

            ScanDatabaseElement& scan2 = *scanDatabase[scan2Index];
            const Eigen::Isometry3f& poseGuess2 = scan2.knownPoses["Odometry"];
            Eigen::Vector3f position2 = poseGuess2.translation();

            Eigen::Vector3f positionDiff = (position2-position1);
            this->distanceMapMatrixIndex.insert(
                    std::pair<float, int>(
                        positionDiff.norm(), confusionMatrixIdx));
        }
    }
}

// [TODO]: Working on this method - 2015-07-31 01:55pm
std::map<float, float> Results::getRecallForDistances(float scoreThreshold) {
    int TNCount = 0, TPCount = 0, FNCount = 0, FPCount = 0;
    std::map<float, float> recallForDistances;

    // Init testResults for distance == 0
    for(std::pair<float, int> it : this->distanceMapMatrixIndex) {
        int matrixIndex = it.second;
        std::pair<int, int> scanIndices = this->getScanIndices(matrixIndex);
        if(scanIndices.first != scanIndices.second) {
            float score = confusionMatrixScores[matrixIndex];
            (score > scoreThreshold) ? FPCount++ : TNCount++;
        }
    }

    // Recall recall rate for other distances
    float lastDistance = 0;
    for(std::pair<float, int> it : this->distanceMapMatrixIndex) {
        int matrixIndex = it.second;
        std::pair<int, int> scanIndices = this->getScanIndices(matrixIndex);

        if(scanIndices.first != scanIndices.second) {
            float distance = it.first;
            if(distance != lastDistance) {
                float recallRate = this->getRecallRate(TPCount, FNCount);
                recallForDistances.insert(std::pair<float,float>(
                            lastDistance, recallRate));
                lastDistance = distance;
            }

            float score = confusionMatrixScores[matrixIndex];

            std::cout << "Dist:" << distance << std::endl;
            std::cout << "Score:" << score << std::endl;
            std::cout << "TP:" << TPCount << std::endl;
            std::cout << "TN:" << TNCount << std::endl;
            std::cout << "FP:" << FPCount << std::endl;
            std::cout << "FN:" << FNCount << std::endl;
            getchar();

            TestResult lastTestResult = this->getTestResult(
                    lastDistance, lastDistance, score, scoreThreshold);
            switch(lastTestResult) {
                case TruePositive:
                    TPCount--;
                    std::cout << "TruePositive" << std::endl;
                    break;
                case TrueNegative:
                    TNCount--;
                    std::cout << "TrueNegative" << std::endl;
                    break;
                case FalsePositive:
                    FPCount--;
                    std::cout << "FalsePositive" << std::endl;
                    break;
                case FalseNegative:
                    FNCount--;
                    std::cout << "FalseNegative" << std::endl;
                    break;
            }

            //TestResult currentTestResult = this->getTestResult(
            //distance, distance, score, scoreThreshold);
            //switch(currentTestResult) {
            //case TruePositive: TPCount++; break;
            //case TrueNegative: TNCount++; break;
            //case FalsePositive: FPCount++; break;
            //case FalseNegative: FNCount++; break;
            //}
        }
    }
    // Record the recall rate for the last distance
    float recallRate = this->getRecallRate(TPCount, FNCount);
    recallForDistances.insert(std::pair<float,float>(
                lastDistance, recallRate));

    return recallForDistances;
}

void Results::plotStuff() {
    if(this->customPlot == NULL) {
        this->customPlot = new QCustomPlot();
    }

    std::string fileName = scanDatabase.databaseDirectory+"/plot/testPlot.pdf";

    QVector<double> a(101), b(101);
    for (int i=0; i<101; ++i) {
        a[i] = i/50.0 - 1;
        b[i] = a[i]*a[i];
    }

    this->customPlot->addGraph();
    this->customPlot->graph(0)->setData(a, b);
    this->customPlot->xAxis->setLabel("x");
    this->customPlot->yAxis->setLabel("y");
    this->customPlot->xAxis->setRange(-1, 1);
    this->customPlot->yAxis->setRange(0, 1);
    this->customPlot->show();
    this->customPlot->savePdf(QString::fromStdString(fileName));
}


void Results::saveDistIndicesScore(std::string filename) {
    std::ofstream file;
    file.open(filename);
    for(std::pair<float, int> it : this->distanceMapMatrixIndex) {
        int matrixIndex = it.second;
        std::pair<int, int> scanIndices = this->getScanIndices(matrixIndex);

        file << it.first << " : "
            << scanIndices.first << "," << scanIndices.second << std::endl;
    }
    file.close();
}

Results::TestResult Results::getTestResult(float distance,
        float distanceThreshold, float score, float scoreThreshold) {
    if(score > scoreThreshold) {
        if(distance <= distanceThreshold) {
            std::cout << "TP" << std::endl;
            return TruePositive;
        } else {
            std::cout << "FP" << std::endl;
            return FalsePositive;
        }
    } else {
        if(distance <= distanceThreshold) {
            std::cout << "FN" << std::endl;
            return FalseNegative;
        } else {
            std::cout << "TN" << std::endl;
            return TrueNegative;
        }
    }
}

float Results::getRecallRate(int TruePositiveCount, int FalseNegativeCount) {
    if(TruePositiveCount+FalseNegativeCount == 0) {
        return 1.0f;
    }
    return TruePositiveCount/(TruePositiveCount+FalseNegativeCount);
}

int Results::getMatrixIndex(int scan1Index, int scan2Index) {
    return scan2Index*scanDatabase.size()+scan1Index;
}

std::pair<int, int> Results::getScanIndices(int matrixIndex) {
    int first = matrixIndex % scanDatabase.size();
    int second = static_cast<int>(matrixIndex/scanDatabase.size());

    std::pair<int, int> scanPair(first, second);
    return scanPair;
}

Results::~Results() {
    if(customPlot != NULL) {
        delete customPlot;
    }
}
