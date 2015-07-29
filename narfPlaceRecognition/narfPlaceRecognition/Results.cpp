#include "Results.hpp"

Results::Results(ScanDatabase& scanDatabase, MainWidget &window,
        std::vector<float> &confusionMatrixScores):
    scanDatabase(scanDatabase), window(window),
    confusionMatrixScores(confusionMatrixScores) {
    }

void Results::compute() {
    std::cout << "Computing results..." << std::endl;

    int scansCount = scanDatabase.size();

    for (int scan1Index = 0; scan1Index < scansCount; ++scan1Index) {
        for (int scan2Index = 0; scan2Index < scansCount; ++scan2Index) {
            int confusionMatrixIdx = getMatrixIndex(scan1Index, scan2Index);
            // [TODO]: Get the real score - 2015-07-29 03:55pm
            float score = confusionMatrixScores[confusionMatrixIdx];

            ScanDatabaseElement& scan1 = *scanDatabase[scan1Index];
            const Eigen::Isometry3f& poseGuess1 = scan1.knownPoses["Odometry"];
            Eigen::Vector3f position1 = poseGuess1.translation();

            ScanDatabaseElement& scan2 = *scanDatabase[scan2Index];
            const Eigen::Isometry3f& poseGuess2 = scan2.knownPoses["Odometry"];
            Eigen::Vector3f position2 = poseGuess2.translation();

            Eigen::Vector3f positionDiff = (position2-position1);
            float x = positionDiff(0);
            float y = positionDiff(1);
            float z = positionDiff(2);
            float distanceBetweenScans = positionDiff.norm();
            std::cout << "[" << scan1Index << "," << scan2Index << "] -> "
                << "(" << x << "," << y << "," << z << ") = "
                << distanceBetweenScans << std::endl;
            std::cout << "Score:" << score << std::endl;

            //// [TODO]: Get the odom and create the ROC curve - 2015-07-23 04:46pm
            //// The true/false positive/negative is calculated in CreateConfusionMatrixStatistics
            //// Compute the true/false positive/negative --> Recall rate(TP / TP+FN)
            //// Article use recall vs dist. and vs scores to match
            //if(score > scoreTreshold) {
                //if(distanceBetweenScans < maxDistanceToConsiderScansOverlapping) {
                    //std::cout << "True Positive" << std::endl;
                //} else {
                    //std::cout << "False Positive" << std::endl;
                //}
            //} else {
                //if(distanceBetweenScans < maxDistanceToConsiderScansOverlapping) {
                    //std::cout << "False Negative" << std::endl;
                //} else {
                    //std::cout << "True Negative" << std::endl;
                //}
            //}
        }
    }
}

void Results::plotStuff() {
    std::string fileName = scanDatabase.databaseDirectory+"/plot/testPlot.pdf";

    QVector<double> a(101), b(101);
    for (int i=0; i<101; ++i) {
        a[i] = i/50.0 - 1;
        b[i] = a[i]*a[i];
    }

    window.customPlot.addGraph();
    window.customPlot.graph(0)->setData(a, b);
    window.customPlot.xAxis->setLabel("x");
    window.customPlot.yAxis->setLabel("y");
    window.customPlot.xAxis->setRange(-1, 1);
    window.customPlot.yAxis->setRange(0, 1);
    //window.customPlot.show();
    //window.customPlot.replot();
    window.customPlot.savePdf(QString::fromStdString(fileName));
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
