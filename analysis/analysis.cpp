//
// Created by djcupo on 4/18/19.
//

#include "analysis.h"
#include <cmath>
#include <numeric>
#include <vector>
#include <algorithm>

CAnalysis::AnalysisResults CAnalysis::AnalyzeAll() {
    AnalysisResults results;

    AnalyzeSparseness();
    AnalyzeSpeed();
    AnalyzeAngMomentum();
    AnalyzeState();

    results.Sparseness = this->sparseness;
    results.RadialStdDev = this->radialStdDev;
    results.Speed = this->speed;
    results.AngularMomentum = this->angMomentum;
    results.StateZeroCount = this->state0Count;
    results.AvgStateTime0 = this->avgStateTime0;
    results.AvgStateTime1 = this->avgStateTime1;

    return results;
}

void CAnalysis::AnalyzeSparseness() {
    //Look I know there are chances that they might be different sizes, but if that happens something else is fucked
    int size = (int) m_vecRobotX.size();

    double centroidX = 0.0;
    double centroidY = 0.0;

    for (int i = 0; i < size; ++i) {
        centroidX += m_vecRobotX[i];
        centroidY += m_vecRobotY[i];
    }

    centroidX /= (double) size;
    centroidY /= (double) size;

    this->centroidX = centroidX;
    this->centroidY = centroidY;

    std::vector<float> vecSparseness;

    for (int i = 0; i < size; ++i) {
        auto distance = (float) sqrt(pow(m_vecRobotX[i] - centroidX, 2) + pow(m_vecRobotY[i] - centroidY, 2));
        vecSparseness.push_back(distance);
    }

    auto avgSparseness = (float) accumulate(vecSparseness.begin(), vecSparseness.end(), 0.0) / vecSparseness.size();
    sparseness = avgSparseness;

    std::vector<double> diff(vecSparseness.size());
    std::transform(vecSparseness.begin(), vecSparseness.end(), diff.begin(), [avgSparseness](double x) { return x - avgSparseness; });
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / vecSparseness.size());

    radialStdDev = (float) stdev;
}

void CAnalysis::AnalyzeSpeed() {
    double avgSpeed = accumulate(m_vecRobotSpeed.begin(), m_vecRobotSpeed.end(), 0.0) / m_vecRobotSpeed.size();
    speed = float(avgSpeed);
}

void CAnalysis::AnalyzeAngMomentum() {
    auto avgAngle = (float) accumulate(m_vecRobotZ.begin(), m_vecRobotZ.end(), 0.0) / m_vecRobotZ.size();
    angMomentum = avgAngle;
}

void CAnalysis::AnalyzeState() {
    int size = (int) m_vecRobotState.size();

    int counter = 0;
    for (int i = 0; i < size; ++i) {
        if (m_vecRobotState[i] == 0){
            counter++;
        }
    }
    state0Count = counter;

    auto avgTime0 = (float) accumulate(m_vecAvgRobotState0.begin(), m_vecAvgRobotState0.end(), 0.0) / m_vecAvgRobotState0.size();
    auto avgTime1 = (float) accumulate(m_vecAvgRobotState1.begin(), m_vecAvgRobotState1.end(), 0.0) / m_vecAvgRobotState1.size();

    avgStateTime0 = avgTime0;
    avgStateTime1 = avgTime1;
}
