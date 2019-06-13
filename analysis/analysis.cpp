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

    AnalyzeScatter();
    AnalyzeRadialVariance();
    AnalyzeGroupRotation();
    AnalyzeSpeed();
    AnalyzeAngMomentum();
    AnalyzeState();

    results.Scatter = this->scatter;
    results.RadialVariance = this->radialVariance;
    results.Speed = this->speed;
    results.AngularMomentum = this->angMomentum;
    results.GroupRotation = this->groupRotation;
    results.StateZeroCount = this->state0Count;
    results.AvgStateTime0 = this->avgStateTime0;
    results.AvgStateTime1 = this->avgStateTime1;

    return results;
}

void CAnalysis::AnalyzeScatter() {
    //Look I know there are chances that they might be different sizes, but if that happens something else is fucked
    int size = (int) m_vecRobotX.size();

    this->centroidX = (float) accumulate(m_vecRobotX.begin(), m_vecRobotX.end(), 0.0) / m_vecRobotX.size();
    this->centroidY = (float) accumulate(m_vecRobotY.begin(), m_vecRobotY.end(), 0.0) / m_vecRobotY.size();

    std::vector<float> vecScatter;

    for (int i = 0; i < size; ++i) {
        auto distance = (float) sqrt(pow(m_vecRobotX[i] - centroidX, 2) + pow(m_vecRobotY[i] - centroidY, 2));
        vecScatter.push_back(distance);
    }

    auto avgScatter = (float) accumulate(vecScatter.begin(), vecScatter.end(), 0.0) / (vecScatter.size() * (float) pow(arenaSize, 2));
    scatter = avgScatter;
}

void CAnalysis::AnalyzeRadialVariance() {
    unsigned long size = m_vecRobotX.size();

    std::vector<float> vecAvgDist;
    for (int i = 0; i < size; ++i) {
        auto distance = (float) sqrt(pow(m_vecRobotX[i] - centroidX, 2) + pow(m_vecRobotY[i] - centroidY, 2));
        vecAvgDist.push_back(distance);
    }
    auto avgDist = (float) accumulate(vecAvgDist.begin(), vecAvgDist.end(), 0.0) / m_vecRobotX.size();

    std::vector<float> vecRadialVariance;
    for (int j = 0; j < size; ++j) {
        auto distance = (float) sqrt(pow(m_vecRobotX[j] - centroidX, 2) + pow(m_vecRobotY[j] - centroidY, 2));
        vecRadialVariance.push_back(distance - avgDist);
    }

    radialVariance = (float) accumulate(vecRadialVariance.begin(), vecRadialVariance.end(), 0.0) / (vecRadialVariance.size() * (float) pow(arenaSize, 2));
}

void CAnalysis::AnalyzeSpeed() {
    double avgSpeed = accumulate(m_vecRobotSpeed.begin(), m_vecRobotSpeed.end(), 0.0) / m_vecRobotSpeed.size();
    speed = float(avgSpeed);
}

void CAnalysis::AnalyzeAngMomentum() {
    std::vector<float> vecCrossProd;
    for(int i = 0; i < m_vecRobotX.size(); ++i){
        float x_pos = m_vecRobotX[i];
        float y_pos = m_vecRobotY[i];

        float x_diff = x_pos - centroidX;
        float y_diff = y_pos - centroidY;

        float x_vel = m_vecRobotSpeed[i] * (float) cos(m_vecRobotZ[i]);
        float y_vel = m_vecRobotSpeed[i] * (float) sin(m_vecRobotZ[i]);

        float cross_prod = x_diff * y_vel - y_diff * x_vel;

        vecCrossProd.push_back(cross_prod);
    }

    angMomentum = (float) accumulate(vecCrossProd.begin(), vecCrossProd.end(), 0.0) / (float) (vecCrossProd.size() * 3.535);
}

void CAnalysis::AnalyzeGroupRotation() {
    std::vector<float> vecCrossProd;
    for(int i = 0; i < m_vecRobotX.size(); ++i){
        float x_pos = m_vecRobotX[i];
        float y_pos = m_vecRobotY[i];

        float x_diff = x_pos - centroidX;
        float y_diff = y_pos - centroidY;

        auto mag = (float) sqrt(pow(x_diff, 2) + pow(y_diff, 2));

        x_diff /= mag;
        y_diff /= mag;

        float x_vel = m_vecRobotSpeed[i] * (float) cos(m_vecRobotZ[i]);
        float y_vel = m_vecRobotSpeed[i] * (float) sin(m_vecRobotZ[i]);

        float cross_prod = x_diff * y_vel - y_diff * x_vel;

        vecCrossProd.push_back(cross_prod);
    }

    groupRotation = (float) accumulate(vecCrossProd.begin(), vecCrossProd.end(), 0.0) / vecCrossProd.size();
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
