//
// Created by djcupo on 4/18/19.
//

#ifndef EMERGENT_BEHAVIOR_CANALYSIS_H
#define EMERGENT_BEHAVIOR_CANALYSIS_H

#include <map>
#include <vector>

class CAnalysis {
public:
    CAnalysis(const std::vector<float> &vecRobotX, const std::vector<float> &vecRobotY,
              const std::vector<float> &vecRobotZ, const std::vector<float> &vecRobotSpeed,
              const std::vector<int> &vecRobotState, const std::vector<float> &vecAvgRobotState0,
              const std::vector<float> &vecAvgRobotState1) :
            m_vecRobotX(vecRobotX),
            m_vecRobotY(vecRobotY),
            m_vecRobotZ(vecRobotZ),
            m_vecRobotSpeed(vecRobotSpeed),
            m_vecRobotState(vecRobotState),
            m_vecAvgRobotState0(vecAvgRobotState0),
            m_vecAvgRobotState1(vecAvgRobotState1) {
        scatter = -9.9f;
        radialVariance = -9.9f;
        speed = -9.9f;
        angMomentum = -9.9f;
        groupRotation = -9.9f;
        state0Count = -9;
        avgStateTime0 = -9.9f;
        avgStateTime1 = -9.9f;
        centroidX = -9.9f;
        centroidY = -9.9f;
    }

    virtual ~CAnalysis() = default;

    typedef struct {
        int size = 8;
        float Scatter = 0.0;
        float RadialVariance = 0.0;
        float Speed = 0.0;
        float AngularMomentum = 0.0;
        float GroupRotation = 0.0;
        int StateZeroCount = 0;
        float AvgStateTime0 = 0.0;
        float AvgStateTime1 = 0.0;
    } AnalysisResults;

    AnalysisResults AnalyzeAll();

private:
    std::vector<float> m_vecRobotX;
    std::vector<float> m_vecRobotY;
    std::vector<float> m_vecRobotZ;
    std::vector<float> m_vecRobotSpeed;
    std::vector<int> m_vecRobotState;
    std::vector<float> m_vecAvgRobotState0;
    std::vector<float> m_vecAvgRobotState1;

    float scatter;
    float radialVariance;
    float speed;
    float angMomentum;
    float groupRotation;
    int state0Count;
    float avgStateTime0;
    float avgStateTime1;

    float arenaSize = 3.535;

    float centroidX;
    float centroidY;

    void AnalyzeScatter();

    void AnalyzeRadialVariance();

    void AnalyzeSpeed();

    void AnalyzeAngMomentum();

    void AnalyzeGroupRotation();

    void AnalyzeState();

};


#endif //EMERGENT_BEHAVIOR_CANALYSIS_H
