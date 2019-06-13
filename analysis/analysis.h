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
              const std::vector<int> &vecRobotState, const std::vector<float> vecAvgRobotState0,
              const std::vector<float> vecAvgRobotState1) :
            m_vecRobotX(vecRobotX),
            m_vecRobotY(vecRobotY),
            m_vecRobotZ(vecRobotZ),
            m_vecRobotSpeed(vecRobotSpeed),
            m_vecRobotState(vecRobotState),
            m_vecAvgRobotState0(vecAvgRobotState0),
            m_vecAvgRobotState1(vecAvgRobotState1) {}

    virtual ~CAnalysis() {}

    typedef struct {
        int size = 7;
        float Scatter;
        float RadialStdDev;
        float Speed;
        float AngularMomentum;
        int StateZeroCount;
        float AvgStateTime0;
        float AvgStateTime1;
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
