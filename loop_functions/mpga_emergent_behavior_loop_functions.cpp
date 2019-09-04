#include "mpga_emergent_behavior_loop_functions.h"
#include <buzz/argos/buzz_controller.h>


/****************************************/
/****************************************/

CMPGAEmergentBehaviorLoopFunctions::CMPGAEmergentBehaviorLoopFunctions() {
    /*
     * Create the random number generator
     */
    m_pcRNG = CRandom::CreateRNG("argos");
    m_pfControllerParams.resize(GENOME_SIZE);
}

/****************************************/
/****************************************/

CMPGAEmergentBehaviorLoopFunctions::~CMPGAEmergentBehaviorLoopFunctions() {
    m_pfControllerParams.clear();
}

/****************************************/
/****************************************/

/**
 * Functor to put the stimulus in the Buzz VMs.
 */
struct PutGenome : public CBuzzLoopFunctions::COperation {

    /** Constructor */
    explicit PutGenome(const std::vector<double> &vec_genome) : m_vecGenome(vec_genome) {}

    /** The action happens here */
    virtual void operator()(const std::string str_robot_id,
                              buzzvm_t t_vm) {
        /* Set the values of the table 'genome' in the Buzz VM */
        BuzzTableOpen(t_vm, "genome");
        for (int i = 0; i < m_vecGenome.size(); ++i) {
            BuzzTablePut(t_vm, i, (float) m_vecGenome[i]);
        }
        BuzzTableClose(t_vm);
    }

    /** Genome */
    const std::vector<double> &m_vecGenome;
};


/****************************************/
/****************************************/

/**
 * Functor to get data from the robots
 */
struct GetFinalRobotData : public CBuzzLoopFunctions::COperation {

    /** Constructor */
    //temp is unneeded, but if we get rid of it, it errors out. So we left it in *shrug*
    GetFinalRobotData(int t) : temp(t) {}

    /** The action happens here */
    virtual void operator()(const std::string str_robot_id,
                              buzzvm_t t_vm) {

        // get the buzzobj corresponding to the value we want
        buzzobj_t tCurX = BuzzGet(t_vm, "cur_x");

        // confirm the value is the type we expect. Print error and then exit if they don't match
        if (!buzzobj_isfloat(tCurX)) {
            LOGERR << str_robot_id << ": variable 'cur_x' has wrong type " << buzztype_desc[tCurX->o.type] << std::endl;
            return;
        }

        //Cast the value to the appropriate type and variable
        float fCurX = buzzobj_getfloat(tCurX);

        //Repeat the process with every other variable
        buzzobj_t tCurY = BuzzGet(t_vm, "cur_y");
        if (!buzzobj_isfloat(tCurY)) {
            LOGERR << str_robot_id << ": variable 'cur_y' has wrong type " << buzztype_desc[tCurY->o.type] << std::endl;
            return;
        }

        float fCurY = buzzobj_getfloat(tCurY);

        buzzobj_t tCurZ = BuzzGet(t_vm, "cur_z");
        if (!buzzobj_isfloat(tCurZ)) {
            LOGERR << str_robot_id << ": variable 'cur_z' has wrong type " << buzztype_desc[tCurZ->o.type] << std::endl;
            return;
        }

        float fCurZ = buzzobj_getfloat(tCurZ);

        buzzobj_t tCurS = BuzzGet(t_vm, "cur_s");
        if (!buzzobj_isint(tCurS)) {
            LOGERR << str_robot_id << ": variable 'cur_s' has wrong type " << buzztype_desc[tCurS->o.type] << std::endl;
            return;
        }

        int iCurS = buzzobj_getint(tCurS);


        buzzobj_t tCurR = BuzzGet(t_vm, "cur_r");
        if (!buzzobj_isint(tCurR)) {
            LOGERR << str_robot_id << ": variable 'cur_r' has wrong type " << buzztype_desc[tCurR->o.type] << std::endl;
            return;
        }

        int iCurR = buzzobj_getint(tCurR);

        buzzobj_t tCurSpeed = BuzzGet(t_vm, "cur_speed");
        if (!buzzobj_isfloat(tCurSpeed)) {
            LOGERR << str_robot_id << ": variable 'cur_speed' has wrong type " << buzztype_desc[tCurSpeed->o.type]
                   << std::endl;
            return;
        }

        float fCurSpeed = buzzobj_getfloat(tCurSpeed);

        //Add each value to the back of the vector that stores all this
        m_vecRobotX.push_back(fCurX);
        m_vecRobotY.push_back(fCurY);
        m_vecRobotZ.push_back(fCurZ);
        m_vecRobotSpeed.push_back(fCurSpeed);
        m_vecRobotReading.push_back(iCurR);
        m_vecRobotState.push_back(iCurS);

    }

    int temp;

    std::vector<float> m_vecRobotX;
    std::vector<float> m_vecRobotY;
    std::vector<float> m_vecRobotZ;
    std::vector<int> m_vecRobotState;
    std::vector<int> m_vecRobotReading;
    std::vector<float> m_vecRobotSpeed;

};

/****************************************/
/****************************************/

struct GetStepRobotData : public CBuzzLoopFunctions::COperation {

    /** Constructor */
    //temp is unneeded, but if we get rid of it, it errors out. So we left it in *shrug*
    GetStepRobotData(int t) : temp(t) {}

    /** The action happens here */
    virtual void operator()(const std::string str_robot_id,
                            buzzvm_t t_vm) {

        // get the buzzobj corresponding to the value we want
        buzzobj_t tCurX = BuzzGet(t_vm, "cur_x");

        // confirm the value is the type we expect. Print error and then exit if they don't match
        if (!buzzobj_isfloat(tCurX)) {
            LOGERR << str_robot_id << ": variable 'cur_x' has wrong type " << buzztype_desc[tCurX->o.type] << std::endl;
            return;
        }

        //Cast the value to the appropriate type and variable
        float fCurX = buzzobj_getfloat(tCurX);

        //Repeat the process with every other variable
        buzzobj_t tCurY = BuzzGet(t_vm, "cur_y");
        if (!buzzobj_isfloat(tCurY)) {
            LOGERR << str_robot_id << ": variable 'cur_y' has wrong type " << buzztype_desc[tCurY->o.type] << std::endl;
            return;
        }

        float fCurY = buzzobj_getfloat(tCurY);

        buzzobj_t tCurZ = BuzzGet(t_vm, "cur_z");
        if (!buzzobj_isfloat(tCurZ)) {
            LOGERR << str_robot_id << ": variable 'cur_z' has wrong type " << buzztype_desc[tCurZ->o.type] << std::endl;
            return;
        }

        float fCurZ = buzzobj_getfloat(tCurZ);

        buzzobj_t tCurS = BuzzGet(t_vm, "cur_s");
        if (!buzzobj_isint(tCurS)) {
            LOGERR << str_robot_id << ": variable 'cur_s' has wrong type " << buzztype_desc[tCurS->o.type] << std::endl;
            return;
        }

        int iCurS = buzzobj_getint(tCurS);


        buzzobj_t tCurR = BuzzGet(t_vm, "cur_r");
        if (!buzzobj_isint(tCurR)) {
            LOGERR << str_robot_id << ": variable 'cur_r' has wrong type " << buzztype_desc[tCurR->o.type] << std::endl;
            return;
        }

        int iCurR = buzzobj_getint(tCurR);

        buzzobj_t tCurSpeed = BuzzGet(t_vm, "cur_speed");
        if (!buzzobj_isfloat(tCurSpeed)) {
            LOGERR << str_robot_id << ": variable 'cur_speed' has wrong type " << buzztype_desc[tCurSpeed->o.type]
                   << std::endl;
            return;
        }

        float fCurSpeed = buzzobj_getfloat(tCurSpeed);

        buzzobj_t tCurTick = BuzzGet(t_vm, "tick");
        if (!buzzobj_isint(tCurTick)) {
            LOGERR << str_robot_id << ": variable 'tick' has wrong type " << buzztype_desc[tCurTick->o.type]
                   << std::endl;
            return;
        }
        int iCurTick = buzzobj_getint(tCurTick);

        //Add each value to the back of the vector that stores all this
        m_vecRobotX.push_back(fCurX);
        m_vecRobotY.push_back(fCurY);
        m_vecRobotZ.push_back(fCurZ);
        m_vecRobotSpeed.push_back(fCurSpeed);
        m_vecRobotReading.push_back(iCurR);
        m_vecRobotState.push_back(iCurS);
        currentTick = iCurTick;

    }

    int temp;

    int currentTick;
    std::vector<float> m_vecRobotX;
    std::vector<float> m_vecRobotY;
    std::vector<float> m_vecRobotZ;
    std::vector<int> m_vecRobotState;
    std::vector<int> m_vecRobotReading;
    std::vector<float> m_vecRobotSpeed;

};

/****************************************/
/****************************************/


void CMPGAEmergentBehaviorLoopFunctions::Init(TConfigurationNode &t_node) {

    // printErr("Statrted Init");
    //From the argos XML, get the number of robots we want to test with
    UInt32 iNumRobots;
    GetNodeAttribute(t_node, "num_robots", iNumRobots);

    //Create, locate, and place all the robots
    CreateRobots(iNumRobots);
    // printErr("Got numRobots");

    /*
     * Process trial information, if any
     * Not really used for us, unless you wish to trial a genome
     */
    try {
        UInt32 unTrial;
        GetNodeAttribute(t_node, "trial", unTrial);
        SetTrial(unTrial);
        Reset();
    } catch (CARGoSException &ex) {}
    // printErr("Fiished INIT");
}



/****************************************/
/****************************************/
//For whatever reason, if you don't put the genome back each time step, it defaults back to the 0.0 defaults.
void CMPGAEmergentBehaviorLoopFunctions::PreStep() {
    // PutGenome cPutGenome(m_pfControllerParams);
    // BuzzForeachVM(cPutGenome);
}
/****************************************/
/****************************************/

void CMPGAEmergentBehaviorLoopFunctions::PostStep() {
    GetStepRobotData cGetStepRobotData(0);
    BuzzForeachVM(cGetStepRobotData);

    PrintExperiment(std::string("experiment_" + ToString(::getpid()) + ".csv"), cGetStepRobotData.currentTick,
                    cGetStepRobotData.m_vecRobotX, cGetStepRobotData.m_vecRobotY,
                    cGetStepRobotData.m_vecRobotZ,
                    cGetStepRobotData.m_vecRobotState, cGetStepRobotData.m_vecRobotReading,
                    cGetStepRobotData.m_vecRobotSpeed);
}
/****************************************/
/****************************************/

//This is just a wrapper that takes the genome values supplied by MPGA and places them in the buzz script
void CMPGAEmergentBehaviorLoopFunctions::ConfigureFromGenome(const Real *pf_genome) {
    printErr("Started Config from Genome");
    /* Copy the genes into the NN parameter buffer */
    for (size_t i = 0; i < GENOME_SIZE; ++i) {
        m_pfControllerParams[i] = pf_genome[i];
    }
    PutGenome cPutGenome(m_pfControllerParams);
    BuzzForeachVM(cPutGenome);
    // printErr("finished config from genome");
}

/****************************************/
/****************************************/

CAnalysis::AnalysisResults CMPGAEmergentBehaviorLoopFunctions::AnalyzeSwarm(int swarmID,
        const std::vector<float> &vecRobotX,
        const std::vector<float> &vecRobotY,
        const std::vector<float> &vecRobotZ,
        const std::vector<float> &vecRobotSpeed,
        const std::vector<int> &vecRobotState) {

    std::vector<int> stateIndexes;
    for (int i = 0; i < vecRobotState.size(); ++i) {
        if (vecRobotState[i] == swarmID) {
            stateIndexes.push_back(i);
        }
    }

    std::vector<float> filteredRobotX;
    std::vector<float> filteredRobotY;
    std::vector<float> filteredRobotZ;
    std::vector<float> filteredRobotSpeed;
    std::vector<int> filteredRobotState;


    for(auto index : stateIndexes) {
        filteredRobotX.push_back(vecRobotX[index]);
        filteredRobotY.push_back(vecRobotY[index]);
        filteredRobotZ.push_back(vecRobotZ[index]);
        filteredRobotSpeed.push_back(vecRobotSpeed[index]);
        filteredRobotState.push_back(vecRobotState[index]);

    }

    CAnalysis analysis(filteredRobotX, filteredRobotY, filteredRobotZ, filteredRobotSpeed, filteredRobotState);
    return analysis.AnalyzeAll();
}

/****************************************/
/****************************************/

//Oh buddy
//it's been 4 months since I wrote this comment, and it's even more relevant now than then
Real CMPGAEmergentBehaviorLoopFunctions::Score() {
    std::vector<float> m_vecRobotX;
    std::vector<float> m_vecRobotY;
    std::vector<float> m_vecRobotZ;
    std::vector<int> m_vecRobotState;
    std::vector<int> m_vecRobotReading;
    std::vector<float> m_vecRobotSpeed;

    std::ifstream experimentFile(std::string("experiment_" + ToString(::getpid()) + ".csv"), std::ios::in);
    std::string line;

    if(experimentFile.is_open()) {
        unsigned long counter = 0;
        while (getline(experimentFile, line)) {
          if (counter >= 1200) {
            double robot[8]; //time, rid, X, Y, Z, State, Reading, Speed
            //Parse the line and store the values in scores array
            ParseValues(line, (UInt32) 8, robot, ',');
            m_vecRobotX.push_back((float) robot[2]);
            m_vecRobotY.push_back((float) robot[3]);
            m_vecRobotZ.push_back((float) robot[4]);
            m_vecRobotState.push_back((int) robot[5]);
            m_vecRobotReading.push_back((int) robot[6]);
            m_vecRobotSpeed.push_back((float) robot[7]);
          }

          counter++;
        }
    } else {
      return 0.0;
    }

    experimentFile.close();

    // cGetStepRobotData.currentTick,
    //RID
    //                 cGetStepRobotData.m_vecRobotX,
     // cGetStepRobotData.m_vecRobotY,
    //                 cGetStepRobotData.m_vecRobotZ,
    //                 cGetStepRobotData.m_vecRobotState,
    // cGetStepRobotData.m_vecRobotReading,
    //                 cGetStepRobotData.m_vecRobotSpeed
    //Create the Analysis class, with the vectors from the data
    CAnalysis fullSwarmAnalysis(m_vecRobotX, m_vecRobotY, m_vecRobotZ,
                                m_vecRobotSpeed, m_vecRobotState);


    //Analyze all, and save the results in a struct that has each value.
    CAnalysis::AnalysisResults fullSwarmResults = fullSwarmAnalysis.AnalyzeAll();

    CAnalysis::AnalysisResults swarm0Results = AnalyzeSwarm(0, m_vecRobotX, m_vecRobotY, m_vecRobotZ,
            m_vecRobotSpeed, m_vecRobotState);

    CAnalysis::AnalysisResults swarm1Results = AnalyzeSwarm(1, m_vecRobotX, m_vecRobotY, m_vecRobotZ,
            m_vecRobotSpeed, m_vecRobotState);
    //Open the master file as READ-ONLY. This is important, you can open a file from multiple locations read only, but if you try to open it to write things get fucky. Don't let them get fucky.
    std::ifstream score_files("master_scores.csv", std::ios::in);

    //Variables for reference later
    Real minDistance = 999999999999;
    bool needMinDist = true;

    //This makes sure we ignore the first value, which are the names of the csv columns
    bool skipped = false;

    //Confirm the file opened
    if (score_files.is_open()) {
        //While loop to iterate over each line. getLine will return 0 when there are no more lines, exiting the loop
        while (getline(score_files, line)) {
            if (skipped) {
                //Make and array the size of the genome + the size of the feature values for the whole swarm and each state + 1 for the score tacked onto the end
                double scores[42]; //Genomes, feature scores, +1 for the actual score that gets added, +1 again for the experiment identifier

                //Parse the line and store the values in scores array
                ParseValues(line, (UInt32) 42, scores, ',');

                //Pull out the values for easier reference
                Real comp_full_centroid_x = scores[GENOME_SIZE];
                Real comp_full_centroid_y = scores[GENOME_SIZE + 1];
                Real comp_full_scatter = scores[GENOME_SIZE + 2];
                Real comp_full_variance = scores[GENOME_SIZE + 3];
                Real comp_full_speed = scores[GENOME_SIZE + 4];
                Real comp_full_angMomentum = scores[GENOME_SIZE + 5];
                Real comp_full_groupRotation = scores[GENOME_SIZE + 6];
                Real comp_full_state_freq = scores[GENOME_SIZE + 7];


                Real comp_swarm0_centroid_x = scores[GENOME_SIZE + 8];
                Real comp_swarm0_centroid_y = scores[GENOME_SIZE + 9];
                Real comp_swarm0_scatter = scores[GENOME_SIZE + 10];
                Real comp_swarm0_variance = scores[GENOME_SIZE + 11];
                Real comp_swarm0_speed = scores[GENOME_SIZE + 12];
                Real comp_swarm0_angMomentum = scores[GENOME_SIZE + 13];
                Real comp_swarm0_groupRotation = scores[GENOME_SIZE + 14];


                Real comp_swarm1_centroid_x = scores[GENOME_SIZE + 15];
                Real comp_swarm1_centroid_y = scores[GENOME_SIZE + 16];
                Real comp_swarm1_scatter = scores[GENOME_SIZE + 17];
                Real comp_swarm1_variance = scores[GENOME_SIZE + 18];
                Real comp_swarm1_speed = scores[GENOME_SIZE + 19];
                Real comp_swarm1_angMomentum = scores[GENOME_SIZE + 20];
                Real comp_swarm1_groupRotation = scores[GENOME_SIZE + 21];



                //State difference is not calculated as part of the score
                //Same with the previous generations

                //Find the distance from basic Pythagorean method. Dist is the score

                Real dist1 = sqrt(handlenan(
                                pow(comp_full_centroid_x - fullSwarmResults.CentroidX, 2) +
                                pow(comp_full_centroid_y - fullSwarmResults.CentroidY, 2) +
                                pow(comp_full_scatter - fullSwarmResults.Scatter, 2) +
                                pow(comp_full_variance - fullSwarmResults.RadialVariance, 2) +
                                pow(comp_full_speed - fullSwarmResults.Speed, 2) +
                                pow(comp_full_angMomentum - fullSwarmResults.AngularMomentum, 2) +
                                pow(comp_full_groupRotation - fullSwarmResults.GroupRotation, 2) +
                                pow(comp_full_state_freq - fullSwarmResults.StateChangeFreq, 2)));

                Real dist2 = sqrt(handlenan(
                                pow(comp_swarm0_centroid_x - swarm0Results.CentroidX, 2) +
                                pow(comp_swarm0_centroid_y - swarm0Results.CentroidY, 2) +
                                pow(comp_swarm0_scatter - swarm0Results.Scatter, 2) +
                                pow(comp_swarm0_variance - swarm0Results.RadialVariance, 2) +
                                pow(comp_swarm0_speed - swarm0Results.Speed, 2) +
                                pow(comp_swarm0_angMomentum - swarm0Results.AngularMomentum, 2) +
                                pow(comp_swarm0_groupRotation - swarm0Results.GroupRotation, 2)));

                Real dist3 = sqrt(handlenan(
                                pow(comp_swarm1_centroid_x - swarm1Results.CentroidX, 2) +
                                pow(comp_swarm1_centroid_y - swarm1Results.CentroidY, 2) +
                                pow(comp_swarm1_scatter - swarm1Results.Scatter, 2) +
                                pow(comp_swarm1_variance - swarm1Results.RadialVariance, 2) +
                                pow(comp_swarm1_speed - swarm1Results.Speed, 2) +
                                pow(comp_swarm1_angMomentum - swarm1Results.AngularMomentum, 2) +
                                pow(comp_swarm1_groupRotation - swarm1Results.GroupRotation, 2)));

                Real dist = dist1 + dist2 + dist3;

                if(isnan(dist)){
                    dist = 0.0;
                }

                if (needMinDist) {
                    needMinDist = false;
                    minDistance = dist;
                }

                //We want to find the shortest distance
                if (dist < minDistance) {
                    minDistance = dist;
                }
            } else {
                skipped = true;
            }
        }
    } else {
        minDistance = 0;
    }
    // printErr("Finished Scoring");

    //Close the master score file. VERY IMPORTANT
    score_files.close();


    // printErr("Flushing genomes to individual files");

    //Open the file for that genome.
    std::ofstream cScoreFile(std::string("score_" + ToString(::getpid()) + ".csv").c_str(),
                             std::ios::out | std::ios::trunc);

    //Confirm the file is open
    if (cScoreFile.is_open()) {
        //Output each genome value
        for (auto val : m_pfControllerParams) {
            cScoreFile << val << ',';
        }

        //Output the feature values and the score (minDistance)
        cScoreFile
                << fullSwarmResults.CentroidX << ','
                << fullSwarmResults.CentroidY << ','
                << fullSwarmResults.Scatter << ','
                << fullSwarmResults.RadialVariance << ','
                << fullSwarmResults.Speed << ','
                << fullSwarmResults.AngularMomentum << ','
                << fullSwarmResults.GroupRotation << ','
                << fullSwarmResults.StateChangeFreq << ','

                << swarm0Results.CentroidX << ','
                << swarm0Results.CentroidY << ','
                << swarm0Results.Scatter << ','
                << swarm0Results.RadialVariance << ','
                << swarm0Results.Speed << ','
                << swarm0Results.AngularMomentum << ','
                << swarm0Results.GroupRotation << ','


                << swarm1Results.CentroidX << ','
                << swarm1Results.CentroidY << ','
                << swarm1Results.Scatter << ','
                << swarm1Results.RadialVariance << ','
                << swarm1Results.Speed << ','
                << swarm1Results.AngularMomentum << ','
                << swarm1Results.GroupRotation << ','


                << minDistance << std::endl;
    } else {
        //panic
    }
    //Close the score file
    cScoreFile.close();

    return minDistance;
}

/****************************************/
/****************************************/
void CMPGAEmergentBehaviorLoopFunctions::CreateRobots(UInt32 un_robots) {
    //The angular gap between each robot. Dependent on the number of robots
    CRadians robStep = CRadians::TWO_PI / static_cast<Real>(un_robots);
    CRadians cOrient;

    //for each robot, calculate the position based on spherical coordinates
    for (size_t i = 0; i < un_robots; ++i) {

        CRange<Real> distRange;
        distRange.SetMin(-1);
        distRange.SetMax(1);
        Real dist_x = m_pcRNG->Uniform(distRange);
        Real dist_y = m_pcRNG->Uniform(distRange);
        CVector3 pos = CVector3(dist_x), dist_y, 0.0);

        //Randomly choose the starting orientation
        CQuaternion head;
        cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
        head.FromEulerAngles(
            cOrient,        // rotation around Z
            CRadians::ZERO, // rotation around Y
            CRadians::ZERO  // rotation around X
        );
        /* Create robot */
        CKheperaIVEntity *pcRobot = new CKheperaIVEntity(
            "kh" + ToString(i),
            KH_CONTROLLER,
            pos,
            head,
            KH_COMMRANGE,
            KH_DATASIZE);
        /* Add it to the simulation */
        AddEntity(*pcRobot);
        /* Add it to the internal lists */
        m_vecKheperas.push_back(pcRobot);
        m_vecControllers.push_back(

            &dynamic_cast<CBuzzController &>(
                pcRobot->GetControllableEntity().GetController()));


        SInitSetup str;
        str.Position = pos;
        str.Orientation = head;
        m_vecInitSetup.push_back(str);
    }
    BuzzRegisterVMs();
}

/****************************************/
/****************************************/

void CMPGAEmergentBehaviorLoopFunctions::Reset() {
    /*
     * Move robot to the initial position corresponding to the current trial
     */
    // printErr("Started Reset");

    //For each robot, check to see if it moved, if it had, put it back. IF it errors out, print the robot and where we tried to move it to
    for(size_t i = 0; i < m_vecKheperas.size(); i++) {
        if(!MoveEntity(
                    m_vecKheperas[i]->GetEmbodiedEntity(),        //Move this robot
                    m_vecInitSetup[i].Position,          // with this position
                    m_vecInitSetup[i].Orientation,       // with this orientation
                    false                                         // this is not a check, so actually move the robot back
                )) {
            LOGERR << "Can't move robot kh(" << i << ") in <"
                   << m_vecInitSetup[i].Position
                   << ">, <"
                   << m_vecInitSetup[i].Orientation
                   << ">"
                   << std::endl;
        }
    }
    // printErr("Finished Reset");
}

//Easy wrapper for printing to the error log so when you get a "Blah failed. Check ARGoS_LOGERR_#### there's actually something useful there"
void CMPGAEmergentBehaviorLoopFunctions::printErr(std::string in) {
    LOGERR << in << std::endl;
    LOGERR.Flush();
}

void CMPGAEmergentBehaviorLoopFunctions::PrintExperiment(std::string filename, int currentTick,
        std::vector<float> m_vecRobotX, std::vector<float> m_vecRobotY,
        std::vector<float> m_vecRobotZ,
        std::vector<int> m_vecRobotState,
        std::vector<int> m_vecRobotReading,
        std::vector<float> m_vecRobotSpeed) {

    unsigned long size = m_vecRobotX.size();

    std::ofstream experimentFile(filename, std::ios::out | std::ios::app);

    for (int i = 0; i < size; ++i) {
        experimentFile << currentTick << "," << i << "," << m_vecRobotX[i] << "," << m_vecRobotY[i] << ","
                       << m_vecRobotZ[i] << ","
                       << m_vecRobotState[i]
                       << "," << m_vecRobotReading[i] << "," << m_vecRobotSpeed[i] << std::endl;
    }

    experimentFile.close();

}

double CMPGAEmergentBehaviorLoopFunctions::handlenan(double d){
    if (isnan(d))
    {
        return 0.0;
    }
    return d;
}

//Register the loop functions so buzz and ARGoS can find it
REGISTER_LOOP_FUNCTIONS(CMPGAEmergentBehaviorLoopFunctions, "mpga_emergent_behavior_loop_functions")
