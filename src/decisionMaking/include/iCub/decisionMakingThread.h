// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
    * Copyright: (C) 2022 Istituto Italiano di Tecnologia | Robotics, Brains and Cognitive Science
    * Authors: Letícia Berto
    * Email: leticia.maraberto@iit.it
    * Permission is granted to copy, distribute, and/or modify this program
    * under the terms of the GNU General Public License, version 2 or any
    * later version published by the Free Software Foundation.
    *
    * A copy of the license can be found at
    * http://www.robotcub.org/icub/license/gpl.txt
    *
    * This program is distributed in the hope that it will be useful, but
    * WITHOUT ANY WARRANTY; without even the implied warranty of
    * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
    * Public License for more details
*/

/**
 * @file decisionMakingThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _DECISIONMAKING_PERIODTHREAD_H_
#define _DECISIONMAKING_PERIODTHREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <cstring>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <random>
#include "iCub/approximateQAgent.h"
#include <string.h>

//enum robotState {initial, idle, interact, recharge, powerOff, play, endInteraction};
//change var "numberOfStatesToDesconsider" if necessary
enum robotState {idle, interact, recharge, play, lookDown, initial, endInteraction};//endInteraction MUST be the last one (used to set the RL total behaviors)

#define TIME_HOME_ARE       8
#define TIME_POINT_ARE      12
#define TIME_GAZE           1
#define TIME_CTP            2
#define TIME_LEDS           1
#define TIME_SPEECH         3
#define TIME_RECHARGE       6

#define LEARNING_PHASE      "learn"
#define TESTING_PHASE       "evaluate"
#define FINETUNING_PHASE    "finetuning"
#define RULE_BASED          "rulebased"
#define DRIVE_BASED        "drivebased"

#define SOCIAL_PROFILE  "social"
#define PLAYFUL_PROFILE "playful"
#define REGULAR_PROFILE "regular"

#define EGREEDY_DECAY_LINEAR         "linear"
#define EGREEDY_DECAY_EXPONENTIAL    "exponential"
#define EGREEDY_DECAY_CONSTANT       "constant"

//Used for the rule based agent
#define LOWERBOUND              "lower"
#define MAXVALUE                "max"

#define UNDEF -999

class decisionMakingThread : public yarp::os::PeriodicThread {
private:

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    std::string name;

    yarp::os::ResourceFinder rf;

    std::string robot_color;        //Used to not consider the robot's arm as an object to interact
    std::string robot_profile;
    std::string robot_name;
    
    //Save data
    std::string filepath;
    std::string filenameAllData = "decisionMaking.csv";
    std::string fileHeaderAllData = "time,durationExp,exploreDrive,affectDrive,surviveDrive,survive_min_threshold,affect_min_threshold,explore_min_threshold,objColor,decision,saturationInteraction,episode,step";
    std::string filenameRewards = "rewards.csv";
    std::string fileHeaderRewards = "episode,reward";

    time_t now;
    std::string timeNow;
    std::time_t timeInitial;

    std::time_t timeOfAction;
    float durationOfAction;

    time_t timeInitExperiment;
    int durationOfExperiment = 300;//600;   //time in seconds (so, 10 minutes)

    //Define what boundary to use to compute the threshold in the rule based agent. It will be a porcentage of the max value the need can achieve or porcentage of the lower boundary
    std::string boundary = MAXVALUE; //LOWERBOUND or MAXVALUE (default)
    float survive_min_threshold, affect_min_threshold, explore_min_threshold;

    bool first;
    robotState behavior, previousBehavior;
    int actionRL;

    const int numberOfFixedObjs = 5;

    enum bodyPart {torso, leftHand, leftForearm, leftUpper, rightHand, rightForearm, rightUpper, noTouch};
    enum bodySide {torsoS, leftS, rightS, noTouchS};

    //std::string affectInput;
    //double focusX, focusY;
    int originOfTouch, sideOfTouch;
    float touch_current, face_current, gaze_current;

    double surviveDrive, affectDrive, boredomDrive;
    float last_affectDrive, last_boredomDrive;
    int waitingInteraction, saturedAffect;
    int timeToWhistle;

    bool saturationInteraction;

    bool noBattery;

    //yarp::os::Bottle specificObject;                                                       
    yarp::os::Bottle* allObjsSeen;
    std::string colorObj;  
    int numberOfObjectsScene;
    int indexRobotAsObject;

    std::string soundsPlay[24] = {"Wow!", "#@U#", "e-be","#MMM01#", "#D#", "u-uu", "do-do", "#d#", "#E#", "#U#",  "#u#", "#V#", "#MMM02#", "#v#", "a-da", "#b#", "Yippee!", "#g#", "#i#", "pii po", "#j#", "#t_h#", "#w#", "#z#"};
    std::string soundsInteract[2] = {"#EI#", "#CRY04#"};
    int indexSoundPlay, indexSoundInteract, indexInteractionReply;
    int objectsSequence[20] = {3, 5, 1, 4, 2, 3, 1, 4, 2, 5, 3, 2, 4, 5, 1, 3, 5, 4, 1, 2};
    int indexSeq_objToPlay;

    std::mt19937 rdx{static_cast<long unsigned int>(21)};
    
    //RL vars
    approximateQAgent *QL_agent;
    double alpha;               //learning rate
    double gamma;               //discount factor
    double epsilon;             //exploration probability at start
    double epsilon_min;         //minimum exploration probability
    double epsilon_decay;       //exponential decay rate for exploration prob
    std::string eGreedyDecay;   //EGREEDY_DECAY_LINEAR, EGREEDY_DECAY_EXPONENTIAL, EGREEDY_DECAY_CONSTANT
    std::string mode;          //LEARNING_PHASE, TESTING_PHASE, FINETUNING_PHASE, RULE_BASED
    double episode_rewards;
    int total_episodes;
    int current_episode;
    int steps;
    int maxSteps;
    bool fim = true;
    
    const int deathPunishment = -100;

    bool endTest;
    const int numberOfStatesToDesconsider = 2;  //Some states (robot's behavior) are used in robotState to provide a better HRI but they are not used in RL
    const int features = 7;
    const int useLastBehavior = 1;      //Use the last behavior as part of the state in the MDP: 1 - yes, 0 - no
    const int previousStatesToRepeat = 2;
    double *featuresToMDP = new double[features];
    int waitBufferPerception;           //counter to wait readings from perception to fill the previousStatesToRepeat in the MDP 
    double *allRewardsTraining;

    //-------------------------Values are defined in the configuration file
    double MAX_BOREDOM;
    double MIN_BOREDOM;

    double MIN_COMFORT;
    double MAX_COMFORT;

    double MIN_BATTERY;
    double MAX_BATTERY;

    double SURVIVAL_HOMEOSTASIS;
    double AFFECT_HOMEOSTASIS;
    double EXPLORE_HOMEOSTASIS;

    //Range to define the lowerbound (H - range) and the upperbound (H + range) around the homeostasis threshold
    int RANGE_EXPLORE;
    int RANGE_AFFECT;
    int RANGE_SURVIVE;

    double PERCEN_THRE;

    //Input from Motivation Module
    yarp::os::BufferedPort<yarp::os::Bottle> inputPortBoredomDriveObject;
    yarp::os::BufferedPort<yarp::os::Bottle> inputSurvivalDrive;
    yarp::os::BufferedPort<yarp::os::Bottle> inputAffectDrive;

    //Input from Perception Module
    yarp::os::BufferedPort<yarp::os::Bottle> inputAllObjectsPerceived;
    yarp::os::BufferedPort<yarp::os::Bottle> inputSkinPerceivedPort;                
    //yarp::os::BufferedPort<yarp::os::Bottle> inputAffectPerceivedPort;

    yarp::os::BufferedPort<yarp::os::Bottle> inputInteractionPort;              //read face_current, gaze_current, touch_current

    //Input from Battery Sensor Module
    yarp::os::BufferedPort<yarp::os::Bottle> inputNoBatteryPort; 

    //Output to Action Module
    yarp::os::BufferedPort<yarp::os::Bottle> outputActionPort;
    
    yarp::os::BufferedPort<yarp::os::Bottle> outputUpdateBoredomPort;
    yarp::os::BufferedPort<yarp::os::Bottle> outputUpdBatteryConsPort;
    yarp::os::BufferedPort<yarp::os::Bottle> ouputStopApplication;
    yarp::os::BufferedPort<yarp::os::Bottle> outputPortResetBattery;
    yarp::os::BufferedPort<yarp::os::Bottle> outputPortResetBoredom;
    yarp::os::BufferedPort<yarp::os::Bottle> outputPortResetWorld;        //reset simulator when the episode finishes in RL
    yarp::os::BufferedPort<yarp::os::Bottle> outputPortEndInitialBehavior;

    yarp::os::BufferedPort<yarp::os::Bottle> outputPortSaturetedAffect;

    //Ports used to debug the behavior of the needs (Boredom, Comfort)
    yarp::os::BufferedPort<yarp::os::Bottle> inputComfortPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inputBoredomPort;
    
public:
    /**
    * constructor default
    */
    decisionMakingThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    decisionMakingThread(std::string robotname,std::string configFile);

    //decisionMakingThread(std::string _robot, std::string _configFile, std::string _robot_color);
    decisionMakingThread(std::string _robot, yarp::os::ResourceFinder &_rf, std::string _robot_color, std::string robot_profile);
    /**
     * destructor
     */
    ~decisionMakingThread();

    /**
    *  initialises the thread
    */
    bool threadInit();

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run();

    /**
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

    /**
     * method for the processing in the ratethread
     **/
    bool processing();

    bool openAllPorts();
    bool waitForPortConnections();
    void initAllVars();

    void getData();
    void readBoredomDriveData();
    void readBatteryDriveData();
    void readAffectDriveData();
    void readSkinPerceived();
    void readAffectPerceived();
    void readObjectsPerceived();

    bool desconsiderRobotColorAsObject();

    void checkIfNoBattery();

    std::string lookAtTouchedPart(int originOfTouch);

    yarp::os::Bottle getObjectCoords(std::string colorToLookAt);

    yarp::os::Bottle getRandomObjectToPlay();
    void definePlay(yarp::os::Bottle specificObjectData);

    void playFixedObjects_CTP();
    void playDynamicObjects_ARE();

    std::string lookAtSpecificObject(int i);

    void updateBoredom();

    void detailBehaviorActions(robotState currentState);
    void detailBehaviorActions_NoLearning(robotState behavior);

    void writeCommand(std::string commandSM);

    void updateBatteryConsumption(std::string behavior);

    void saveData(robotState behavior);
    void saveHeaders();
    void saveRewards();

    void featuresToRL();
    void updateToRL();
    void getNextAction();
    
    //Methods to communicate with RL class
    bool setQL();
    bool endExperiment();
    bool endEpisode();
    void composeFeatures();
    double rewardFunction(double surviveDrive, double affectDrive, double boredomDrive);

    void reset();
    void saveTrainingData();

    void makeDecision_RuleBased();
    void makeDecision_RL();
    void makeDecision_DriveBased();

    void initVarsFromFile();
    void loadRLVars();
    void loadRBVars();
    void printData();

};

#endif  //_DECISIONMAKING_PERIODTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------