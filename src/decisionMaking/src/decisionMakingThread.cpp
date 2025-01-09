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
 * @file decisionMakingThread.cpp
 * @brief Implementation of the eventDriven thread (see decisionMakingThread.h).
 */

#include <iCub/decisionMakingThread.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace boost;

#define THPERIOD 0.5//s
#define NON_EXIST -1 //Must be the same value defined in objectPerception module

decisionMakingThread::decisionMakingThread():PeriodicThread(THPERIOD) {
    robot = "icub";        
}

decisionMakingThread::decisionMakingThread(string _robot, string _configFile):PeriodicThread(THPERIOD){
    robot = _robot;
    configFile = _configFile;
}

decisionMakingThread::decisionMakingThread(string _robot, ResourceFinder &_rf, string _robot_color, string _robot_profile):PeriodicThread(THPERIOD){
    robot = _robot;
    rf = _rf;
    robot_color = _robot_color;
    robot_profile = _robot_profile;
}

decisionMakingThread::~decisionMakingThread() {
    delete QL_agent;
    delete [] featuresToMDP;
    delete [] allRewardsTraining;
}

void decisionMakingThread::setName(string str) {
    this->name=str;
}


std::string decisionMakingThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void decisionMakingThread::setInputPortName(string InpPort) {
    
}

bool decisionMakingThread::openAllPorts(){
    if(!inputNoBatteryPort.open(getName("/noBattery:i").c_str())){
        yError("unable to open port to send unmasked events ");
        return false;
    }

    if(!inputAllObjectsPerceived.open(getName("/allObjsSeen:i").c_str())){
        yError("unable to open port to send unmasked events ");
        return false;
    }

    if(!inputInteractionPort.open(getName("/gazeFaceSkin:i").c_str())){
        yError("unable to open port to receive evaluation of interaction data");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!inputPortBoredomDriveObject.open(getName("/exploreDriveObject:i").c_str())){
        yError("unable to open port to send unmasked events ");
        return false;
    }

    if(!inputSurvivalDrive.open(getName("/survivalDrive:i").c_str())){
        yError("unable to open port to receive output");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!inputAffectDrive.open(getName("/affectDrive:i").c_str())){
        yError("unable to open port to receive output");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!inputSkinPerceivedPort.open(getName("/dataSkin:i").c_str())){
        yError("unable to open port to receive output");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    /*if(!inputAffectPerceivedPort.open(getName("/dataAffect:i").c_str())){
        yError("unable to open port to receive output");
        return false;  // unable to open; let RFModule know so that it won't run
    }*/

    if(!outputActionPort.open(getName("/commandSM:o").c_str())){
        yError("unable to open port to receive output");
        return false;
    }

    /* if(!ouputStopApplication.open(getName("/stopApplication:o").c_str())){
        yError("unable to open port to receive output");
        return false;
    } */

    if(!outputUpdateBoredomPort.open(getName("/updateBoredom:o").c_str())){
        yError("unable to open port to receive output");
        return false;
    }

    if(!outputUpdBatteryConsPort.open(getName("/updateBatteryConsumption:o").c_str())){
        yError("unable to open port to receive output");
        return false;
    }
    
    if(!outputPortSaturetedAffect.open(getName("/saturetedAffectAct:o").c_str())){
        yError("unable to open port to receive output");
        return false;
    }

    if(mode == LEARNING_PHASE || mode == TESTING_PHASE || mode == FINETUNING_PHASE){
        if(!outputPortResetBattery.open(getName("/resetBattery:o").c_str())){
                yError("unable to open port to receive output");
                return false;
            }

            if(!outputPortResetBoredom.open(getName("/resetBoredomComfort:o").c_str())){
                yError("unable to open port to receive output");
                return false;
            }

            if(!outputPortResetWorld.open(getName("/resetWorld:o").c_str())){
                yError("unable to open port to receive output");
                return false;
            }
    }
    
    if(mode == RULE_BASED || mode == DRIVE_BASED){
        if(!outputPortEndInitialBehavior.open(getName("/startDrivesComputation:o").c_str())){
            yError("unable to open port to receive output");
            return false;  // unable to open; let RFModule know so that it won't run
        }
    }

    //######### Just for debugging #########
    if(!inputBoredomPort.open(getName("/boredom:i").c_str())){
        yError("unable to open port to receive output");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!inputComfortPort.open(getName("/comfort:i").c_str())){
        yError("unable to open port to receive output");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    return true;
}

bool decisionMakingThread::waitForPortConnections(){
    while(inputInteractionPort.getInputCount() < 1);
    while(inputNoBatteryPort.getInputCount() < 1);
    while(inputPortBoredomDriveObject.getInputCount() < 1);
    while(inputSurvivalDrive.getInputCount() < 1);
    while(inputAffectDrive.getInputCount() < 1);
    while(inputAllObjectsPerceived.getInputCount() < 1);
    while(inputSkinPerceivedPort.getInputCount() < 1);
    //while(inputAffectPerceivedPort.getInputCount() < 1);
    while(outputActionPort.getOutputCount() < 1);
    while(outputUpdateBoredomPort.getOutputCount() < 1);
    while(outputUpdBatteryConsPort.getOutputCount() < 1);

    if(mode == LEARNING_PHASE || mode == TESTING_PHASE || mode == FINETUNING_PHASE){
        while(outputPortResetBattery.getOutputCount() < 1);
        while(outputPortResetBoredom.getOutputCount() < 1);
    }

   // while(ouputStopApplication.getOutputCount() < 5);//one for each module that I want to stop (and it is defined in XML)

    return true;
}

bool decisionMakingThread::setQL(){
    // QLearning initialization
    QL_agent = new approximateQAgent(alpha, gamma, epsilon, epsilon_min, epsilon_decay, eGreedyDecay, total_episodes);
    QL_agent->setFilenamePath(filepath);

    //endInteraction + 1: returns the total behaviors that the robot can execute.
    //endInteraction + 1 - 2: the "initial" and "endInteraction" are not used on the qlearning algorithm, but they are used to have a nice HRI
    QL_agent->setTotalBehaviors(endInteraction + 1 - numberOfStatesToDesconsider); //Simplified equation explained above

    QL_agent->setNumberOfFeatures(features, previousStatesToRepeat, useLastBehavior);
    
    if(mode.compare(LEARNING_PHASE) == 0){ //Training phase
        QL_agent->init_featuresWeight();
    }else if(mode.compare(FINETUNING_PHASE) == 0){//Fine-tuning phase
        if(!QL_agent->recoverWeights())
            return false;
        //Check initial value for epsilon (e-greedy policy to action selection)
    }else{ //Test phase
        if(!QL_agent->recoverWeights())
            return false;
        QL_agent->setEpsilon(0);
    }

    return true;
}

void decisionMakingThread::initVarsFromFile(){
    MIN_BATTERY = rf.findGroup("variables").find("MIN_BATTERY").asFloat32();
    MAX_BATTERY = rf.findGroup("variables").find("MAX_BATTERY").asFloat32();
    MIN_COMFORT = rf.findGroup("variables").find("MIN_COMFORT").asFloat32();
    MAX_COMFORT = rf.findGroup("variables").find("MAX_COMFORT").asFloat32();
    MIN_BOREDOM = rf.findGroup("variables").find("MIN_BOREDOM").asFloat32();
    MAX_BOREDOM = rf.findGroup("variables").find("MAX_BOREDOM").asFloat32();

    SURVIVAL_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_ENERGY").asFloat32();

    if(robot_profile.compare(SOCIAL_PROFILE) == 0){
        AFFECT_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_AFFECT_SOCIAL").asFloat32();
        EXPLORE_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_BOREDOM_SOCIAL").asFloat32();
        robot_name = "Berry";//"Berry";
    }else if(robot_profile.compare(PLAYFUL_PROFILE) == 0){
        AFFECT_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_AFFECT_PLAYFUL").asFloat32();
        EXPLORE_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_BOREDOM_PLAYFUL").asFloat32();
        robot_name = "Berry";//"Reddy";
    }else{//REGULAR_PROFILE
        AFFECT_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_REGULAR").asFloat32();
        EXPLORE_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_REGULAR").asFloat32();
        robot_name = "Berry";//"icub";
    }

    RANGE_SURVIVE = rf.findGroup("variables").find("RANGE_SURVIVE").asInt32();
    RANGE_AFFECT = rf.findGroup("variables").find("RANGE_AFFECT").asInt32();
    RANGE_EXPLORE = rf.findGroup("variables").find("RANGE_EXPLORE").asInt32();

    mode = rf.findGroup("variables").find("mode").asString();
    if(mode.compare(LEARNING_PHASE) == 0)
        mode = LEARNING_PHASE;
    else if(mode.compare(TESTING_PHASE) == 0)
        mode = TESTING_PHASE;
    else if(mode.compare(FINETUNING_PHASE) == 0)
        mode = FINETUNING_PHASE;
    else if(mode.compare(RULE_BASED) == 0)
        mode = RULE_BASED;
    else
        mode = DRIVE_BASED;

    if(mode.compare(RULE_BASED) == 0 || mode.compare(DRIVE_BASED) == 0)
        durationOfExperiment = rf.check("durationOfExperiment", Value(600)).asInt32();
}

//Load from file the variables values specific to the Rule Based agent
void decisionMakingThread::loadRBVars(){
    PERCEN_THRE = rf.findGroup("variables").find("PERCEN_THRE").asFloat32();

    boundary = rf.check("boundary", Value("max")).asString();
    if(boundary.compare(MAXVALUE) == 0)
        boundary = MAXVALUE;
    else
        boundary = LOWERBOUND;

    //Rule to define the threshold to activate the related behavior
    if(boundary == LOWERBOUND){//Lower bound of homeostasis value -- percentage far from the homeostasis (Ex: when battery is 10% far from homeostasis)
        survive_min_threshold = -(((SURVIVAL_HOMEOSTASIS - RANGE_SURVIVE) - MIN_BATTERY) * PERCEN_THRE);
        affect_min_threshold = -(((AFFECT_HOMEOSTASIS - RANGE_AFFECT) - MIN_COMFORT) * PERCEN_THRE);
        explore_min_threshold = (MAX_BOREDOM - (EXPLORE_HOMEOSTASIS + RANGE_EXPLORE)) * PERCEN_THRE;
    }else{//Max value the need reach (Ex: when drive < 10% of the TOTAL battery, recharge)
        survive_min_threshold = -((MIN_BATTERY + (SURVIVAL_HOMEOSTASIS - RANGE_SURVIVE)) - (MIN_BATTERY + (PERCEN_THRE * (MAX_BATTERY - MIN_BATTERY) + MIN_BATTERY)));
        affect_min_threshold = -((MIN_COMFORT + (AFFECT_HOMEOSTASIS - RANGE_AFFECT)) - (MIN_COMFORT + (PERCEN_THRE * (MAX_COMFORT - MIN_COMFORT) + MIN_COMFORT)));
        explore_min_threshold = -((MAX_BOREDOM - (EXPLORE_HOMEOSTASIS + RANGE_EXPLORE)) - (PERCEN_THRE * (MAX_BOREDOM - MIN_BOREDOM)));
    }

    cout<<"durationOfExperiment: "<<durationOfExperiment<<endl;
    cout<<"PERCEN_THRE: "<<PERCEN_THRE<<endl;
    cout<<"survive_min_threshold: "<<survive_min_threshold<<endl;
    cout<<"affect_min_threshold: "<<affect_min_threshold<<endl;
    cout<<"explore_min_threshold: "<<explore_min_threshold<<endl;
}

//Load from file the variables values specific to the Reinforcement Learning agent
void decisionMakingThread::loadRLVars(){
    allRewardsTraining =  new double[total_episodes];
    for(int i = 0; i < total_episodes; i++)
        allRewardsTraining[i] = 0.0;

    alpha = rf.check("alpha", Value(0.0001)).asFloat64();
    gamma = rf.check("gamma", Value(0.9)).asFloat64();
    epsilon = rf.check("epsilon", Value(1.0)).asFloat64();
    epsilon_min = rf.check("epsilon_min", Value(0.01)).asFloat64();
    epsilon_decay = rf.check("epsilon_decay", Value(0.0003)).asFloat64();
    eGreedyDecay = rf.check("EGREEDY", Value("linear")).asString();
    total_episodes = rf.check("total_episodes", Value(15)).asInt32();
    maxSteps = rf.check("maxSteps", Value(20)).asInt32();

    if(eGreedyDecay.compare(EGREEDY_DECAY_LINEAR) == 0)
        eGreedyDecay = EGREEDY_DECAY_LINEAR;
    else if(eGreedyDecay.compare(EGREEDY_DECAY_CONSTANT) == 0)
        eGreedyDecay = EGREEDY_DECAY_CONSTANT;
    else
        eGreedyDecay = EGREEDY_DECAY_EXPONENTIAL;

    cout<<"alpha: "<<alpha<<endl;
    cout<<"gamma: "<<gamma<<endl;
    cout<<"epsilon: "<<epsilon<<endl;
    cout<<"epsilon_min: "<<epsilon_min<<endl;
    cout<<"epsilon_decay: "<<epsilon_decay<<endl;
    cout<<"eGreedyDecay: "<<eGreedyDecay<<endl;
    cout<<"total_episodes: "<<total_episodes<<endl;
    cout<<"maxSteps: "<<maxSteps<<endl;
}

void decisionMakingThread::printData(){
    cout<<"Running with:"<<endl;
    cout<<"Robot color: "<<robot_color<<endl;
    cout<<"Robot profile: "<<robot_profile<<endl;
    cout<<"mode: "<<mode<<endl;
    cout<<"MIN_BATTERY: "<<MIN_BATTERY<<endl;
    cout<<"MAX_BATTERY: "<<MAX_BATTERY<<endl;
    cout<<"MIN_COMFORT: "<<MIN_COMFORT<<endl;
    cout<<"MAX_COMFORT: "<<MAX_COMFORT<<endl;
    cout<<"MIN_BOREDOM: "<<MIN_BOREDOM<<endl;
    cout<<"MAX_BOREDOM: "<<MAX_BOREDOM<<endl;

    cout<<"RANGE_SURVIVE: "<<RANGE_SURVIVE<<endl;
    cout<<"RANGE_AFFECT: "<<RANGE_AFFECT<<endl;
    cout<<"RANGE_EXPLORE: "<<RANGE_EXPLORE<<endl;

    cout<<"SURVIVAL_HOMEOSTASIS: "<<SURVIVAL_HOMEOSTASIS<<endl;
    cout<<"AFFECT_HOMEOSTASIS: "<<AFFECT_HOMEOSTASIS<<endl;
    cout<<"EXPLORE_HOMEOSTASIS: "<<EXPLORE_HOMEOSTASIS<<endl;
}

void decisionMakingThread::initAllVars(){
    first = true;
    behavior = initial;
    previousBehavior = behavior;

    allObjsSeen = nullptr;
    numberOfObjectsScene = 0;
    indexRobotAsObject = -2;

    indexSeq_objToPlay = 0;
    indexSoundPlay = 0;
    indexSoundInteract = 0;
    indexInteractionReply = 0;
    saturedAffect = 0;
    saturationInteraction = false;

    timeToWhistle = 0;

    waitingInteraction = 0;

    timeOfAction = Time::now();
    durationOfAction = 0;

    current_episode = 0;
    steps = 0;
    endTest = false;

    waitBufferPerception = 0;
    
    touch_current = 0.0;
    face_current = 0.0;
    gaze_current = 0.0;

    survive_min_threshold = UNDEF;
    affect_min_threshold = UNDEF;
    explore_min_threshold = UNDEF;

    initVarsFromFile();

    SURVIVAL_HOMEOSTASIS = SURVIVAL_HOMEOSTASIS * (MAX_BATTERY - MIN_BATTERY) + MIN_BATTERY;
    AFFECT_HOMEOSTASIS = AFFECT_HOMEOSTASIS * (MAX_COMFORT - MIN_COMFORT) + MIN_COMFORT;
    EXPLORE_HOMEOSTASIS = MAX_BOREDOM - (MAX_BOREDOM - MIN_BOREDOM) * EXPLORE_HOMEOSTASIS;

    printData();
}

bool decisionMakingThread::threadInit(){
    filepath = rf.find("filepath").asString();
    filenameAllData = filepath + filenameAllData;
    filenameRewards = filepath + filenameRewards;
    
    initAllVars();

    if(mode.compare(LEARNING_PHASE) == 0 || mode.compare(FINETUNING_PHASE) == 0 || mode.compare(TESTING_PHASE) == 0){
        loadRLVars();
        if(!setQL())
            return false;
    }else if(mode.compare(RULE_BASED) == 0)
        loadRBVars();

    saveHeaders();

    if(!openAllPorts())
        return false;
    
    yInfo("Initialization of the processing thread correctly ended");

    yInfo("Wait for connections");
    if(!waitForPortConnections()){
        yError("Connection Error");
        return false;
    }

    timeInitial = Time::now();
    timeInitExperiment = Time::now();//Here is "fake" because we start to really count the time after the initial state (except for the RL agent)

    return true;
}

void decisionMakingThread::readBatteryDriveData(){
    if(inputSurvivalDrive.getInputCount()){
        Bottle* inpuSurviveDriveB = inputSurvivalDrive.read(true);
        surviveDrive = inpuSurviveDriveB->get(0).asFloat64();
        cout<<"surviveDrive: "<<surviveDrive<<endl;
    }
}

void decisionMakingThread::readAffectDriveData(){
   if(inputAffectDrive.getInputCount()){
        Bottle* inputAffectDriveB = inputAffectDrive.read(true);
        affectDrive = inputAffectDriveB->get(0).asFloat64();
        cout<<"affectDrive: "<<affectDrive<<endl;
    }
}

void decisionMakingThread::readBoredomDriveData(){
    if(inputPortBoredomDriveObject.getInputCount()){
        Bottle* boredomDriveObjData = inputPortBoredomDriveObject.read(true);
        boredomDrive = boredomDriveObjData->get(0).asFloat64();
        colorObj = boredomDriveObjData->get(1).asString();//toSTring();
        cout<<"Boredom Drive: "<<boredomDrive<<" next objChoosen: "<<colorObj<<endl;
    }
}

Bottle decisionMakingThread::getObjectCoords(string colorToLookAt){
    Bottle specificObject;
    specificObject.clear();
    if(allObjsSeen->size() > 0)
        cout<<"#objects: "<<allObjsSeen->get(0).asInt16()<<endl;
    for(int i = 0; i < allObjsSeen->get(0).asInt16(); i++){
        if(allObjsSeen->get(i+1).asList()->get(8).asString().compare(colorToLookAt) == 0){
            specificObject.add(allObjsSeen->get(i+1));
            cout<<"The object choosen by the motivation is in my FOV"<<endl;
            break;
        }
    }
    return specificObject;
}

void decisionMakingThread::readSkinPerceived(){
    originOfTouch = noTouch;
    if(inputSkinPerceivedPort.getInputCount()){
        Bottle* data = inputSkinPerceivedPort.read(false);
        if(data != nullptr){
            originOfTouch = data->get(0).asInt16();
            sideOfTouch = data->get(1).asInt16();
        }
    }
}

void decisionMakingThread::readAffectPerceived(){
    if(inputInteractionPort.getInputCount()){
        Bottle* input = inputInteractionPort.read(false);
        if(input != nullptr){
            face_current = input->get(0).asFloat64();
            gaze_current = input->get(1).asFloat64();
            touch_current = input->get(2).asFloat64();
        }
    }
    
    /*if(inputAffectPerceivedPort.getInputCount()){
        Bottle* data = inputAffectPerceivedPort.read(true);
        affectInput = data->get(0).asString();
        focusX = data->get(1).asFloat64();
        focusY = data->get(2).asFloat64();
    }*/
}

void decisionMakingThread::readObjectsPerceived(){
    if(allObjsSeen != nullptr)
        allObjsSeen->clear();

    if(inputAllObjectsPerceived.getInputCount())
        allObjsSeen = inputAllObjectsPerceived.read(false);
    
    if(allObjsSeen != nullptr){
        if(desconsiderRobotColorAsObject() && indexRobotAsObject != -1)//&& indexRobotAsObject != -1 to not decrease 2 times the number of objects when playing with a random object (already check the robot color when compose the features)
            numberOfObjectsScene = allObjsSeen->get(0).asInt16() - 1;//-1 to desconsider the object with the same color of the robot (probably his arm)
        else
            numberOfObjectsScene = allObjsSeen->get(0).asInt16();
    }else
        numberOfObjectsScene = 0;
    cout<<"#Objects from perception: "<<numberOfObjectsScene<<endl;    
}

void decisionMakingThread::composeFeatures(){
    featuresToMDP[0] = face_current;     //There is or not a face in my FOV -- from perception
    featuresToMDP[1] = touch_current;    //There is or not touch in my skin -- from perception
    featuresToMDP[2] = gaze_current;     //The person is looking to me or not -- from perception
    
    /*Use this if the continuous values are not good in the learning phase (the function approx should deal with this)
    //There is or not a face in my FOV -- from perception
    if(face_current != 0.0)
        featuresToMDP[0] = 1;
    else
        featuresToMDP[0] = 0;
  
    //There is or not touch in my skin -- from perception
    if(touch_current != 0.0)
        featuresToMDP[1] = 1;
    else
        featuresToMDP[1] = 0;

    //The person is looking to me or not -- from perception
    if(gaze_current != 0.0)
        featuresToMDP[2] = 1;
    else
        featuresToMDP[2] = 0;
    */

    //There is or not toys in my FOV -- from perception
    if(numberOfObjectsScene != 0)
        featuresToMDP[3] = 1;
        //featuresToMDP[3] = numberOfObjectsScene;//Use this in the future versions
    else
        featuresToMDP[3] = 0;

    featuresToMDP[4] = surviveDrive;
    featuresToMDP[5] = affectDrive;
    featuresToMDP[6] = boredomDrive;

    /*cout<<"featuresToMDP: ";
    for(int i = 0; i < features; i++)
        cout<<featuresToMDP[i]<<" ";
    cout<<"\n";*/
}

double decisionMakingThread::rewardFunction(double surviveDrive, double affectDrive, double boredomDrive){
    double reward = 0;
    //Reward related to battery
    if((abs(int(surviveDrive)) == 0) && (surviveDrive * (-1) <= 0))
        reward += 1;
    else if(surviveDrive < 0)
        reward += surviveDrive;
    else
        reward += -(surviveDrive * 0.5);

    //Reward related to affection
    if((abs(int(affectDrive)) == 0) && (affectDrive * (-1) <= 0))
        reward += 1;
    else if(affectDrive < 0)
        reward += affectDrive;
    else
        reward += -(affectDrive * 0.5);

    //Reward related to boredom
    if((abs(int(boredomDrive)) == 0) && (boredomDrive * (-1) <= 0))
        reward += 1;
    else if(boredomDrive < 0)
        reward += boredomDrive;
    else
        reward += -(boredomDrive * 0.5);

    return reward;
}

void decisionMakingThread::featuresToRL(){
    //Set the Q state features in the QLearn
    QL_agent->shiftStateFeatures();

    //Set the Q' state features in the QLearn
    composeFeatures();

    QL_agent->setFeatures(featuresToMDP, previousBehavior);
}

void decisionMakingThread::updateToRL(){
    //Just have a reward and update the table in the learning phase
    if(mode.compare(LEARNING_PHASE) == 0 && !first){
        //Compute the reward
        double reward = rewardFunction(surviveDrive, affectDrive, boredomDrive);
        allRewardsTraining[current_episode] += reward;
        //Update the "table"
        QL_agent->update(previousBehavior, reward);
        cout<<"previousBehavior: "<<previousBehavior<<endl;
    }else
        first = false;
}

void decisionMakingThread::getNextAction(){
    //Choose an action to execute
    actionRL = QL_agent->getAction();
    cout<<"actionRL: "<<actionRL<<endl;
    detailBehaviorActions(robotState(actionRL));//access enum by index c++ (https://stackoverflow.com/questions/321801/enum-c-get-by-index)
}

void decisionMakingThread::getData(){
    //Data from the Motivation modules
    readBatteryDriveData();
    readAffectDriveData();
    readBoredomDriveData();

    //Data from the Perception modules
    readAffectPerceived();
    readSkinPerceived();
    readObjectsPerceived();

    //DATA FOR DEBUGGING
    if(inputBoredomPort.getInputCount()){
        Bottle* boredomB = inputBoredomPort.read(true);
        cout<<"Boredom: "<<boredomB->get(0).asFloat64()<<endl;
    }

    if(inputComfortPort.getInputCount()){
        Bottle* comfortB = inputComfortPort.read(true);
        cout<<"Comfort: "<<comfortB->get(0).asFloat64()<<endl;
    }
}

void decisionMakingThread::checkIfNoBattery(){
    Bottle* noBatteryBottle = inputNoBatteryPort.read(true);
    noBattery = noBatteryBottle->get(0).asBool();
    cout<<"No Battery: "<<noBattery<<endl;
}

void decisionMakingThread::run(){
    now = time(0);
    char* timeNow_ = ctime(&now);
    timeNow = timeNow_;
    erase_all(timeNow, "\n");

    saturationInteraction = false;

    if(mode.compare(LEARNING_PHASE) == 0 || mode.compare(FINETUNING_PHASE) == 0 || mode.compare(TESTING_PHASE) == 0)
        makeDecision_RL();//Use this one for run the algorithm using RL (training, testing and finetunning phases)
    else if(mode.compare(RULE_BASED) == 0)
        makeDecision_RuleBased();//Use this one for the pre-defined behavior with fixed values (used to collect data in a HRI experiment to then go to RL -- first phase)
    else
        makeDecision_DriveBased();
}

void decisionMakingThread::makeDecision_RuleBased(){
    if(Time::now() - timeOfAction >= durationOfAction){
        //Tells the motivation that can start compute the affect and explore drives
        if(previousBehavior == initial && first == false){
            if(outputPortEndInitialBehavior.getOutputCount()){
                Bottle update;
                update.clear();
                update.addInt16(1);
                outputPortEndInitialBehavior.prepare() = update;
                outputPortEndInitialBehavior.write();
            }
        }
        if(Time::now() - timeInitExperiment <= durationOfExperiment){
            durationOfAction = 0;
            getData();
            if(first){
                behavior = initial;
                first = false;
            }//Check rules related to battery first cause is the most important one
            else if(previousBehavior == recharge && surviveDrive < 0)//Recharge until reach homeostasis
                behavior = recharge;
            else if(surviveDrive <= survive_min_threshold)//Activate the Recharge behavior
                behavior = recharge;
            //Check rules related to play cause depends "just" the robot behavior and not the partner interacting with
            else if((previousBehavior == play || previousBehavior == lookDown) && boredomDrive < 0)//Play until reach homeostasis
                behavior = play;
            else if(boredomDrive <= explore_min_threshold && (previousBehavior != play && previousBehavior != lookDown))//Needs to play, but first should look to the table
                behavior = lookDown;
            else if(boredomDrive <= explore_min_threshold)//Activate the Explore/Play behavior (maybe not needed this if -- considering we are using the if above)
                behavior = play;
            //Check rules related to interaction
            else if(previousBehavior == interact && affectDrive < 0){
                cout<<"waitingInteraction: "<<waitingInteraction<<endl;
                /*  (affectDrive - last_affectDrive) > 0 means person interacted
                    (affectDrive - last_affectDrive) < 0 means no interaction */
                if((affectDrive - last_affectDrive) <= 0)//Needs interact but the person is not interacting, so wait interaction for a while
                    waitingInteraction++;
                else
                    waitingInteraction = 0;

                cout<<"originOfTouch: "<<originOfTouch<<endl;
                
                if(originOfTouch != noTouch){
                    string gazeDirection = lookAtTouchedPart(originOfTouch);
                    writeCommand("gaze,moveHeadDirection," + gazeDirection);
                    Time::delay(3);
                    writeCommand("gaze,moveHeadDirection,home");
                    durationOfAction += 2 * TIME_GAZE;
                    waitingInteraction = 0;
                }

                last_affectDrive = affectDrive;
            }else if(affectDrive <= affect_min_threshold){//Activate the Interact behavior
                behavior = interact;
                last_affectDrive = affectDrive;
                waitingInteraction = 0;
            }else //All the drives are satisfied, so stay in idle
                behavior = idle;

            if(previousBehavior == recharge && behavior != recharge) //openEyes when finish the recharge behavior and change to the next one
                writeCommand("eyelids,1.0,0.0,open");

            if(behavior != interact || (behavior == interact && previousBehavior != interact)){
                detailBehaviorActions(behavior);
                waitingInteraction = 0;
            }else if(waitingInteraction >= 35){//waitingInteraction >= 15 -> needs interaction and ask again for waiting for a while
                    detailBehaviorActions(behavior);
                    waitingInteraction = 0;
                }
            previousBehavior = behavior;
        }else{
            cout<<"End of the experiment"<<endl;
            
            if(previousBehavior == recharge) //openEyes when finish the recharge behavior and change to the next one
                writeCommand("eyelids,1.0,0.0,open");

            if(previousBehavior != endInteraction){
                Time::delay(3);
                getData();
                behavior = endInteraction;
                detailBehaviorActions(behavior);
            }
        } 
    }else{
        cout<<"Action in progress"<<endl;
    }

    cout<<"--------------------------------"<<endl;
}

void decisionMakingThread::makeDecision_DriveBased(){
    if(Time::now() - timeOfAction >= durationOfAction){
        //Tells the motivation that can start compute the affect and explore drives
        if(previousBehavior == initial && first == false){
            timeInitExperiment = Time::now();
            if(outputPortEndInitialBehavior.getOutputCount()){
                Bottle update;
                update.clear();
                update.addInt16(1);
                outputPortEndInitialBehavior.prepare() = update;
                outputPortEndInitialBehavior.write();
            }
        }       
        if(Time::now() - timeInitExperiment <= durationOfExperiment){
            durationOfAction = 0;
            getData();
            if(first){
                behavior = initial;
                first = false;
            }
            
            else if(surviveDrive >= 0 && affectDrive >= 0 && boredomDrive >= 0)
                behavior = idle;

            //Recharge until homeostasis
            else if(previousBehavior == recharge && surviveDrive <= 0)//Recharge until reach homeostasis
                behavior = recharge;

            else if(surviveDrive <= affectDrive && surviveDrive <= boredomDrive)//Activate the Recharge behavior
                behavior = recharge;

            else if(previousBehavior == lookDown)//means that the other conditions already defined that it is time to play (first step is to look down)
                behavior = play;

            else if(affectDrive < boredomDrive){
                if(previousBehavior != play)//Was interacting and still is a high drive OR was recharging or idle before 
                    behavior = interact;
                else if(previousBehavior == play && (affectDrive - boredomDrive < -12))//Was playing but wait a bit to change the behavior (to avoid switching behavior at each timestp when the drives are very close)
                    behavior = interact;
                else if(previousBehavior == play)
                    behavior = play;
                else
                    behavior = lookDown;
            }

            else if(boredomDrive <= affectDrive){
                if(previousBehavior != interact && previousBehavior != play  && previousBehavior != lookDown)
                    behavior = lookDown;
                else if(previousBehavior != interact)
                    behavior = play;
                else if(previousBehavior == interact && (boredomDrive - affectDrive < -12))
                    behavior = lookDown;
                else
                    behavior = interact;
            }

            //Check rules related to interaction -- to not ask for attention during a short time
            if(behavior == interact && previousBehavior == interact){
                cout<<"waitingInteraction: "<<waitingInteraction<<endl;
                /*  (affectDrive - last_affectDrive) > 0 means person interacted
                    (affectDrive - last_affectDrive) < 0 means no interaction */
                if((affectDrive - last_affectDrive) <= 0)//Needs interact but the person is not interacting, so wait interaction for a while
                    waitingInteraction++;
                else{
                    waitingInteraction = 0;
                    saturedAffect = 0;
                }

                if((affectDrive - last_affectDrive) == 0 && affectDrive < 0)//means that reach the saturation value
                    saturedAffect++;

                cout<<"saturedAffect: "<<saturedAffect<<endl;
                cout<<"originOfTouch: "<<originOfTouch<<endl;
                
                if(originOfTouch != noTouch){
                    waitingInteraction = 0;
                    indexInteractionReply++;
                    if(indexInteractionReply % 3 == 0){
                        string gazeDirection = lookAtTouchedPart(originOfTouch);
                        writeCommand("gaze,moveHeadDirection," + gazeDirection);
                        Time::delay(3);
                        writeCommand("gaze,moveHeadDirection,home");
                        durationOfAction += 2 * TIME_GAZE;
                    }else if(indexInteractionReply % 5 == 0)
                        writeCommand("speech,#LAUGH01#");
                    Time::delay(1.5);
                    writeCommand("face,raise,smile");
                    timeOfAction = Time::now();
                }
            }
            last_affectDrive = affectDrive;
            
            if(previousBehavior == recharge && behavior != recharge){ //openEyes when finish the recharge behavior and change to the next one
                writeCommand("speech,#YAWN01#");
                writeCommand("eyelids,1.0,0.0,open");
                Time::delay(3);
            }
            
            if(behavior != idle)
                timeToWhistle = 0;
            
            if(behavior != interact || (behavior == interact && previousBehavior != interact)){
                detailBehaviorActions(behavior);
                waitingInteraction = 0;
            }else if(waitingInteraction >= 35 || saturedAffect >= 115){//needs interaction and ask again for waiting for a while
                if(saturedAffect >= 115)
                    saturationInteraction = true;
                detailBehaviorActions(behavior);
                waitingInteraction = 0;
            }
            previousBehavior = behavior;
        }else{
            cout<<"End of the experiment"<<endl;
            
            if(previousBehavior == recharge) //openEyes when finish the recharge behavior and change to the next one
                writeCommand("eyelids,1.0,0.0,open");

            if(previousBehavior != endInteraction){
                Time::delay(3);
                getData();//To save the last drives values
                behavior = endInteraction;
                detailBehaviorActions(behavior);
            }
        } 
    }else{
        if(behavior == recharge)
            writeCommand("speech,#SLEEP02#");
        cout<<"Action in progress"<<endl;
    }

    cout<<"--------------------------------"<<endl;
}

void decisionMakingThread::makeDecision_RL(){
    colorObj = "";

    if(previousBehavior == recharge)
        writeCommand("eyelids,1.0,0.0,open");

    checkIfNoBattery();
   
    waitBufferPerception += 1;

    cout<<"Time: "<<Time::now() - timeOfAction<<endl;

    if(Time::now() - timeOfAction >= durationOfAction){
        if(noBattery){
            reset();
            allRewardsTraining[current_episode-1] += deathPunishment;//current_episode-1 cause in reset() there is current_episode++
            getData();
            featuresToRL();
            //Update the "table"
            QL_agent->update(previousBehavior, deathPunishment);
            cout<<"previousBehavior: "<<previousBehavior<<endl;
            durationOfAction = 0;
            first = true;
        }else{
            durationOfAction = 0;
            //Get all the data needed to compose the features to be used in the MDP
            getData();
            if(!endExperiment()){
                if(!endEpisode()){
                    featuresToRL();
                    if(waitBufferPerception > previousStatesToRepeat){
                        updateToRL();
                        getNextAction();
                        QL_agent->setFeaturesOldState();
                        steps += 1;
                    }else
                        cout<<"Waiting fill buffer from perception to compose the previous first state"<<endl;
                }else{
                    //update the "Qtable" after execute the last action of the episode
                    featuresToRL();
                    updateToRL();
                    first = true;
                }
            }else{
                cout<<"Experiment Finished"<<endl;
                //TODO: stop app
                if(fim){//remove this -- its here now just to not save several times until stop the app
                    fim = false;
                    saveTrainingData();
                    detailBehaviorActions(endInteraction);
                }
            }
        }
    }else
        cout<<"Executing action: "<<actionRL<<endl;
    
    cout<<"Reward: "<<allRewardsTraining[current_episode]<<endl;


    cout<<"Episode: "<<current_episode<<"     step: "<<steps<<endl;
    cout<<"----------------------"<<endl;
}

bool decisionMakingThread::endExperiment(){
    if(((mode.compare(LEARNING_PHASE) == 0 || mode.compare(FINETUNING_PHASE) == 0) && current_episode >= total_episodes) || (mode.compare(TESTING_PHASE) == 0 && endTest))//code related to the test phase is just to stop the app like in the training
        return true;
    return false;
}

void decisionMakingThread::saveTrainingData(){
    if(current_episode % 10 == 0 && (mode.compare(LEARNING_PHASE) == 0 || mode.compare(FINETUNING_PHASE) == 0)){//save partial results
        QL_agent->saveWeights();
        QL_agent->saveEpsilonData();
        saveRewards();
    }
}

bool decisionMakingThread::endEpisode(){
    if(steps >= maxSteps || noBattery){
        saveTrainingData();
        reset();
        return true; 
    }
    return false;
}

void decisionMakingThread::reset(){
    //Robot come back to the initial position and in the simulator we apply a random new position to head
    writeCommand("gaze,moveHeadDirection,home");
    writeCommand("action,1.0,0.0,SaraHome");
    writeCommand("face,neutral,neutral");

    std::uniform_int_distribution<int> distrNeck(-20, 20); // Neck pitch joint range limit (-30, 22)
    std::uniform_int_distribution<int> distrYaw(-20, 20); //Neck yaw joint range limit

    int pitch = distrNeck(rdx);//Neck pitch (vertical)
    int yaw = distrYaw(rdx);//Neck yaw (horizontal)

    writeCommand("gaze,turnHeadTwoAngles," + to_string(yaw) + "," + to_string(pitch));
    
    steps = 0;
    waitBufferPerception = 0;
    saturationInteraction = false;

    cout<<"Finish Episode: "<<current_episode<<endl;
    cout<<"Approx Reward: "<<allRewardsTraining[current_episode]<<endl;

    current_episode++;
    endTest = true;
    cout<<"Next Episode Initial Reward: "<<allRewardsTraining[current_episode]<<endl;

    if(mode.compare(LEARNING_PHASE) == 0 || mode.compare(FINETUNING_PHASE) == 0)
        QL_agent->updateEpsilon(current_episode, total_episodes);

    if(outputPortResetWorld.getOutputCount()){
        Bottle update;
        update.clear();
        update.addInt16(1);
        outputPortResetWorld.prepare() = update;
        outputPortResetWorld.write();
    }
                
    if(outputPortResetBattery.getOutputCount()){//Send a message to the simulated sensor
        Bottle resetBottle;
        resetBottle.clear();           
        resetBottle.addInt16(1);
        outputPortResetBattery.prepare() = resetBottle;
        outputPortResetBattery.write();
    }

    if(outputPortResetBoredom.getOutputCount()){
        Bottle update;
        update.clear();
        update.addInt16(1);
        outputPortResetBoredom.prepare() = update;
        outputPortResetBoredom.write();
    }

    cout<<"Reseting"<<endl;

    Time::delay(TIME_GAZE);//check the need of this command

    cout<<"----------------------"<<endl;
}

string decisionMakingThread::lookAtSpecificObject(int i){
    string gazeDirection;

    //Objects are with hardcoded positions
    if(i == 1)
        gazeDirection = "farDiagonalLeft_Down";
    else if(i == 2)
        gazeDirection = "closeDiagonalLeft_Down";
    else if(i == 3)
        gazeDirection = "center_Down";
    else if(i == 4)
        gazeDirection = "farDiagonalRight_Down";
    else if(i == 5)
        gazeDirection = "closeDiagonalRight_Down";
    
    return gazeDirection;
}

void decisionMakingThread::playFixedObjects_CTP(){
    std::uniform_int_distribution<int> distr(1, numberOfFixedObjs);
    //int i = distr(rdx);
    int i = objectsSequence[indexSeq_objToPlay];

    indexSeq_objToPlay++;
    if(indexSeq_objToPlay >= 20)
        indexSeq_objToPlay = 0;

    writeCommand("gaze,moveHeadDirection," + lookAtSpecificObject(i));
    Time::delay(2);
    writeCommand("action,1.0,0.0,point_object_" + to_string(i));
    writeCommand("speech," + soundsPlay[indexSoundPlay]);
    
    indexSoundPlay++;
    if(indexSoundPlay == 24)
        indexSoundPlay = 0;

    Time::delay(15);
    writeCommand("action,2.0,0.0,SaraHome");

    Time::delay(4);
    writeCommand("gaze,moveHeadDirection,homeDown");
    durationOfAction += TIME_GAZE;

    updateBoredom();
}

void decisionMakingThread::playDynamicObjects_ARE(){
    if(previousBehavior != play){
        writeCommand("homeARE");
        //updateBatteryConsumption("play");
        Time::delay(3);
        //writeCommand("speech,Play!");
        //writeCommand("face,raise,smile");
        //Time::delay(1);
        //TODO:when the object choosen has the maximum value to Boredom (or is new) can express surprise/novelty
        durationOfAction += TIME_HOME_ARE;
    }

    if(numberOfObjectsScene != 0){
        writeCommand("face,raise,neutral");
        durationOfAction += TIME_LEDS;
        if(colorObj.compare("") != 0){//Play with the object indicated by the motivation module
            Bottle specificObjectData;
            specificObjectData.clear();
            specificObjectData = getObjectCoords(colorObj);
            if(specificObjectData.size() != 0){
                cout<<"Playing with the object indicated by the motivation"<<endl;
                cout<<"specificObjData "<<specificObjectData.get(0).toString()<<endl;
                definePlay(specificObjectData);
            }else{
                cout<<"The object choosen by motivation is not in my FOV anymore. I'll play with a random one in my FOV"<<endl;
                Bottle randObj = getRandomObjectToPlay();
                if(randObj.get(0).asList())
                    definePlay(randObj);
                else
                    cout<<"No objects to play"<<endl;
            }
        }else{//play with a random toy(that it is seen after move the head down, so the motivation doesn't know about it)
            cout<<"Motivation doesn't choose an object, so I'm playing with a random one there is in my FOV"<<endl;
            Bottle randObj = getRandomObjectToPlay();
            if(randObj.get(0).asList())
                definePlay(randObj);
            else
                cout<<"No objects to play"<<endl;
        }
    }else{
        writeCommand("speech, Hum");//Where are my toys?
        Time::delay(2);
        writeCommand("face,neutral,sad");
        cout<<"Decision is to play, but I'm not seeing my toys"<<endl;
        durationOfAction += TIME_SPEECH;
    }
}

void decisionMakingThread::definePlay(Bottle specificObjectData){
    cout<<"Color of object to play: "<<specificObjectData.get(0).asList()->get(8).asString()<<endl;
    writeCommand("play," 
                + to_string(specificObjectData.get(0).asList()->get(0).asInt16()) + "," 
                + to_string(specificObjectData.get(0).asList()->get(1).asInt16()) + "," 
                + to_string(specificObjectData.get(0).asList()->get(2).asInt16()) + "," 
                + to_string(specificObjectData.get(0).asList()->get(3).asInt16()) + "," 

                + to_string(specificObjectData.get(0).asList()->get(4).asInt16()) + "," 
                + to_string(specificObjectData.get(0).asList()->get(5).asInt16()) + "," 
                + to_string(specificObjectData.get(0).asList()->get(6).asInt16()) + "," 
                + to_string(specificObjectData.get(0).asList()->get(7).asInt16()) + "," 

                + specificObjectData.get(0).asList()->get(8).asString()
    );

    writeCommand("speech," + soundsPlay[indexSoundPlay]);
    indexSoundPlay++;
    if(indexSoundPlay == 22)
        indexSoundPlay = 0;

    durationOfAction += TIME_POINT_ARE + TIME_SPEECH;

    Time::delay(3);

    updateBoredom();
}

void decisionMakingThread::updateBoredom(){
    if(outputUpdateBoredomPort.getOutputCount()){
        Bottle update;
        update.clear();
        update.addInt16(1);
        outputUpdateBoredomPort.prepare() = update;
        outputUpdateBoredomPort.write();
    }
}

//TODO: just one part of the robot is desconsidered as an object (if there is more than one, the other parts still are considered as object)
//easy (but not elegant) way to fix: having an array with the robot parts indexes
bool decisionMakingThread::desconsiderRobotColorAsObject(){
    for(int i = 0; i < allObjsSeen->get(0).asInt16(); i++)
        if(allObjsSeen->get(i+1).asList()->get(8).asString().compare(robot_color) == 0){
            indexRobotAsObject = i + 1;//save the array index where the object is the robot arm
            return true;
        }
    indexRobotAsObject = -1;
    return false;
}

Bottle decisionMakingThread::getRandomObjectToPlay(){
    //readObjectsPerceived();
    int i, upperBound;

    Bottle specificObjectData;
    specificObjectData.clear();

    if(numberOfObjectsScene > 0){
        if(indexRobotAsObject != -1)//Means that the part of the robot is recognized as an object
            upperBound = numberOfObjectsScene + 1;
        else
            upperBound = numberOfObjectsScene;
        
        std::uniform_int_distribution<int> distr(1, upperBound);
        i = distr(rdx) - 1;

        while(i == indexRobotAsObject)//desconsider robot part as a toy
            i = distr(rdx) - 1;

        specificObjectData.add(allObjsSeen->get(i));
    }

    return specificObjectData;
}

void decisionMakingThread::detailBehaviorActions(robotState behavior){
    if(previousBehavior == endInteraction){
        cout<<"Stoping all application"<<endl;
        //Interrupt all the application -- maybe should be in the action module (after executing all commands)
        if(ouputStopApplication.getOutputCount()){
            Bottle stop;
            stop.clear();
            stop.addString("quit");
            ouputStopApplication.prepare() = stop;
            ouputStopApplication.write();
        }
    }else{
        saveData(behavior);
        switch(behavior){
            case initial:
                cout<<"Initial"<<endl;
                updateBatteryConsumption("initial");
                writeCommand("face,neutral,smile");
                writeCommand("eyelids,1.0,0.0,open");//For cases when stopp the last interaction recharging
                writeCommand("gaze,moveHeadDirection,home");
                writeCommand("speech,Hi I'm " + robot_name + "!");
                writeCommand("action,1.5,0.0,greet0");
                writeCommand("action,0.7,0.0,greet1");
                writeCommand("action,0.7,0.0,greet2");
                writeCommand("action,0.7,0.0,greet1");
                writeCommand("action,0.7,0.0,greet2");
                Time::delay(5);
                writeCommand("action,2.0,0.0,SaraHome");
                Time::delay(1);
                durationOfAction += TIME_SPEECH + TIME_CTP;
                timeOfAction = Time::now();
                break;
            case idle://No drive need to be satisfied
                cout<<"Idle"<<endl;
                if(previousBehavior != idle){
                    updateBatteryConsumption("idle");
                    writeCommand("action,2.0,0.0,SaraHome");
                    writeCommand("gaze,moveHeadDirection,home");
                    writeCommand("face,neutral,neutral");
                    Time::delay(2);
                    durationOfAction += TIME_CTP + TIME_LEDS + TIME_GAZE;
                }
                writeCommand("gaze,lookAround");

                if(timeToWhistle % 3 == 0)
                    writeCommand("speech,#WHISTLE01#");//To not being so annoy during a sequence of idle state
                timeToWhistle += 1;

                durationOfAction += TIME_GAZE + TIME_SPEECH;
                timeOfAction = Time::now();
                break;
            case interact:
                cout<<"Interact"<<endl;
                if(previousBehavior != interact){
                    updateBatteryConsumption("interact");
                    writeCommand("action,2.0,0.0,SaraHome");
                    Time::delay(2);
                    writeCommand("speech," + soundsInteract[1]);
                    writeCommand("gaze,moveHeadDirection,home");
                    writeCommand("speech,care");
                    writeCommand("action,2.0,0.0,Interact");
                    Time::delay(2);
                    durationOfAction += TIME_LEDS + TIME_SPEECH + 2 * TIME_CTP + TIME_GAZE;
                }else{
                    writeCommand("speech," + soundsInteract[indexSoundInteract]);
                    indexSoundInteract++;
                    if(indexSoundInteract == 2)
                        indexSoundInteract = 0;
                    durationOfAction += TIME_SPEECH;

                    if(saturedAffect >= 15){//Mechanism to try to solve the affect drive when the person is not interacting and the drive is oversatured -- look to a photo in the environment
                        writeCommand("action,2.0,0.0,SaraHome");
                        Time::delay(2);
                        writeCommand("gaze,moveHeadDirection,rightUp");
                        Time::delay(2);
                        if(outputPortSaturetedAffect.getOutputCount()){
                            Bottle update;
                            update.clear();
                            update.addInt16(1);
                            outputPortSaturetedAffect.prepare() = update;
                            outputPortSaturetedAffect.write();
                        }
                        Time::delay(5);
                        writeCommand("face,raise,smile");
                        writeCommand("gaze,moveHeadDirection,home");
                        writeCommand("speech,#BREATH02#");//Change sound
                        durationOfAction += TIME_GAZE;
                        saturedAffect = 0;
                        cout<<"Interacting saturation behavior"<<endl;
                    }
                }
                Time::delay(1.5);
                writeCommand("face,neutral,sad");
                Time::delay(1.5);
                
                timeOfAction = Time::now();
                break;
            case recharge:
                cout<<"Recharge"<<endl;

                if(previousBehavior != recharge){
                    updateBatteryConsumption("recharge");
                    writeCommand("action,1.0,0.0,SaraHome");
                    writeCommand("speech,#BREATH02# Sleeeep!");
                    writeCommand("gaze,moveHeadDirection,home");
                    writeCommand("eyelids,1.0,0.0,close");
                    Time::delay(3);
                    writeCommand("face,neutral,neutral");
                    
                    durationOfAction += TIME_LEDS + TIME_SPEECH;
                }
                writeCommand("recharge");
                durationOfAction += TIME_RECHARGE;
                Time::delay(1.5);
                timeOfAction = Time::now();
                break;
            //case jointInteraction/attention:
                //maybe later look at the person after point the object as is showing the decision and then look back to the table
                //break;
            case play:
                cout<<"Play"<<endl;
                if(previousBehavior != play){
                    updateBatteryConsumption("play");
                    Time::delay(2);
                    writeCommand("speech,Play!");
                    writeCommand("face,raise,smile");
                    Time::delay(1);
                    durationOfAction += TIME_SPEECH;
                }
                //Used when the objects are in a fixed position and the position to point is pre-recorded
                playFixedObjects_CTP();

                //Used when the objects can move the position and the point to reach is not pre-recorded
                //playDynamicObjects_ARE();

                timeOfAction = Time::now();
                break;
            case lookDown:
                cout<<"lookDown"<<endl;
                updateBatteryConsumption("lookDown");
                if(previousBehavior == interact)
                    writeCommand("action,1.0,0.0,SaraHome");
                writeCommand("gaze,moveHeadDirection,homeDown");
                durationOfAction = TIME_GAZE + TIME_CTP; 
                timeOfAction = Time::now();
                break;
            case endInteraction:
                cout<<"endInteraction"<<endl;
                updateBatteryConsumption("end");
                Time::delay(10);
                writeCommand("gaze,moveHeadDirection,home");
                writeCommand("speech,#BREATH01#...... Funny.... Bye!");//Our time is over. It was great to see you. 
                Time::delay(1);
                writeCommand("face,raise,smile");
                writeCommand("action,1.5,0.0,greet0");
                writeCommand("action,0.7,0.0,greet1");
                writeCommand("action,0.7,0.0,greet2");
                writeCommand("action,0.7,0.0,greet1");
                writeCommand("action,0.7,0.0,greet2");
                Time::delay(5);
                writeCommand("action,1.0,0.0,SaraHome");
                durationOfAction = 3 * TIME_CTP + TIME_GAZE + TIME_SPEECH + TIME_LEDS;
                timeOfAction = Time::now();
                break;
        }

        previousBehavior = behavior;
        //saveData(behavior);
    }
    
}

void decisionMakingThread::writeCommand(std::string commandSM){
    Bottle bottleToSendAction;
    if(outputActionPort.getOutputCount()){
        bottleToSendAction.clear();
        bottleToSendAction.addString(commandSM);
        outputActionPort.prepare() = bottleToSendAction;
        outputActionPort.writeStrict();
        //outputActionPort.write();
    }
}

void decisionMakingThread::updateBatteryConsumption(string behavior){
    Bottle updateBatteryCons;
    if(outputUpdBatteryConsPort.getOutputCount()){
        updateBatteryCons.clear();
        updateBatteryCons.addString(behavior);
        outputUpdBatteryConsPort.prepare() = updateBatteryCons;
        outputUpdBatteryConsPort.write();
    }
}

string decisionMakingThread::lookAtTouchedPart(int originOfTouch){
    string gazeDirection;

    if(originOfTouch == torso)
        gazeDirection = "down";
    else if(originOfTouch == leftHand || originOfTouch == leftForearm)
        gazeDirection = "leftDown";
    else if(originOfTouch == leftUpper)
        gazeDirection = "leftDownUpper";
    else if(originOfTouch == rightHand || originOfTouch == rightForearm)
        gazeDirection = "rightDown";
    else if(originOfTouch == rightUpper)
        gazeDirection = "rightDownUpper";

    return gazeDirection;
}

void decisionMakingThread::saveHeaders(){
    //Save header in the file
    ofstream fout; 
    
    fout.open(filenameAllData, ios::app);
    fout << fileHeaderAllData << "\n";
    fout.close( );

    if(mode.compare(LEARNING_PHASE) == 0){
        fout.open(filenameRewards, ios::app);
        fout << fileHeaderRewards << "\n";
        fout.close( );
    }
}

void decisionMakingThread::saveData(robotState behavior){
    ofstream fout;
    fout.open(filenameAllData, ios::app);
    
    fout << timeNow << ',' << to_string(Time::now() - timeInitial) << ',' << to_string(boredomDrive) << ',' << to_string(affectDrive) << ',' << to_string(surviveDrive) << ',' 
    << to_string(survive_min_threshold) << ',' << to_string(affect_min_threshold) << ',' << to_string(explore_min_threshold) << ','
    << colorObj << ',' << behavior << ',' << saturationInteraction << ',' << current_episode << ',' << steps;

    fout <<"\n";
    fout.close();
}

void decisionMakingThread::saveRewards(){
    ofstream fout;
    
    fout.open(filenameRewards);
    fout << fileHeaderRewards << "\n";
    
    for(int i = 0; i < total_episodes; i++)
        fout << i << ',' << to_string(allRewardsTraining[i]) << '\n';

    fout.close();
}

bool decisionMakingThread::processing(){
    // here goes the processing...
    return true;
}

void decisionMakingThread::threadRelease() {
    inputInteractionPort.interrupt();
    inputNoBatteryPort.interrupt();
    inputPortBoredomDriveObject.interrupt();
    inputAllObjectsPerceived.interrupt();
    inputSurvivalDrive.interrupt();
    inputAffectDrive.interrupt();
    inputSkinPerceivedPort.interrupt();
    //inputAffectPerceivedPort.interrupt();
    outputActionPort.interrupt();
    ouputStopApplication.interrupt();
    outputUpdateBoredomPort.interrupt();
    outputUpdBatteryConsPort.interrupt();
    outputPortResetBattery.interrupt();
    outputPortResetBoredom.interrupt();
    outputPortResetWorld.interrupt();
    outputPortEndInitialBehavior.interrupt();
    outputPortSaturetedAffect.interrupt();


    inputBoredomPort.interrupt();
    inputComfortPort.interrupt();
    inputBoredomPort.close();
    inputComfortPort.close();

    inputInteractionPort.close();
    inputNoBatteryPort.close();
    inputPortBoredomDriveObject.close();
    inputAllObjectsPerceived.close();
    inputSurvivalDrive.close();
    inputAffectDrive.close();
    inputSkinPerceivedPort.close();
    //inputAffectPerceivedPort.close();
    outputActionPort.close();
    ouputStopApplication.close();
    outputUpdateBoredomPort.close();
    outputUpdBatteryConsPort.close();
    outputPortResetBattery.close();
    outputPortResetBoredom.close();
    outputPortResetWorld.close();
    outputPortEndInitialBehavior.close();
    outputPortSaturetedAffect.close();
}