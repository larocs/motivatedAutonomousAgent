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
 * @file motivationThread.cpp
 * @brief Implementation of the eventDriven thread (see motivationThread.h).
 */

#include <iCub/motivationThread.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace boost;

#define THPERIOD 0.5 //s
#define NON_EXIST -1 //Must be the same value defined in objectPerception module

motivationThread::motivationThread():PeriodicThread(THPERIOD) {
    robot = "icub";        
}

motivationThread::motivationThread(string _robot, string _configFile):PeriodicThread(THPERIOD){
    robot = _robot;
    configFile = _configFile;
}

motivationThread::motivationThread(string _robot, ResourceFinder &_rf, string _robot_color, string _robot_profile):PeriodicThread(THPERIOD){
    robot = _robot;
    rf = _rf;
    robot_color = _robot_color;
    robot_profile = _robot_profile;
}

motivationThread::~motivationThread() {
    // do nothing
}

void motivationThread::setName(string str) {
    this->name=str;
}


std::string motivationThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void motivationThread::setInputPortName(string InpPort) {
    
}

bool motivationThread::openAllPorts(){
    if(!inputBatteryLevelPort.open(getName("/batteryLevel:i").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!inputAllObjects.open(getName("/allObjectsSeen:i").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!inputInteractionPort.open(getName("/gazeFaceSkin:i").c_str())){
        yError("unable to open port to receive evaluation of interaction data");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!inputUpdateBoredomPort.open(getName("/updateBoredom:i").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!inputPortResetBoredomComfort.open(getName("/resetBoredomComfort:i").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!inputPortEndInitialBehavior.open(getName("/startDrivesComputation:i").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!outputAffectDrivePort.open(getName("/affectDrive:o").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if(!outputExploreDriveAndObject.open(getName("/exploreDriveObject:o").c_str())){
        yError("unable to open port to receive output");
        return false;  // unable to open; let RFModule know so that it won't run
    }
        
    if(!outputSurvivalDrivePort.open(getName("/survivalDrive:o").c_str())){
        yError("unable to open port to receive output");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!inputPortSaturetedAffect.open(getName("/saturetedAffectAct:i").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!inputiCubesPort.open(getName("/iCubes:i").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    //######### Just for debugging #########
    if(!outputBoredomPort.open(getName("/boredom:o").c_str())){
        yError("unable to open port to receive output");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!outputComfortPort.open(getName("/comfort:o").c_str())){
        yError("unable to open port to receive output");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    return true;
}

bool motivationThread::waitForPortConnections(){
    yInfo("Wait for iCub port connections");

    while(inputBatteryLevelPort.getInputCount() < 1);
    while(inputAllObjects.getInputCount() < 1);
    while(inputUpdateBoredomPort.getInputCount() < 1);
    while(inputInteractionPort.getInputCount() < 1);
    while(outputExploreDriveAndObject.getOutputCount() < 1);
    while(outputSurvivalDrivePort.getOutputCount() < 1);
    while(outputAffectDrivePort.getOutputCount() < 1);
    while(inputPortResetBoredomComfort.getInputCount() < 1);

    return true;
}

void motivationThread::initAllVars(){
    indexObjChoosen = NON_EXIST;
    mostInterestingReward = NON_EXIST;

    touch_current = 0.0;
    face_current = 0.0;
    gaze_current = 0.0;
    comfort_prev = 0.5;

    batteryLevel = -1;//default value in case we are not using this sensor
    numberOfObjectsScene = 0;

    initVarsFromFile();
    comfort_prev = comfort_current;

    saturActingTime = 0;
    playingTime_remaining = 0;

    //In these two modes the robot has the state Initial and EndInteraction. So should not computing the drives while the robot is presenting itself
    if(mode == RULE_BASED || mode == DRIVE_BASED)
        computeDrives = false;

    SURVIVAL_HOMEOSTASIS = SURVIVAL_HOMEOSTASIS * (MAX_BATTERY - MIN_BATTERY) + MIN_BATTERY;
    AFFECT_HOMEOSTASIS = AFFECT_HOMEOSTASIS * (MAX_COMFORT - MIN_COMFORT) + MIN_COMFORT;
    EXPLORE_HOMEOSTASIS = MAX_BOREDOM - (MAX_BOREDOM - MIN_BOREDOM) * EXPLORE_HOMEOSTASIS;

    printData();

    resetVars();

    affectDrive = computeDrive(comfort_current, AFFECT_HOMEOSTASIS, RANGE_AFFECT);
    surviveDrive = computeDrive(batteryLevel, SURVIVAL_HOMEOSTASIS, RANGE_SURVIVE);
    exploreDrive = computeDrive(boredom, EXPLORE_HOMEOSTASIS, RANGE_EXPLORE) * (-1);
}

void motivationThread::initVarsFromFile(){
    MIN_BATTERY = rf.findGroup("variables").find("MIN_BATTERY").asFloat32();
    MAX_BATTERY = rf.findGroup("variables").find("MAX_BATTERY").asFloat32();
    MIN_COMFORT = rf.findGroup("variables").find("MIN_COMFORT").asFloat32();
    MAX_COMFORT = rf.findGroup("variables").find("MAX_COMFORT").asFloat32();
    MIN_BOREDOM = rf.findGroup("variables").find("MIN_BOREDOM").asFloat32();
    MAX_BOREDOM = rf.findGroup("variables").find("MAX_BOREDOM").asFloat32();
    INCREASE_BOREDOM = rf.findGroup("variables").find("INCREASE_BOREDOM").asFloat32();
    
    //comfort_current, boredom -- Initial values for the rule based agent

    batteryLevel = rf.findGroup("variables").check("INIT_BATTERY", Value(MAX_BATTERY)).asFloat64();

    SURVIVAL_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_ENERGY").asFloat32();

    if(robot_profile.compare(SOCIAL_PROFILE) == 0){
        AFFECT_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_AFFECT_SOCIAL").asFloat32();
        EXPLORE_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_BOREDOM_SOCIAL").asFloat32();
        comfort_current = rf.findGroup("variables").check("INIT_COMFORT_SOCIAL", Value(MIN_COMFORT)).asFloat64();
        boredom = rf.findGroup("variables").check("INIT_BOREDOM_SOCIAL", Value(MAX_BOREDOM)).asFloat64();
    }else if(robot_profile.compare(PLAYFUL_PROFILE) == 0){
        AFFECT_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_AFFECT_PLAYFUL").asFloat32();
        EXPLORE_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_BOREDOM_PLAYFUL").asFloat32();
        comfort_current = rf.findGroup("variables").check("INIT_COMFORT_PLAYFUL", Value(MIN_COMFORT)).asFloat64();
        boredom = rf.findGroup("variables").check("INIT_BOREDOM_PLAYFUL", Value(MAX_BOREDOM)).asFloat64();
    }else{//REGULAR_PROFILE
        AFFECT_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_REGULAR").asFloat32();
        EXPLORE_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_REGULAR").asFloat32();
        comfort_current = rf.findGroup("variables").check("INIT_COMFORT_REGULAR", Value(MIN_COMFORT)).asFloat64();
        boredom = rf.findGroup("variables").check("INIT_BOREDOM_REGULAR", Value(MAX_BOREDOM)).asFloat64();
    }

    RANGE_SURVIVE = rf.findGroup("variables").find("RANGE_SURVIVE").asInt32();
    RANGE_AFFECT = rf.findGroup("variables").find("RANGE_AFFECT").asInt32();
    RANGE_EXPLORE = rf.findGroup("variables").find("RANGE_EXPLORE").asInt32();

    DRIVE_BOREDOM = rf.findGroup("variables").find("DRIVE_BOREDOM").asInt32();
    DRIVE_AFFECT = rf.findGroup("variables").find("DRIVE_AFFECT").asInt32();
    DRIVE_SURVIVE = rf.findGroup("variables").find("DRIVE_SURVIVE").asInt32();

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
}

void motivationThread::printData(){
    cout<<"Running with:"<<endl;
    cout<<"Robot color: "<<robot_color<<endl;
    cout<<"Robot profile: "<<robot_profile<<endl;
    cout<<"mode: "<<mode<<endl;
    cout<<"comfort_current: "<<comfort_current<<endl;
    cout<<"boredom: "<<boredom<<endl;
    cout<<"MIN_BATTERY: "<<MIN_BATTERY<<endl;
    cout<<"MAX_BATTERY: "<<MAX_BATTERY<<endl;
    cout<<"MIN_COMFORT: "<<MIN_COMFORT<<endl;
    cout<<"MAX_COMFORT: "<<MAX_COMFORT<<endl;
    cout<<"MIN_BOREDOM: "<<MIN_BOREDOM<<endl;
    cout<<"MAX_BOREDOM: "<<MAX_BOREDOM<<endl;
    cout<<"INCREASE_BOREDOM: "<<INCREASE_BOREDOM<<endl;
    cout<<"SURVIVAL_HOMEOSTASIS: "<<SURVIVAL_HOMEOSTASIS<<endl;
    cout<<"AFFECT_HOMEOSTASIS: "<<AFFECT_HOMEOSTASIS<<endl;
    cout<<"EXPLORE_HOMEOSTASIS: "<<EXPLORE_HOMEOSTASIS<<endl;
    cout<<"RANGE_SURVIVE: "<<RANGE_SURVIVE<<endl;
    cout<<"RANGE_AFFECT: "<<RANGE_AFFECT<<endl;
    cout<<"RANGE_EXPLORE: "<<RANGE_EXPLORE<<endl;
    cout<<"DRIVE_BOREDOM: "<<DRIVE_BOREDOM<<endl;
    cout<<"DRIVE_AFFECT: "<<DRIVE_AFFECT<<endl;
    cout<<"DRIVE_SURVIVE: "<<DRIVE_SURVIVE<<endl;
}

bool motivationThread::threadInit() {
    filepath = rf.find("filepath").asString();
    filenameAllData = filepath + filenameAllData;
    filenameAllObjectsMemory = filepath + filenameAllObjectsMemory;
    
    initAllVars();

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

    return true;
}

void motivationThread::startDrivesComputation(){
    if(inputPortEndInitialBehavior.getInputCount()){
        Bottle *endI = nullptr;
        endI = inputPortEndInitialBehavior.read(false);
        if(endI != nullptr){
            computeDrives = true;
            saveData();
        }
    }
}

void motivationThread::run() {
    now = time(0);
    char* timeNow_ = ctime(&now);
    timeNow = timeNow_;
    erase_all(timeNow, "\n");
    
    if(computeDrives == false && (mode == RULE_BASED || mode == DRIVE_BASED))//Check if can start the drives computation until finish the initial state
        startDrivesComputation();

    if(computeDrives){
        updateObjectMemory();
        computeInterestInObjects();

        if(inputPortResetBoredomComfort.getInputCount()){
            Bottle* readR = nullptr;
            readR = inputPortResetBoredomComfort.read(false);
            if(readR != nullptr){
                resetVars();
            }
        }

        computeAffect();
        computeEnergy();
        computeBoredom();

        saveData();
        saveObjectsMemory();
    }
    
    writeAllOutputPorts();

    cout << "-------------------" << endl;
}

void motivationThread::iCubesProcessing(){
    if(inputiCubesPort.getInputCount()){
        dataAllCubes = inputiCubesPort.read(false);
        if(dataAllCubes != NULL){
            for(int i = 0; i < dataAllCubes->size(); i++){
                cout<<"iCube_" <<i<<":  ";
                cout<<dataAllCubes->get(i).asList()->get(0).asInt32()<<",   [ ";
                if(dataAllCubes->get(i).asList()->get(1).asList() != NULL)
                    cout<<dataAllCubes->get(i).asList()->get(1).asList()->get(0).asInt32()<<"  "<<dataAllCubes->get(i).asList()->get(1).asList()->get(1).asInt32()<<"  "<<dataAllCubes->get(i).asList()->get(1).asList()->get(2).asInt32()<<"  "<<dataAllCubes->get(i).asList()->get(1).asList()->get(3).asInt32()<<"  "<<dataAllCubes->get(i).asList()->get(1).asList()->get(4).asInt32()<<"  "<<dataAllCubes->get(i).asList()->get(1).asList()->get(5).asInt32();
                cout<<"],   "<<dataAllCubes->get(i).asList()->get(2).asString()<<endl;
            }
        }else
            cout<<"Perception/iCubes:o DID NOT send data"<<endl;
    }
}

float motivationThread::computeDrive(float need, int homeostasis_base, int range){
    if((need >= (homeostasis_base - range)) && (need <= (homeostasis_base + range)))//Homeostasis
        return 0;
    else if(need < homeostasis_base)
        return -((homeostasis_base - range) - need);//Consider the lowerBound
    else
        return -((homeostasis_base + range) - need);//Consider the upperBound
}

float motivationThread::comfortProcessing(){
    if(inputPortSaturetedAffect.getInputCount()){
        Bottle* inputSatured = nullptr;
        inputSatured = inputPortSaturetedAffect.read(false);
        if(inputSatured != nullptr){
            saturActingTime = 20;
        }
    }

    //The comfort decreases slowly when the robot is recharging (eyes closed)
    if(face_current == NOFACE){
        cout<<"eyes closed"<<endl;
        comfort_current = max(comfort_prev * beta_comfort_Ambivalent_Sleeping, MIN_BOREDOM); 
    }else{
        if(saturActingTime <= 0){//Compute the comfort normally
            if((gaze_current > 0 && face_current >= 0.25) || touch_current > 0)//increase
                //comfort_current = min(((0.5 * touch_current + 0.5 * face_current + 0.5 * gaze_current) + (comfort_prev * tau_comfort_Ambivalent)) / (tau_comfort_Ambivalent + 0.01), MAX_COMFORT);
                comfort_current = min(((touch_current + face_current + gaze_current) + (comfort_prev * tau_comfort_Ambivalent)) / (tau_comfort_Ambivalent + 0.01), MAX_COMFORT);
            else//decrease
                comfort_current = max(comfort_prev * beta_comfort_Ambivalent, MIN_BOREDOM);   
        }else{//Means the comfort is satureted for a while and the robot is executing a behavior to try to solve the drive by itself
            comfort_current = min((0.5 + (comfort_prev * tau_comfort_Ambivalent)) / (tau_comfort_Ambivalent + 0.01), MAX_COMFORT);
            saturActingTime--;
        }
    }

    if(comfort_current < 0.01)//to not go to small
        comfort_current = 0.0;
    
    return comfort_current;
}

/*  for now, just comfort impacts the affect drive
    Comfort decrease in time if no person interacting with the robot and increase when has interaction
    Comfort decrease very slow when the robot is recharging
*/
void motivationThread::computeAffect(){
    if(DRIVE_AFFECT){
        if(inputInteractionPort.getInputCount()){
            Bottle* input = inputInteractionPort.read(true);

            if(input != nullptr){
                face_current = input->get(0).asFloat64();
                gaze_current = input->get(1).asFloat64();
                touch_current = input->get(2).asFloat64();
            }
        }

        comfort_current = comfortProcessing();
        cout<<"Comfort level "<< comfort_current <<endl;

        affectDrive = computeDrive(comfort_current, AFFECT_HOMEOSTASIS, RANGE_AFFECT);

        comfort_prev = comfort_current;
    }else
        affectDrive = 0;//If not using this drive, consider it is already satisfied (to desconsider as an option in the Decision Making)
}

//Battery decrease in time and increase when recharge.
void motivationThread::computeEnergy(){
    if(DRIVE_SURVIVE){
        if(inputBatteryLevelPort.getInputCount()){
            Bottle* inputBattery = inputBatteryLevelPort.read(true);
            batteryLevel = inputBattery->get(0).asFloat64();

            surviveDrive = computeDrive(batteryLevel, SURVIVAL_HOMEOSTASIS, RANGE_SURVIVE);
        }
    }else
        surviveDrive = 0;//If not using this drive, consider it is already satisfied (to desconsider as an option in the Decision Making)
}

//Boredom increase in time and decrease when play with the objects
void motivationThread::computeBoredom(){
    if(DRIVE_BOREDOM){
        if(inputUpdateBoredomPort.getInputCount()){
            Bottle* update = nullptr;
            //update->clear();
            update = inputUpdateBoredomPort.read(false);
            if(update != nullptr){//Decision was play, so update the boredom with the value of the object choosen in the step before
                for(int i = 0; i < allObjectsMemory.size(); i++){
                    allObjectsMemory[i].value = allObjectsMemory[i].value * alpha;//idea: if it is seeing the object, the alpha can be different (so the increase in interest is not so high as the one that it is not seing)
                    cout<<allObjectsMemory[i].color<<" "<<allObjectsMemory[i].value<<endl;
                }
                if(indexObjChoosen != NON_EXIST){
                    allObjectsMemory[indexObjChoosen].value = 0;//TODO = maxValue;
                    playingValue = mostInterestingReward;
                    //boredom = max(MIN_BOREDOM, boredom - mostInterestingReward);
                }
                else
                    playingValue = mostInterestingRewardRandObj;
                    //boredom = max(MIN_BOREDOM, boredom - mostInterestingRewardRandObj);

                playingTime_remaining += playingTime;
                boredom = max(MIN_BOREDOM, boredom - playingValue/playingTime);
                playingTime_remaining--;

                indexObjChoosen = NON_EXIST;
                mostInterestingReward = NON_EXIST;
            }else{//at the moment reducing the boredom is based just in play with toys, otherwise increases
                if(playingTime_remaining > 0){
                    boredom = max(MIN_BOREDOM, boredom - playingValue/playingTime);
                    playingTime_remaining--;
                }else if(face_current == NOFACE){//Increase slowly the boredom if the robot is recharging
                    cout<<"eyes closed"<<endl;
                    boredom = min(MAX_BOREDOM, boredom + INCREASE_BOREDOM * 0.2);
                }else
                    boredom = min(MAX_BOREDOM, boredom + INCREASE_BOREDOM);
            }
        }
        cout<<"Boredom: "<<boredom<<endl;
        exploreDrive = computeDrive(boredom, EXPLORE_HOMEOSTASIS, RANGE_EXPLORE); 
        if(exploreDrive != 0)
            exploreDrive *= (-1);// *-1 because the direction of increase/decrease is the opposite of the other drives
        
    }else
        exploreDrive = 0;//If not using this drive, consider it is already satisfied (to desconsider as an option in the Decision Making)
}

void motivationThread::updateObjectMemory(){
    //At each timestep considers that is not seeing the object and then update it next if it is in the FOV
    for(int i = 0; i < allObjectsMemory.size(); i++)
        allObjectsMemory[i].seeing = false;

    //update objects seen and new ones in the memory
    if(inputAllObjects.getInputCount()){
        Bottle* objectsScene = inputAllObjects.read(false);
        if(objectsScene != nullptr){
            numberOfObjectsScene = objectsScene->get(0).asInt16();
            cout<<"numberOfObjectsScene: "<<numberOfObjectsScene<<endl;
            
            bool newObject;

            for(int i = 0; i < numberOfObjectsScene; i++){
                newObject = true;
                for(int j = 0; j < allObjectsMemory.size(); j++){
                    if(allObjectsMemory[j].color.compare(objectsScene->get(i+1).asList()->get(8).asString()) == 0){
                        allObjectsMemory[j].seeing = true;
                        newObject = false;
                        break;
                    }
                //ignore specific color objects to avoid mistake between the object and the color of the robot's arm (if the robot has other color has to change here)
                }if(newObject && (objectsScene->get(i+1).asList()->get(8).asString().compare(robot_color) != 0)){
                    objects data;
                    data.color = objectsScene->get(i+1).asList()->get(8).asString();
                    data.value = 0;
                    data.seeing = true;
                    allObjectsMemory.push_back(data);
                }
            }
        }
    }
}

//IDEA: change the boredom according to the number of objects in the scene and objects already explored
//use kind of elegibility trace to define this -- think better
void motivationThread::computeInterestInObjects(){
    double reward;
    double objectValue;
    //update interest in each object.TODO: if value == 0 && seeing == false can mean that I forget about the object?
    for(int i = 0; i < allObjectsMemory.size(); i++){
        //objectValue = allObjectsMemory[i].value * alpha;
        if(allObjectsMemory[i].seeing){
            reward = maxValue - allObjectsMemory[i].value;//reward = maxValue - objectValue;
            if(reward > mostInterestingReward){
                indexObjChoosen = i;
                mostInterestingReward = reward;
            }
            cout<<allObjectsMemory[i].color<< " - Reward: "<<reward<<endl;
        }
    }
}

void  motivationThread::resetVars(){
    if(mode.compare(LEARNING_PHASE) == 0 || mode.compare(FINETUNING_PHASE) == 0){
        //Reset boredom
        std::uniform_real_distribution<double> distCur(MIN_BOREDOM, MAX_BOREDOM);
        boredom = distCur(rdx);

        //Reset Comfort
        std::uniform_real_distribution<float> distCom(MIN_COMFORT, MAX_COMFORT);
        comfort_current = distCom(rdx);
        comfort_prev = 0.5;
    }else if(mode.compare(TESTING_PHASE) == 0){
        //default starts in homeostasis(testing phase)
        boredom = EXPLORE_HOMEOSTASIS - RANGE_EXPLORE;
        comfort_current = AFFECT_HOMEOSTASIS + RANGE_AFFECT;//UpperBound
        comfort_prev = 0.5;
    }
}

bool motivationThread::processing(){
    // here goes the processing...
    return true;
}

void motivationThread::saveHeaders(){
    //Save header in the file
    ofstream fout; 
    
    fout.open(filenameAllData, ios::app);
    fout << fileHeaderAllData << "\n";
    fout.close( );

    fout.open(filenameAllObjectsMemory, ios::app); 
    fout << fileHeaderAllObjectsMemory << "\n";
    fout.close( );
}

void motivationThread::saveData(){
    ofstream fout;
    fout.open(filenameAllData, ios::app);
    //expHome,expRang,affHome,affRang,surHome,surRang,exploreDrive,affectDrive,surviveDrive,indexObjChoosen"
    fout << timeNow << ',' << to_string(Time::now() - timeInitial) << ',' << to_string(batteryLevel) << ',' << to_string(numberOfObjectsScene) << ',' << to_string(touch_current) << ',' 
    << to_string(face_current) << ',' << to_string(gaze_current) << ',' << to_string(comfort_current) << ',' << to_string(boredom) << ',' 
    << to_string(EXPLORE_HOMEOSTASIS) << ',' << to_string(RANGE_EXPLORE) <<',' << to_string(AFFECT_HOMEOSTASIS) << ',' << to_string(RANGE_AFFECT)
    <<',' << to_string(SURVIVAL_HOMEOSTASIS) << ',' << to_string(RANGE_SURVIVE) << ','
    << to_string(exploreDrive) << ',' << to_string(affectDrive) << ',' << to_string(surviveDrive) << ',' << to_string(indexObjChoosen);
    
    fout <<"\n";
    fout.close();
}

void motivationThread::saveObjectsMemory(){
    ofstream fout;

    if(allObjectsMemory.size() > 0){
        fout.open(filenameAllObjectsMemory, ios::app);
        for(int i = 0; i < allObjectsMemory.size(); i++){
            fout << timeNow << ',' << to_string(Time::now() - timeInitial) << ',' << allObjectsMemory[i].color << ',' << to_string(allObjectsMemory[i].value) << ',' << allObjectsMemory[i].seeing;
            fout <<"\n";
        }
        fout.close();
    }
}

void motivationThread::writeAllOutputPorts(){
    if(outputExploreDriveAndObject.getOutputCount()){
        Bottle obj;
        obj.clear();
        obj.addFloat64(exploreDrive);
        if(indexObjChoosen != NON_EXIST){
            cout<<"Color choosen: "<<allObjectsMemory[indexObjChoosen].color<<endl;
            obj.addString(allObjectsMemory[indexObjChoosen].color);
        }else
            obj.addString("");
        outputExploreDriveAndObject.prepare() = obj;
        outputExploreDriveAndObject.write();
    }

    if(outputSurvivalDrivePort.getOutputCount()){
        Bottle batteryBottle;
        batteryBottle.clear();
        batteryBottle.addFloat64(surviveDrive);
        outputSurvivalDrivePort.prepare() = batteryBottle;
        outputSurvivalDrivePort.write();
    }

    if(outputAffectDrivePort.getOutputCount()){
        Bottle affectBottle;
        affectBottle.clear();
        affectBottle.addFloat64(affectDrive);
        outputAffectDrivePort.prepare() = affectBottle;
        outputAffectDrivePort.write();
    }

    /// DEBUG
    if(outputBoredomPort.getOutputCount()){
        Bottle boredomBottle;
        boredomBottle.clear();
        boredomBottle.addFloat64(boredom);
        outputBoredomPort.prepare() = boredomBottle;
        outputBoredomPort.write();
    }

    if(outputComfortPort.getOutputCount()){
        Bottle comfortBottle;
        comfortBottle.clear();
        comfortBottle.addFloat64(comfort_current);
        outputComfortPort.prepare() = comfortBottle;
        outputComfortPort.write();
    }
}

void motivationThread::threadRelease() {
    inputAllObjects.interrupt();
    inputBatteryLevelPort.interrupt();
    inputInteractionPort.interrupt();
    outputExploreDriveAndObject.interrupt();
    outputSurvivalDrivePort.interrupt();
    outputAffectDrivePort.interrupt();
    inputUpdateBoredomPort.interrupt();
    inputPortResetBoredomComfort.interrupt();
    inputPortEndInitialBehavior.interrupt();
    inputPortSaturetedAffect.interrupt();
    inputiCubesPort.interrupt();

    outputBoredomPort.interrupt();
    outputComfortPort.interrupt();
    outputBoredomPort.close();
    outputComfortPort.close();

    inputAllObjects.close();
    inputBatteryLevelPort.close();
    inputInteractionPort.close();
    outputExploreDriveAndObject.close();
    outputSurvivalDrivePort.close();
    outputAffectDrivePort.close();
    inputUpdateBoredomPort.close();
    inputPortResetBoredomComfort.close();
    inputPortEndInitialBehavior.close();
    inputPortSaturetedAffect.close();
    inputiCubesPort.close();
}

//-------------- ARTIFICIAL SOLUTION WHEN THE DRIVE IS MAXIMUM FOR A LONG TIME AND "CANNOT" BE SOLVED DURING INTERACTION -----------//
//AFFECT DRIVE "solving"
//measure the time the drive doesnt change (should enter here just when the drive is satureted)
/* if((affectDrive - last_affectDrive) == 0)
    deltaAffectDrive_SatTime++;
else
    deltaAffectDrive_SatTime = 0;

//if the drive is satured for a long time the robot "solve" by itself
if(deltaAffectDrive_SatTime >= 100){
    deltaAffectDrive_SatTime = 0;
    update affect drive in motivation;
}

//BOREDOM DRIVE "solving"
//measure the time the drive doesnt change (should enter here just when the drive is satureted)
if((boredomDrive - last_boredomDrive) == 0)
    deltaBoredomDrive_SatTime++;
else
    deltaBoredomDrive_SatTime = 0;

//if the drive is satured for a long time the robot "solve" by itself
if(deltaBoredomDrive_SatTime >= 100){
    deltaBoredomDrive_SatTime = 0;
    update boredom drive in motivation;
} */
    //-------------- END ARTIFICIAL SOLUTION WHEN THE DRIVE IS MAXIMUM FOR A LONG TIME AND "CANNOT" BE SOLVED DURING INTERACTION -----------//