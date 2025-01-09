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
 * @file batterySensorThread.cpp
 * @brief Implementation of the eventDriven thread (see batterySensorThread.h).
 */

#include <iCub/batterySensorThread.h>
#include <cstring>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace boost;

#define THPERIOD 0.5 //s
#define NON_EXIST -1 //Must be the same value defined in objectPerception module

batterySensorThread::batterySensorThread():PeriodicThread(THPERIOD) {
    robot = "icub";        
}

batterySensorThread::batterySensorThread(string robotname, ResourceFinder &_rf):PeriodicThread(THPERIOD){
    robot = robotname;
    rf = _rf;
}

batterySensorThread::~batterySensorThread() {
    // do nothing
}

void batterySensorThread::setName(string str) {
    this->name=str;
}


std::string batterySensorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void batterySensorThread::setInputPortName(string InpPort) {
    
}

bool batterySensorThread::openAllPorts(){
    if(!inputPortRecharge.open(getName("/recharge:i").c_str())){
        yError("unable to open port to send unmasked events ");
        return false;
    }

    if(!inputUpdBatteryConsPort.open(getName("/updateBatteryConsumption:i").c_str())){
        yError("unable to open port to send unmasked events ");
        return false;
    }

    if(!inputPortResetBattery.open(getName("/resetBattery:i").c_str())){
        yError("unable to open port to send unmasked events ");
        return false;
    }

    if(!outputPortBatteryLevel.open(getName("/batteryLevel:o").c_str())){
        yError("unable to open port");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!outputPortNoBattery.open(getName("/noBattery:o").c_str())){
        yError("unable to open port");
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    return true;
}

bool batterySensorThread::threadInit() {
    filepath = rf.find("filepath").asString();
    filename = filepath + filename;
    
    saveHeaders();

    initVarsFromFile();

    SURVIVAL_HOMEOSTASIS = PERCEN_HOMEOSTASIS * (MAX_BATTERY_LEVEL - MIN_BATTERY_LEVEL) + MIN_BATTERY_LEVEL + RANGE_SURVIVE;//Upperbound homeostasis

    resetBattery();

    printData();

    rechargeTime_remaining = 0;

    if(!openAllPorts())
        return false;

    yInfo("Initialization of the processing thread correctly ended");

    timeInitial = Time::now();

    return true;
}

void batterySensorThread::saveHeaders(){
    //Save header in the file
    ofstream fout; 
    
    fout.open(filename, ios::app);
    fout << fileHeader << "\n";
    fout.close( );
}

void batterySensorThread::saveData(){
    ofstream fout;
    fout.open(filename, ios::app);
    fout << timeNow << ',' << to_string(Time::now() - timeInitial) << ',' << batteryLevel;
    fout <<"\n";
    fout.close();
}

void batterySensorThread::printData(){
    cout<<"Running with:"<<endl;
    cout<<"mode: "<<mode<<endl;
    cout<<"MIN_BATTERY: "<<MIN_BATTERY_LEVEL<<endl;
    cout<<"MAX_BATTERY: "<<MAX_BATTERY_LEVEL<<endl;
    cout<<"VALUE_TO_RECHARGE: "<<VALUE_TO_RECHARGE<<endl;

    cout<<"PERCEN_HOMEOSTASIS: "<<PERCEN_HOMEOSTASIS<<endl;
    cout<<"RANGE_SURVIVE: "<<RANGE_SURVIVE<<endl;

    cout<<"INIT_END_CONS: "<<INIT_END_CONS<<endl;
    cout<<"IDLE_CONS: "<<IDLE_CONS<<endl;
    cout<<"PLAY_CONS: "<<PLAY_CONS<<endl;
    cout<<"RECHARGE_CONS: "<<RECHARGE_CONS<<endl;
    cout<<"INTERACT_CONS: "<<INTERACT_CONS<<endl;

    cout<<"Initial batteryLevel: "<<batteryLevel<<endl;
}

void batterySensorThread::initVarsFromFile(){
    MIN_BATTERY_LEVEL = rf.findGroup("variables").find("MIN_BATTERY").asFloat32();
    MAX_BATTERY_LEVEL = rf.findGroup("variables").find("MAX_BATTERY").asFloat32();
    VALUE_TO_RECHARGE = rf.findGroup("variables").find("VALUE_TO_RECHARGE").asFloat32();

    PERCEN_HOMEOSTASIS = rf.findGroup("variables").find("PERCEN_HOMEOSTASIS_ENERGY").asFloat32();
    RANGE_SURVIVE = rf.findGroup("variables").find("RANGE_SURVIVE").asInt32();

    INIT_END_CONS = rf.findGroup("variables").find("INIT_END_CONS").asFloat32();
    IDLE_CONS = rf.findGroup("variables").find("IDLE_CONS").asFloat32();
    PLAY_CONS = rf.findGroup("variables").find("PLAY_CONS").asFloat32();
    RECHARGE_CONS = rf.findGroup("variables").find("RECHARGE_CONS").asFloat32();
    INTERACT_CONS = rf.findGroup("variables").find("INTERACT_CONS").asFloat32();
    LOOKDOWN_CONS = rf.findGroup("variables").find("LOOKDOWN_CONS").asFloat32();

    mode = rf.findGroup("variables").find("mode").asString();
    if(mode.compare(LEARNING_PHASE) == 0)
        mode = LEARNING_PHASE;
    else if(mode.compare(TESTING_PHASE) == 0)
        mode = TESTING_PHASE;
    else if(mode.compare(FINETUNING_PHASE) == 0)
        mode = FINETUNING_PHASE;
    else
        mode = RULE_BASED;
}

void batterySensorThread::resetBattery(){
    cout<<"Reset battery"<<endl;
    decreaseRate = INIT_END_CONS;

    if(mode.compare(LEARNING_PHASE) == 0 || mode.compare(FINETUNING_PHASE) == 0){
        std::uniform_real_distribution<double> dist(MIN_BATTERY_LEVEL, MAX_BATTERY_LEVEL);
        batteryLevel = dist(rdx);
    }else if(mode.compare(TESTING_PHASE) == 0)
        batteryLevel = SURVIVAL_HOMEOSTASIS;
    else
        batteryLevel = rf.findGroup("variables").check("INIT_BATTERY", Value(MAX_BATTERY_LEVEL)).asFloat64();;
}

double batterySensorThread::recharge(){
    cout<<"Recharge " << batteryLevel + VALUE_TO_RECHARGE << endl;
    return(min(batteryLevel + (VALUE_TO_RECHARGE/rechargeTime), MAX_BATTERY_LEVEL));
}

bool batterySensorThread::noBattery(){
    if(batteryLevel <= MIN_BATTERY_LEVEL)
        return true;
    return false;
}

void batterySensorThread::updateDecreaseRate(string behavior){
    if(behavior.compare("initial") == 0 || behavior.compare("end") == 0)
        decreaseRate = INIT_END_CONS;
    else if(behavior.compare("idle") == 0)
        decreaseRate = IDLE_CONS;
    else if(behavior.compare("play") == 0)
        decreaseRate = PLAY_CONS;
    else if(behavior.compare("recharge") == 0)
        decreaseRate = RECHARGE_CONS;
    else if(behavior.compare("interact") == 0)
        decreaseRate = INTERACT_CONS;
    else if(behavior.compare("lookDown") == 0)
        decreaseRate = LOOKDOWN_CONS;
}

void batterySensorThread::run() {
    now = time(0);
    char* timeNow_ = ctime(&now);
    timeNow = timeNow_;
    erase_all(timeNow, "\n");

    //reset battery level when the episode finishes in RL. Could reset when noBattery() is true, but then could lose control of the data timing in DM
    if(inputPortResetBattery.getInputCount()){
        Bottle* readR = inputPortResetBattery.read(false);
        if(readR != nullptr)
            resetBattery();
    }

    //the decrease rate is according to the activity executed (defined in the decision making module)
    if(inputUpdBatteryConsPort.getInputCount()){
        Bottle* behavior = inputUpdBatteryConsPort.read(false);
        if(behavior != nullptr)
            updateDecreaseRate(behavior->get(0).asString());
    }
    
    cout<<"decreaseRate: "<<decreaseRate<<endl;
    batteryLevel -= decreaseRate;

    if(inputPortRecharge.getInputCount()){
        Bottle* readD = nullptr;
        readD = inputPortRecharge.read(false);
        if(readD != nullptr)
            rechargeTime_remaining += rechargeTime;
            //batteryLevel = recharge();      
    }

    if(rechargeTime_remaining > 0){
        batteryLevel = recharge();
        rechargeTime_remaining--;
    }

    bool noBat = noBattery();

    if(noBat)
        batteryLevel = MIN_BATTERY_LEVEL;//TODO: stop all the application in this case (not if is using RL -- in this case just reset the battery value)

    Bottle battery;
    battery.clear();
    battery.addFloat64(batteryLevel);

    if(outputPortBatteryLevel.getOutputCount()){
        outputPortBatteryLevel.prepare() = battery;
        outputPortBatteryLevel.write();
    }

    Bottle noBatteryBottle;
    noBatteryBottle.clear();
    noBatteryBottle.addInt16(noBat);

    if(outputPortNoBattery.getOutputCount()){
        outputPortNoBattery.prepare() = noBatteryBottle;
        outputPortNoBattery.write();
    }
    cout<<"batteryLevel: "<<batteryLevel<<endl;

    saveData();
}

bool batterySensorThread::processing(){
    // here goes the processing...
    return true;
}

void batterySensorThread::threadRelease() {
    inputPortRecharge.interrupt();
    outputPortBatteryLevel.interrupt();
    inputUpdBatteryConsPort.interrupt();
    outputPortNoBattery.interrupt();
    inputPortResetBattery.interrupt();

    inputPortRecharge.close();
    outputPortBatteryLevel.close();
    inputUpdBatteryConsPort.close();
    outputPortNoBattery.close();
    inputPortResetBattery.close();
}