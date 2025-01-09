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
 * @file sleepingThread.cpp
 * @brief Implementation of the eventDriven thread (see sleepingThread.h).
 */

#include <iCub/sleepingThread.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace boost;

#define THPERIOD 0.5 //s
#define NON_EXIST -1 //Must be the same value defined in objectPerception module

sleepingThread::sleepingThread():PeriodicThread(THPERIOD) {
    robot = "icub";        
}

sleepingThread::sleepingThread(string _robot, ResourceFinder &_rf):PeriodicThread(THPERIOD){
    robot = _robot;
    //configFile = _configFile;
    rf = _rf;
}

sleepingThread::sleepingThread(string _robot, ResourceFinder &_rf, string _robotPlatform):PeriodicThread(THPERIOD){
    robot = _robot;
    //configFile = _configFile;
    rf = _rf;
    robotPlatform = _robotPlatform;
}

sleepingThread::~sleepingThread() {
    // do nothing
}

void sleepingThread::setName(string str) {
    this->name=str;
}


std::string sleepingThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void sleepingThread::setInputPortName(string InpPort) {
    
}

bool sleepingThread::openAllPorts(){
    //speech port - AcapelaSpeak
    if(!outputSpeechPort.open(getName("/speech:o").c_str())){
        yDebug("unable to open port to send speech text");
        return false;
    }

    //facial LED controller
    if(!outputEmotionsPort.open(getName("/cmdFace:rpc").c_str())){
        yDebug("unable to open port to send emotions LED");
        return false;
    }

    //ctpService port Right Arm
    outputMovementRAPort.setRpcMode(true);
    yDebug("OPENING for right arm connection");
    if(!outputMovementRAPort.open(getName("/movement/right_arm").c_str())){
        yDebug("unable to open port to send right arm commands");
        return false;
    }

    //ctpService port Left Arm
    outputMovementLAPort.setRpcMode(true);
    yDebug("OPENING for left arm connection");
    if(!outputMovementLAPort.open(getName("/movement/left_arm").c_str())){
        yDebug("unable to open port to send left arm commands");
        return false;
    }
    
    //ctpService port torso
    outputMovementTPort.setRpcMode(true);
    yDebug("OPENING for torso connection");
    if(!outputMovementTPort.open(getName("/movement/torso").c_str())){
        yDebug("unable to open port to send torso commands");
        return false;
    }
    
    //ctpService port Eyelids
    if(robotPlatform == BERRY_ROBOT){
        outRpcEyelids_berry.setRpcMode(true);
        yInfo("OPENING port for eyelids control");
        if(!outRpcEyelids_berry.open(getName("/movement/face").c_str())){
            yInfo("unable to open port");
            return false;
        }
    }

    //port Emotions -- eyelids
    if(robotPlatform == REDDY_ROBOT){
        outRpcEyelids_reddy.setRpcMode(true);
        yInfo("OPENING port for emotion control");
        if(!outRpcEyelids_reddy.open(getName("/emotions/out").c_str())){
            yInfo("unable to open port");
            return false;
        }
    }
 
    return true;
}

bool sleepingThread::waitForPortConnections(){
    if(robotPlatform == BERRY_ROBOT || robotPlatform == REDDY_ROBOT){
        while(outputSpeechPort.getOutputCount() < 1);
        while(outputEmotionsPort.getOutputCount() < 1);
    }

    if(robotPlatform == BERRY_ROBOT)
        while(outRpcEyelids_berry.getOutputCount() < 1);

    if(robotPlatform == REDDY_ROBOT)
        while(outRpcEyelids_reddy.getOutputCount() < 1);

    while(outputMovementRAPort.getOutputCount() < 1);
    while(outputMovementLAPort.getOutputCount() < 1);
    while(outputMovementTPort.getOutputCount() < 1);
 
    return true;
}


bool sleepingThread::threadInit() {
    init = 0;

    if(!openAllPorts())
        return false;

    cout<<"robotPlatform: "<<robotPlatform<<endl;

    yInfo("Initialization of the processing thread correctly ended");

    yInfo("Wait for connections");
    if(!waitForPortConnections()){
        yError("Connection Error");
        return false;
    }

    return true;
}

void sleepingThread::run() {
    if(init == 0){
        init++;
        executeMovement(2.0, 0.0, " ", "SaraHome");
        setFaceModular("neutral", "neutral");
        moveEyelids(1.0, 0.0, "close");
    }
    Time::delay(1);
    speakText("#SLEEP02#");
    Time::delay(1);

    cout << "-------------------" << endl;
}

void sleepingThread::speakText(string speech){
    // send text to acapelaSpeak
    if (outputSpeechPort.getOutputCount()){
        Bottle bottleToSendSpeech;
        bottleToSendSpeech.clear();
        bottleToSendSpeech.addString(speech);
        outputSpeechPort.prepare() = bottleToSendSpeech;
        outputSpeechPort.writeStrict();
    }
}

vector<string> getValuesLEDSModular(string affectEyebr, string affectMouth){
    vector<string> valueLEDS;
    valueLEDS.push_back("");
    valueLEDS.push_back("");
    valueLEDS.push_back("");
   
    if(affectEyebr.compare("raise") == 0){
        valueLEDS[0] = "L01"; //04
        valueLEDS[1] = "R01"; //04
    }else if(affectEyebr.compare("lower") == 0){
        valueLEDS[0] = "L04"; //01
        valueLEDS[1] = "R04"; //01
    }else if(affectEyebr.compare("neutral") == 0){
        valueLEDS[0] = "L02"; //02
        valueLEDS[1] = "R02"; //02
    }else if(affectEyebr.compare("evil") == 0){
        valueLEDS[0] = "L05"; //02
        valueLEDS[1] = "R05"; //02
    }else if(affectEyebr.compare("cunny") == 0){
        valueLEDS[0] = "L07"; 
        valueLEDS[1] = "R07"; 
    }

    if(affectMouth.compare("smile") == 0)
        valueLEDS[2] = "M01"; //0B
    else if(affectMouth.compare("frown") == 0)
        valueLEDS[2] = "M02";//"M38";
    else if(affectMouth.compare("open") == 0)      
        valueLEDS[2] = "M03"; //16
    else if(affectMouth.compare("sad") == 0)
        valueLEDS[2] = "M02";//"M18";
    else if(affectMouth.compare("neutral") == 0)
        valueLEDS[2] = "M00";//M0A, 08
    else if(affectMouth.compare("evil") == 0)
        valueLEDS[2] = "M05";//M0A, 08
    else if(affectMouth.compare("cunny") == 0)
        valueLEDS[2] = "M07";//M0A, 08
    
    return valueLEDS;
}

void sleepingThread::setFaceModular(string affectEyebr, string affectMouth){
    vector<string> vals;

    vals = getValuesLEDSModular(affectEyebr, affectMouth);

    string lebVal = vals[0];
    string rebVal = vals[1];
    string mouVal = vals[2];
    
    Bottle out;

    if(outputEmotionsPort.getOutputCount()){
        out.fromString(lebVal);
        outputEmotionsPort.write(out);
        //Time::delay(0.05);
        out.clear();

        out.fromString(rebVal);
        outputEmotionsPort.write(out);
        //Time::delay(0.05);
        out.clear();

        out.fromString(mouVal);
        outputEmotionsPort.write(out);
    }
}

void sleepingThread::executeMovement(const double time, const double offset, const string bodyPart, const string movement){
    // send name of gesture to perform, joint positions and body parts are loaded from .ini file
    // processing the config file
    Bottle listOfJointPos = rf.findGroup("movement").findGroup(movement).findGroup("seq").tail();
    Bottle timeNew = rf.findGroup("movement").findGroup(movement).findGroup("time").tail();
    Bottle offsetNew = rf.findGroup("movement").findGroup(movement).findGroup("offset").tail();
    Bottle bodyPartNew = rf.findGroup("movement").findGroup(movement).findGroup("bodyPart").tail();

    string tmpJointPos;
    string tmpBodyPart;

    string testInput = "input";

    //for each separate gesture
    for (int i = 0; i < listOfJointPos.size(); ++i){
        tmpJointPos = listOfJointPos.get(i).asString();
        Bottle tmpList = rf.findGroup("movement").findGroup(tmpJointPos).tail();

        Bottle cmd;
        cmd.clear();
        cmd.addVocab32("ctpq");
        cmd.addVocab32("time");
        cmd.addFloat64(time);
        cmd.addVocab32("off");
        cmd.addFloat64(offset);
        cmd.addVocab32("pos");
        cmd.addList() = rf.findGroup("movement").findGroup(tmpJointPos).tail();

        Bottle response;
        response.clear();

        if(testInput.compare(bodyPartNew.get(i).asString()) != 0)       
            tmpBodyPart = bodyPartNew.get(i).asString();
        else
            tmpBodyPart = bodyPart;
        
        if(tmpBodyPart.compare("rightArm") == 0){ 
            cout << "Right Arm" << endl;
            if (outputMovementRAPort.getOutputCount()) 
                outputMovementRAPort.write(cmd,response);
        }
        
        if(tmpBodyPart.compare("leftArm") == 0){
            cout << "Left Arm" << endl;
            if(outputMovementLAPort.getOutputCount()) 
                outputMovementLAPort.write(cmd,response);
        }
        
        if(tmpBodyPart.compare("torso") == 0){ 
            cout << "Torso" << endl;
            if(outputMovementTPort.getOutputCount())                        
                outputMovementTPort.write(cmd,response);
        }
    }
}

void sleepingThread::moveEyelids(const double time, const double offset, string movement){
    if(robotPlatform == REDDY_ROBOT){// Reddy controls the eyelids from the emotionInterface
        Bottle cmd;
        Bottle response;

        cmd.clear();
        response.clear();
        cmd.addVocab32("set");
        cmd.addVocab32("raw");
        cmd.addVocab32(Vocab32::encode(getEyelidsPose(movement).toString()));

        cout<<cmd.toString()<<endl;
        outRpcEyelids_reddy.write(cmd, response);
    }else if(robotPlatform == BERRY_ROBOT){//Berry controls the eyelids from the ctpService
        Bottle cmd;
        Bottle response;
        cmd.clear();
        response.clear();

        cmd.addVocab32("ctpq");
        cmd.addVocab32("time");
        cmd.addFloat64(time);
        cmd.addVocab32("off");
        cmd.addFloat64(offset);
        cmd.addVocab32("pos");
        cmd.addList() = getEyelidsPose(movement);

        cout<<cmd.toString()<<endl;
        outRpcEyelids_berry.write(cmd, response);
    }
}

Bottle sleepingThread::getEyelidsPose(string movement){
    Bottle bEyelides;
    bEyelides.clear();

    if(robotPlatform == REDDY_ROBOT){
        if(movement == "open")
            bEyelides.addString("S01");
        else
            bEyelides.addString("S64");
    }else if(robotPlatform == BERRY_ROBOT){
        if(movement == "open")
            bEyelides.addInt16(1);
        else
            bEyelides.addInt16(64);
    }

    return bEyelides;
}

bool sleepingThread::processing(){
    // here goes the processing...
    return true;
}

void sleepingThread::threadRelease(){
    outputMovementRAPort.interrupt();
    outputMovementLAPort.interrupt();
    outputMovementTPort.interrupt();
    outputSpeechPort.interrupt();
    outputEmotionsPort.interrupt();
    outRpcEyelids_berry.interrupt();
    outRpcEyelids_reddy.interrupt();

    outputMovementRAPort.close();
    outputMovementLAPort.close();
    outputMovementTPort.close();
    outputSpeechPort.close();
    outputEmotionsPort.close();
    outRpcEyelids_berry.close();
    outRpcEyelids_reddy.close();
}