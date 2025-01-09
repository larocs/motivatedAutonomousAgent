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
 * @file actionThread.cpp
 * @brief Implementation of the eventDriven thread (see actionThread.h).
 */

#include <iCub/actionThread.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace boost;

#define THPERIOD 0.5 //s
#define NON_EXIST -1 //Must be the same value defined in objectPerception module

actionThread::actionThread():PeriodicThread(THPERIOD) {
    robot = "icub";        
}

actionThread::actionThread(string _robot, ResourceFinder &_rf):PeriodicThread(THPERIOD){
    robot = _robot;
    //configFile = _configFile;
    rf = _rf;
}

actionThread::actionThread(string _robot, ResourceFinder &_rf, string _robotPlatform):PeriodicThread(THPERIOD){
    robot = _robot;
    //configFile = _configFile;
    rf = _rf;
    robotPlatform = _robotPlatform;
}

actionThread::~actionThread() {
    // do nothing
}

void actionThread::setName(string str) {
    this->name=str;
}


std::string actionThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void actionThread::setInputPortName(string InpPort) {
    
}

bool actionThread::openAllPorts(){
    if(!inputCommandSMPort.open(getName("/commandSM:i").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }
    inputCommandSMPort.setStrict();
    
    //Send a message to the simulated sensor to simulate the recharging action (and update the battery value)
    if(!outputPortRecharge.open(getName("/recharge:o").c_str())){
        yError("unable to open port to send unmasked events ");
        return false;
    }

    //Send action to ARE
    if(!outputPortObjectToActAt.open(getName("/actionTargetARE:o").c_str())){
        yError("unable to open port to send unmasked events ");
        return false;  // unable to open; let RFModule know so that it won't run
    }

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

    yInfo("OPENING port for eyelids control");
    if(!outputPortEyelids_icub.open(getName("/statusEyelids:o").c_str())){
        yInfo("unable to open port");
        return false;
    }
    

    return true;
}

void actionThread::initAllVars(){
    turnFlag = 0;
    timePastIdle = Time::now();

    middleX_leftCam = -1;
    middleY_leftCam = -1;
    middleX_rightCam = -1;
    middleY_rightCam = -1;
    
    iGaze = NULL;
    clientGazeCtrl = NULL;

    yInfo("Opening the connection to the iKinGaze");
    optGaze.put("device", "gazecontrollerclient");
    optGaze.put("remote", "/iKinGazeCtrl");
    optGaze.put("local", "/client/gaze");
    
    clientGazeCtrl = new PolyDriver();
    clientGazeCtrl->open(optGaze);
}

bool actionThread::waitForPortConnections(){
    yInfo("Connecting to the iKinGaze");
    if (clientGazeCtrl->isValid()){
        clientGazeCtrl->view(iGaze);
        yInfo("Connected to the iKinGaze");
    }else{
        yError("Unable to connect to the iKinGaze");
        return false;
    }

    changeIGazeSpeed();
    
    while(inputCommandSMPort.getInputCount() < 1);
    //while(outputPortObjectToActAt.getOutputCount() < 1);
    while(outputPortRecharge.getOutputCount() < 1);

    if(robotPlatform == BERRY_ROBOT || robotPlatform == REDDY_ROBOT){
        while(outputSpeechPort.getOutputCount() < 1);
        while(outputEmotionsPort.getOutputCount() < 1);
    }

    if(robotPlatform == BERRY_ROBOT)
        while(outRpcEyelids_berry.getOutputCount() < 1);

    if(robotPlatform == REDDY_ROBOT)
        while(outRpcEyelids_reddy.getOutputCount() < 1);

    while(outputPortEyelids_icub.getOutputCount() < 1);

    while(outputMovementRAPort.getOutputCount() < 1);
    while(outputMovementLAPort.getOutputCount() < 1);
    while(outputMovementTPort.getOutputCount() < 1);
 
    return true;
}

bool actionThread::threadInit() {
    filepath = rf.find("filepath").asString();
    filenameARE = filepath + filenameARE;
    filenameAllData = filepath + filenameAllData;

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

void actionThread::run() {    
    now = time(0);
    char* timeNow_ = ctime(&now);
    timeNow = timeNow_;
    erase_all(timeNow, "\n");

    colorToPlay = "";
    readCommandSM();

    cout << "-------------------" << endl;
}

void actionThread::changeIGazeSpeed(){
    double eyesTime, neckTime;

    iGaze->getEyesTrajTime(&eyesTime);
    iGaze->getNeckTrajTime(&neckTime);
    cout << eyesTime << endl;
    cout << neckTime << endl;

    cout << iGaze->blockNeckRoll() << endl; //block roll, use only pitch and yaw, start the robot in normal orientation
    cout << iGaze->setEyesTrajTime(0.7) << endl; //0.5 pilot, 0.2 orig, 0.7
    cout << iGaze->setNeckTrajTime(0.9) << endl; //0.9 pilot, 0.8 orig, 0.9

    iGaze->getEyesTrajTime(&eyesTime);
    iGaze->getNeckTrajTime(&neckTime);
    cout << eyesTime << endl;
    cout << neckTime << endl;
}


void actionThread::readCommandSM(){
    if(inputCommandSMPort.getInputCount()){
        Bottle* bottleCommandSM = nullptr;
        bottleCommandSM = inputCommandSMPort.read(false);

        if(bottleCommandSM != nullptr){
            string commandData = bottleCommandSM->toString();
            bottleCommandSM->clear();
            vector<string> fields;

            cout << "This is the input: "<< commandData << endl;
            erase_all(commandData, "\"");
            split(fields, commandData, is_any_of( "," ), token_compress_on);

            string actionType = fields [0];
            cout << "action type is: " << actionType << endl;

            if(actionType.compare("recharge") == 0){
                cout<<"Recharging"<<endl;
                if(outputPortRecharge.getOutputCount()){//Send a message to the simulated sensor to simulate the recharging action (and update the battery value)
                    Bottle rechargeBottle;
                    rechargeBottle.clear();           
                    rechargeBottle.addInt16(1);
                    outputPortRecharge.prepare() = rechargeBottle;
                    outputPortRecharge.write();
                }
            }else if(actionType.compare("eyelids") == 0){
                moveEyelids(stod(fields[1]), stod(fields[2]), fields[3]);
            }else if(actionType.compare("homeARE") == 0){
                homeARE();
                writeToARE();
            }
            else if(actionType.compare("powerOff") == 0){

            }else if(actionType.compare("play") == 0){
                //topXLeft, topYLeft, bottomXLeft, bottomYLeft, topXRight, topYRight, bottomXRight, bottomYRight;
                colorToPlay = fields[9];
                computeTargetCentroid(stoi(fields[1]), stoi(fields[2]), stoi(fields[3]), stoi(fields[4]), stoi(fields[5]), stoi(fields[6]), stoi(fields[7]), stoi(fields[8]));
                actARE("point", middleX_leftCam, middleY_leftCam, middleX_rightCam, middleY_rightCam);
                writeToARE();
            }else if(actionType.compare("speech") == 0)
                speakText(fields[1]);
            else if(actionType.compare("face") == 0)
                setFaceModular(fields[1], fields[2]);
            else if(actionType.compare("gaze") == 0){ // "gaze functionName functionParameter"
                string functionName = fields [1];
                cout<<"The function Name is: "<<functionName<<endl;

                if(functionName.compare("turnHeadAngle") == 0)
                    turnHeadAngle(stof(fields[2]));
                
                else if(functionName.compare("lookAtPoint") == 0){
                    if(fields.size() == 5)//gaze lookAtPoint pointX pointY camera
                        lookAtPointMono(stoi(fields[2]), stoi(fields[3]), stoi(fields[4]));
                    else //gaze lookAtPoint pointX_Left pointY_Left pointX_Right pointY_Right
                        lookAtPointStereo(stoi(fields[2]), stoi(fields[3]), stoi(fields[4]), stoi(fields[5]));
                }

                else if(functionName.compare("lookAround") == 0)
                    lookAround();
                
                else if(functionName.compare("turnHeadTwoAngles") == 0)
                    turnHeadTwoAngles(stof(fields[2]), stof(fields[3]));
                
                else if(functionName.compare("moveHeadDirection") == 0)
                    moveHeadDirection(fields[2]);
            }else if(actionType.compare("action") == 0)
                executeMovement(stod(fields[1]), stod(fields[2]), " ", fields[3]);
            
            saveData(actionType);
        } 
    }
}

void actionThread::writeToARE(){
    cout<<"Before ARE"<<endl;
    if(outputPortObjectToActAt.getOutputCount()){
        cout<<action.toString()<<endl;
        //outputPortObjectToActAt.prepare() = action;
        //outputPortObjectToActAt.write();
        Bottle reply;
        outputPortObjectToActAt.write(action, reply);
        cout<<"Reply ARE: "<<reply.toString()<<endl;
    }
    changeIGazeSpeed();//ARE overwrites the iGaze speed defined in init - so need to redefine
}

void actionThread::turnHeadAngle(float angle){ 
    Vector ang(3);

    if (iGaze != NULL){
        iGaze->getAngles(ang);
        yDebug("angles before rotate x=%f, y=%f, z=%f", ang[0], ang[1], ang[2]);

        yInfo("Turning head to %f", (angle));
        ang[0] += angle;
        iGaze->lookAtAbsAngles(ang);
        //iGaze->waitMotionDone();

        iGaze->getAngles(ang);
        yDebug("angles after rotate x=%f, y=%f, z=%f", ang[0], ang[1], ang[2]);
    }
}

void actionThread::lookAround(){
   cout<<"Looking around"<<endl;
   //Natural movement 
    int rotationAngle = (1 - 2*(rand() % 2)) * 10.0;
    float timeElapsedGaze = 3 + (rand() % 3);  //random time between 2.5 and 4.5 seconds (5-7)

    if(Time::now() - timePastIdle > timeElapsedGaze){ // try out longer periods after movement
        if(turnFlag < 3){
            turnHeadAngle(rotationAngle);
            turnFlag++;
        }
        else{
            moveHeadDirection("home");
            turnFlag = 0;
        }
        timePastIdle = Time::now();
    }        
}

void actionThread::turnHeadTwoAngles(float angleX, float angleY){
    Vector ang(3);

    if (iGaze != NULL){
        iGaze->getAngles(ang);
        yDebug("angles before rotate x=%f, y=%f, z=%f", ang[0], ang[1], ang[2]);

        yInfo("Turning head to %f, %f", (angleX), (angleY));
        ang[0] += angleX;//Azimuthal (horizontal) 
        ang[1] += angleY;//elevation (vertical) 
        iGaze->lookAtAbsAngles(ang);
        //iGaze->waitMotionDone();

        iGaze->getAngles(ang);
        yDebug("angles after rotate x=%f, y=%f, z=%f", ang[0], ang[1], ang[2]);
    }
}

void actionThread::lookAtPointMono(int pointX, int pointY, int camera){
    Vector px(2);   // specify the pixel where to look

    px[0] = pointX;
    px[1] = pointY;

    if(iGaze != NULL)
        iGaze->lookAtMonoPixel(camera, px, z);  //left camera = 0, right camera = 1
}

void actionThread::lookAtPointStereo(int pointX_Left, int pointY_Left, int pointX_Right, int pointY_Right){
    Vector pxL(2);   // specify the pixel where to look
    Vector pxR(2);

    pxL[0] = pointX_Left;
    pxL[1] = pointY_Left;    

    pxR[0] = pointX_Right;
    pxR[1] = pointY_Right;

    if(iGaze != NULL)
        iGaze->lookAtStereoPixelsSync(pxL, pxR);
        //igaze->waitMotionDone();                        // wait until the operation is done  
}

void actionThread::speakText(string speech){
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

void actionThread::setFaceModular(string affectEyebr, string affectMouth){
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

void actionThread::computeTargetCentroid(int topXLeft, int topYLeft, int bottomXLeft, int bottomYLeft, int topXRight, int topYRight, int bottomXRight, int bottomYRight){
    middleX_leftCam = (topXLeft + bottomXLeft)/2;
    middleY_leftCam = (topYLeft + bottomYLeft)/2;

    middleX_rightCam = (topXRight + bottomXRight)/2;
    middleY_rightCam = (topYRight + bottomYRight)/2;

    cout<<middleX_leftCam<<" "<<middleY_leftCam<<endl;
    cout<<middleX_rightCam<<" "<<middleY_rightCam<<endl;
}

 //-----------------------------------------------------------------------
void actionThread::moveHeadDirection(string direction){
    Vector ang(3);
    
    if(iGaze != NULL){
        iGaze->getAngles(ang);
        //yDebug("angles before move x=%f, y=%f, z=%f", ang[0], ang[1], ang[2]);

        if(direction == "up"){
            yInfo("Moving head UP");
            ang[1] += 10;
        }
        else if(direction == "down"){
            yInfo("Moving head DOWN");
            ang[1] -= 10;
        }
        else if(direction == "left"){
            yInfo("Moving head LEFT");
            ang[0] -= 10; //20
        }
        else if(direction == "right"){
            yInfo("Moving head RIGHT");
            ang[0] += 10;
        }
        else if(direction == "home"){
            yInfo("head home");
            ang[0] = 0.0;
            ang[1] = 0.0;
            ang[2] = 0.0;
        }
        else if(direction == "homeDown"){
            yInfo("head home down");
            ang[0] = 0.0;
            ang[1] = -25.00;
            ang[2] = 0.0;
        }
        else if(direction == "leftDown"){
            yInfo("head home left down");
            ang[0] = -15.0;
            ang[1] = -30.0;
            ang[2] = 0.0;
        }
        else if(direction == "rightDown"){
            yInfo("head home right down");
            ang[0] = 15.0;
            ang[1] = -30.0;
            ang[2] = 0.0;
        }
        else if(direction == "leftDownUpper"){
            yInfo("head home left down");
            ang[0] = -25.0;
            ang[1] = -30.0;
            ang[2] = 0.0;
        }
        else if(direction == "rightDownUpper"){
            yInfo("head home right down");
            ang[0] = 25.0;
            ang[1] = -30.0;
            ang[2] = 0.0;
        }
        else if(direction == "rightUp"){
            yInfo("head home right up");
            ang[0] = 35.0;
            ang[1] = 15.0;
            ang[2] = 0.0;
        }
        else if(direction == "farDiagonalRight_Down"){//Define correct values
            yInfo("head far Diagonal Right");
            ang[0] = 30.0;
            ang[1] = -25.0;
            ang[2] = 0.0;
        }
        else if(direction == "closeDiagonalRight_Down"){
            yInfo("head close Diagonal Right");
            ang[0] = 30.0;
            ang[1] = -35.0;
            ang[2] = 0.0;
        }
        else if(direction == "center_Down"){
            yInfo("head center");
            ang[0] = 0.0;
            ang[1] = -35.0;
            ang[2] = 0.0;
        }
        else if(direction == "farDiagonalLeft_Down"){
            yInfo("head far Diagonal Left");
            ang[0] = -35.0;
            ang[1] = -20.0;
            ang[2] = 0.0;
        }
        else if(direction == "closeDiagonalLeft_Down"){
            yInfo("head closeDiagonalLeft");
            ang[0] = -40.0;
            ang[1] = -35.0;
            ang[2] = 0.0;
        }
        iGaze->lookAtAbsAngles(ang);
        iGaze->waitMotionDone();
/*         iGaze->getAngles(ang);
        yDebug("angles after move x=%f, y=%f, z=%f", ang[0], ang[1], ang[2]); */
    }
}

void actionThread::actARE(string actionARE, int pXL, int pYL, int pXR, int pYR){
    action.clear();
    Vector pixels(2);
    
    int cmd = Vocab32::encode(actionARE);//point
    action.addVocab32(cmd);

    if(pXL == NON_EXIST && pYL == NON_EXIST){
        pixels[0] = pXR;
        pixels[1] = pYR;
        action.addString("right");
    }else{//left camera is the default
        pixels[0] = pXL;
        pixels[1] = pYL;
    }

    /*yarp::sig::Vector pos(3);
    pos[0]=-0.35;
    pos[1]=+0.15;
    pos[2]=+0.0;
    action.addList().read(pos);*/

    action.addList().read(pixels);
    //action.addString("still");// prevents the robot from bringing back home the arm after having accomplished the action.
    //action.addString("left");// arm required for the action 
    
    saveDataARE(pXL, pYL, pXR, pYR);
 }

 void actionThread::homeARE(){
    action.clear();
    int cmd = Vocab32::encode("home");
    action.addVocab32(cmd);
    action.addString("all");
 }

void actionThread::executeMovement(const double time, const double offset, const string bodyPart, const string movement){
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


void actionThread::saveDataARE(int pXL, int pYL, int pXR, int pYR){
    ofstream fout;
    fout.open(filenameARE, ios::app); 

    fout << timeNow << ',' << to_string(Time::now() - timeInitial) << ',' << colorToPlay << "," << to_string(pXL) << "," << to_string(pYL) << "," << to_string(pXR) << "," << to_string(pYR);
    fout << "\n";

    fout.close( );
}


void actionThread::moveEyelids(const double time, const double offset, string movement){
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
    outputPortEyelids_icub.prepare() = updateStatusEyelids_ToPerception(movement);
    outputPortEyelids_icub.write();
}

Bottle actionThread::updateStatusEyelids_ToPerception(string movement){
    Bottle bEyelides;
    bEyelides.clear();

    if(movement == "open")
        bEyelides.addInt16(1);
    else
        bEyelides.addInt16(0);

    return bEyelides;
}

Bottle actionThread::getEyelidsPose(string movement){
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

void actionThread::saveHeaders(){
    //Save header in the file
    ofstream fout; 
    
    fout.open(filenameAllData, ios::app);
    fout << fileHeaderAllData << "\n";
    fout.close();

    fout.open(filenameARE, ios::app);
    fout << fileHeaderARE << "\n";
    fout.close();
}

void actionThread::saveData(string actionType){
    ofstream fout;
    fout.open(filenameAllData, ios::app);
    
    fout << timeNow << ',' << to_string(Time::now() - timeInitial) << ',' << actionType;

    fout << "\n";
    fout.close();
}

bool actionThread::processing(){
    // here goes the processing...
    return true;
}

void actionThread::threadRelease(){
    inputCommandSMPort.interrupt();
    outputPortRecharge.interrupt();
    outputPortObjectToActAt.interrupt();
    outputMovementRAPort.interrupt();
    outputMovementLAPort.interrupt();
    outputMovementTPort.interrupt();
    outputSpeechPort.interrupt();
    outputEmotionsPort.interrupt();
    outRpcEyelids_berry.interrupt();
    outRpcEyelids_reddy.interrupt();
    outputPortEyelids_icub.interrupt();

    inputCommandSMPort.close();
    outputPortRecharge.close();
    outputPortObjectToActAt.close();
    outputMovementRAPort.close();
    outputMovementLAPort.close();
    outputMovementTPort.close();
    outputSpeechPort.close();
    outputEmotionsPort.close();
    outRpcEyelids_berry.close();
    outRpcEyelids_reddy.close();
    outputPortEyelids_icub.close();
}

/*

https://robotology.github.io/robotology-documentation/doc/html/group__actionsRenderingEngine.html

https://github.com/robotology/icub-main/issues/370

https://github.com/robotology/icub-main/blob/master/app/actionsRenderingEngine/scripts/actionsRenderingEngine.xml.template

yarp rpc /actionsRenderingEngine/cmd:io
>>touch (-0.35 0.15 0.0)
Response: [ack]
>>touch (-0.35 0.15 0.0)
Response: [ack]
>>point (-0.35 0.15 0.0)
Response: [ack]
>>grasp (-0.35 0.15 0.0)
Response: [nack]
>>calibration table
Response: [nack]
>>push (-0.35 0.15 0.0)
Response: [ack]
>>look (-0.35 0.15 0.0)
Response: [ack]
>>home

*/