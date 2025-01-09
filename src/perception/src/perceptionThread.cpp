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
 * @file perceptionThread.cpp
 * @brief Implementation of the eventDriven thread (see perceptionThread.h).
 */

#include <iCub/perceptionThread.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace boost;

#define THPERIOD 0.5 //s
#define NON_EXIST -1

perceptionThread::perceptionThread():PeriodicThread(THPERIOD) {
    robot = "icub";        
}

/* perceptionThread::perceptionThread(string _robot, string _configFile):PeriodicThread(THPERIOD){
    robot = _robot;
    configFile = _configFile;
} */

perceptionThread::perceptionThread(string _robot, ResourceFinder &_rf):PeriodicThread(THPERIOD){
    robot = _robot;
    rf = _rf;
}

perceptionThread::~perceptionThread() {
    // do nothing
}

void perceptionThread::setName(string str) {
    this->name=str;
}


string perceptionThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void perceptionThread::setInputPortName(string InpPort) {
    
}

bool perceptionThread::openAllPorts(){
    /* ======================== Input ========================*/
    /* -------------------- Object Perception -------------------------------*/
    if(!inputPortBlobsListL.open(getName("/blobsListL:i").c_str())){
        yError("unable to open port to receive input");
        return false;
    }

    if(!inputPortBlobsListR.open(getName("/blobsListR:i").c_str())){
        yError("unable to open port to receive input");
        return false;
    }

    /* -------------------- Status eyelids  -------------------------------*/
    if(!inputPortEyelids_icub.open(getName("/statusEyelids:i").c_str())){
        yError("unable to open port to receive input");
        return false;
    }

    /* -------------------- Skin Perception -------------------------------*/
    if(!inputSkinPort.open(getName("/skinTouch:i").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    /* -------------------- Affect Perception -------------------------------*/
    if(!inputAffectPort.open(getName("/affectEval:i").c_str())){
        yDebug("unable to open port to receive affect text");
        return false;
    } 

    /* -------------------- Battery Perception -------------------------------*/
    if(!inputBatteryLevelPort.open(getName("/batteryLevel:i").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!inputPortiCube.open(getName("/iCubeData:i").c_str())){
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    /* ======================== Output ========================*/
    if(!outputBatteryPort.open((getName("/batteryLevel:o").c_str()))){
        yError("unable to open port to receive output");
        return false; 
    }

    if(!outputAllObjectsSeen.open((getName("/allObjectsSeen:o").c_str()))){
        yError("unable to open port to receive output");
        return false; 
    }

    if(!outputGazeFaceSkinPort.open(getName("/gazeFaceSkin:o").c_str())) {
        yError("unable to open port to send processed stimuli ");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if(!outputSkinPort.open(getName("/dataSkin:o").c_str())) {
        yError("unable to open port to send evaluation of skin data");
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if(!outputAffectPort.open(getName("/dataAffect:o").c_str())) {
        yError("unable to open port to send face evaluation of face data");
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if(!outputiCubesPort.open(getName("/iCubes:o").c_str())) {
        yError("unable to open port to send face evaluation of face data");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    return true;
}

void perceptionThread::initAllVars(){
    batteryLevel = -1; //default in case not using battery sensor

    eyesOpen = true;

    //Object Perception
    numberOfObjectsL = 0;
    numberOfObjectsR = 0;

    //Skin
    sideOfTouch = noTouchS;
    originOfTouch = noTouch;

    //Affect
    affectInput = "unreliable";
    faceSuccessInput = 0;
    faceCertInput = 0.0;
    faceXInput = 0.0;
    faceYInput = 0.0;
    faceDepthInput = 0.0;
    focusX = 0.0;
    focusY = 0.0;

    // VARIABLES FOR INTERNAL STATES
    touch_current = 0.0;
    touch_prev = 0.0;
    face_current = 0.0;
    face_prev = 0.0;
    gaze_current = 0.0;
    gaze_prev = 0.0;
}

bool perceptionThread::threadInit(){
    filepath = rf.find("filepath").asString();
    cout<<"filepath: "<<filepath<<endl;
    filenameAllData = filepath + filenameAllData;
    filenameAllObjects = filepath + filenameAllObjects;
    filenameAllICubes = filepath + filenameAllICubes;

    initAllVars();

    saveHeaders();

    if(!openAllPorts())
        return false;

    yInfo("Initialization of the processing thread correctly ended");
    
    timeInitial = Time::now();

    return true;
}

bool perceptionThread::readSkin(){
    // MODIFY FOR IGNORING FINGERS AND PALM
    // https://github.com/robotology/icub-main/blob/master/src/libraries/skinDynLib/include/iCub/skinDynLib/common.h
    
    bool okTouch = false;
    sideOfTouch = noTouchS;
    originOfTouch = noTouch;
         
    Bottle* skinData = inputSkinPort.read(false);

    if(skinData != nullptr){
        string skinInput = skinData->toString();
        cout << skinInput << endl;

        skinData->clear();
        vector<string> fields;

        split(fields, skinInput, is_any_of( " " ), token_compress_on);
        cout << "skin origin: " << fields[0][1] << endl;

        if(fields.size() == 3 && stod(fields[1]) > 5.0 && stod(fields[2]) > 7.0){ //fields.size() == 3 to avoid cases when the robot "feels" a touch that doesnt exist (has the value but not the origin)
            okTouch = true;

            if(fields[0][1] == 't'){//torso
                sideOfTouch = torsoS;
                originOfTouch = torso;
            }

            if(fields[0][1] == 'l'){//left arm
                sideOfTouch = leftS;
                if(fields[0][2] == 'h')//hand 
                    originOfTouch = leftHand;  
                else if(fields[0][2] == 'f')//forearm 
                    originOfTouch = leftForearm;   
                else
                    originOfTouch = leftUpper;//upper arm
            }

            if(fields[0][1] == 'r'){//right arm
                sideOfTouch = rightS;
                if(fields[0][2] == 'h')//hand  
                    originOfTouch = rightHand;  
                else if(fields[0][2] == 'f')//forearm 
                    originOfTouch = rightForearm;   
                else 
                    originOfTouch = rightUpper;//upper arm
            }
        }
    }
    return okTouch;
}

float perceptionThread::processTactileStimuli(){
    if(readSkin())
        touch_current = touchMaxActivation;
    else
        touch_current = alpha_sensors * touch_prev;

    if(touch_current < minThresholdTouch) //0.01, maybe try with 0.25/0.5
        touch_current = 0.0;

    return touch_current;
}

float perceptionThread::processFaceStimuli(){
    //Process stimuli from face - function to read facial expression
    if(affectInput.compare("distant") == 0)
        face_current = DISTANT;
    else if(affectInput.compare("disgusted") == 0)
        face_current = DISGUSTED;
    else if(affectInput.compare("frowning") == 0)
        face_current = FROWNING;
    else if(affectInput.compare("neutral") == 0)
        face_current = NEUTRAL;
    else if(affectInput.compare("contemplating") == 0)
        face_current = CONTEMPLATING;
    else if(affectInput.compare("smiling") == 0)
        face_current = SMILING;
    else
        face_current = alpha_sensors * face_prev;

    if(face_current < maxThresholdFace && face_current > minThresholdFace)
        face_current = 0.0;

    return face_current;
}

float perceptionThread::processGazeStimuli(){
    gazeInput = false;
    if(faceSuccessInput == 1 && faceCertInput > 0.7){//(faceSuccessInput != 0){
        focusX = faceXInput;
        focusY = faceYInput;
        gazeInput = true;
    }

    if(gazeInput)
        gaze_current = gazeMaxActivation;
    else
        gaze_current = alpha_sensors * gaze_prev;

    if(gaze_current < minThresholdGaze) //0.01, maybe try with 0.25/0.5
        gaze_current = 0.0;

    return gaze_current;
}


void perceptionThread::readAffectEvaluation(){
    Bottle* bottleAffect = inputAffectPort.read(false);
    
    if(bottleAffect != nullptr && bottleAffect->size() == 6){
        affectInput = bottleAffect->get(0).asString();
        faceSuccessInput = bottleAffect->get(1).asInt32();
        faceCertInput = bottleAffect->get(2).asFloat64();
        faceXInput = bottleAffect->get(3).asFloat64();
        faceYInput = bottleAffect->get(4).asFloat64();
        faceDepthInput = bottleAffect->get(5).asFloat64();

        if(faceSuccessInput == 1 && faceCertInput > 0.7)
            cout << "FACE CERTAIN\n";
        else{
            cout << "NO FACE\n";
            affectInput = "unreliable";
            faceSuccessInput = 0;
            faceCertInput = 0.0;
            faceXInput = 0.0;
            faceYInput = 0.0;
            faceDepthInput = 0.0;
        }
    }/* else{
        cout<<"No input from bottleAffect"<<endl;
        affectInput = "unreliable";
        faceSuccessInput = 0;
        faceCertInput = 0.0;
        faceXInput = 0.0;
        faceYInput = 0.0;
        faceDepthInput = 0.0;
    } */
}

void perceptionThread::detectObjectsL(){
    inputBlobsListL = inputPortBlobsListL.read(false); //topLeftX, bottomRightX, topLeftY, bottomRightY, color
    if(inputBlobsListL != nullptr){
        numberOfObjectsL = inputBlobsListL->size();

        BlobsImage data;
        for(int i = 0; i < numberOfObjectsL; i++){
            data.topLeftX_leftCam = inputBlobsListL->get(i).asList()->get(0).asInt16();
            data.bottomRightX_leftCam = inputBlobsListL->get(i).asList()->get(1).asInt16();
            data.topLeftY_leftCam = inputBlobsListL->get(i).asList()->get(2).asInt16();
            data.bottomRightY_leftCam = inputBlobsListL->get(i).asList()->get(3).asInt16();
            data.color = inputBlobsListL->get(i).asList()->get(4).asString();
            data.topLeftX_rightCam = NON_EXIST;//not read yet. If the same object (considering the color) exists in the right camera, we'll change this value
            data.bottomRightX_rightCam = NON_EXIST;
            data.topLeftY_rightCam = NON_EXIST;
            data.bottomRightY_rightCam = NON_EXIST;
            imageBlobs_bothCameras.push_back(data);//contain the coordinates of the object considering both cameras. We need objects with different colors to work as expected
        }
    }
}

void perceptionThread::detectObjectsR(){
    inputBlobsListR = inputPortBlobsListR.read(false);//topLeftX, bottomRightX, topLeftY, bottomRightY, color
    if(inputBlobsListR != nullptr){
        numberOfObjectsR = inputBlobsListR->size();
        
        BlobsImage data;
        int exists;

        for (int i = 0; i < numberOfObjectsR; i++){
            exists = 0;
            for (int j = 0; j < numberOfObjectsL; j++){//Check if the same object already exist in the left camera
                if ((imageBlobs_bothCameras[j].color.compare(inputBlobsListR->get(i).asList()->get(4).asString())) == 0){//The object exists in both eyes. 
                    //WARNING: this not work if we have two objects with the same color --> should improve the if maybe with considering distance of the objects
                    imageBlobs_bothCameras[j].topLeftX_rightCam = inputBlobsListR->get(i).asList()->get(0).asInt16();
                    imageBlobs_bothCameras[j].bottomRightX_rightCam = inputBlobsListR->get(i).asList()->get(1).asInt16();
                    imageBlobs_bothCameras[j].topLeftY_rightCam = inputBlobsListR->get(i).asList()->get(2).asInt16();
                    imageBlobs_bothCameras[j].bottomRightY_rightCam= inputBlobsListR->get(i).asList()->get(3).asInt16();
                    exists = 1;
                    break;
                }
            }
            if (!exists){//the object exists just in the right eye
                data.topLeftX_leftCam = inputBlobsListR->get(i).asList()->get(0).asInt16();
                data.bottomRightX_leftCam = inputBlobsListR->get(i).asList()->get(1).asInt16();
                data.topLeftY_leftCam = inputBlobsListR->get(i).asList()->get(2).asInt16();
                data.bottomRightY_leftCam = inputBlobsListR->get(i).asList()->get(3).asInt16();
                data.color = inputBlobsListR->get(i).asList()->get(4).asString();
                data.topLeftX_leftCam = NON_EXIST;
                data.topLeftY_leftCam = NON_EXIST;
                data.bottomRightX_leftCam = NON_EXIST;
                data.bottomRightY_leftCam = NON_EXIST;
                imageBlobs_bothCameras.push_back(data);
            }
        }
    }
}

void perceptionThread::printListOfObjectsL(){
    cout << "All Objects detected LEFT: " << endl;
    cout << "[topLeftX, bottomRightX, topLeftY, bottomRightY, color]" << endl;
    int j;
    for (int i = 0; i < numberOfObjectsL; i++){
        cout << inputBlobsListL->get(i).toString() << endl;//topLeftX, bottomRightX, topLeftY, bottomRightY
        for(j = 0; j < inputBlobsListL->get(i).asList()->size()-1; j++)
            cout << inputBlobsListL->get(i).asList()->get(j).asInt16() << " ";
        cout << inputBlobsListL->get(i).asList()->get(j).asString() << endl;;
    } 
}

void perceptionThread::printListOfObjectsR(){
    cout << "All Objects detected RIGHT: " << endl;
    cout << "[topLeftX, bottomRightX, topLeftY, bottomRightY, color]" << endl;
    int j;
    for (int i = 0; i < numberOfObjectsR; i++){
        cout << inputBlobsListR->get(i).toString() << endl;//topLeftX, bottomRightX, topLeftY, bottomRightY
        for(j = 0; j < inputBlobsListR->get(i).asList()->size()-1; j++)
            cout << inputBlobsListR->get(i).asList()->get(j).asInt16() << " ";
        cout << inputBlobsListR->get(i).asList()->get(j).asString() << endl;
    } 
}

void perceptionThread::printListOfAllObjects(){
    if(imageBlobs_bothCameras.size() == 0)
        cout << "No objects detected" << endl;
    else{
        cout << "All Objects detected: " << endl;
        cout << "[color, topLeftX, topLeftY, bottomRightX, bottomRightY]" << endl;
        for (auto element : imageBlobs_bothCameras)
            cout << element.color << " (" << element.topLeftX_leftCam << ", " << element.topLeftY_leftCam << ", " << element.bottomRightX_leftCam << ", " << element.bottomRightY_leftCam << ") ("
                << element.topLeftX_rightCam << ", " << element.topLeftY_rightCam << ", " << element.bottomRightX_rightCam << ", " << element.bottomRightY_rightCam << ")" <<endl;
    }
}

void perceptionThread::perceptAffect(){
    if(inputAffectPort.getInputCount())
        readAffectEvaluation();
}

float perceptionThread::perceptSkin(){
    if(inputSkinPort.getInputCount()) 
        return processTactileStimuli();
    
    return 0;
}

void perceptionThread::perceptBattery(){
    if(inputBatteryLevelPort.getInputCount()){
        Bottle* inputBattery = inputBatteryLevelPort.read(false);
        if(inputBattery != nullptr)
            batteryLevel = inputBattery->get(0).asFloat64();
    }
}

void perceptionThread::perceptObject(){
    numberOfObjectsL = 0;
    numberOfObjectsR = 0;
    imageBlobs_bothCameras.clear();

    if(inputPortBlobsListL.getInputCount()){
        detectObjectsL();
        printListOfObjectsL(); 
    }

    if(inputPortBlobsListR.getInputCount()){
        detectObjectsR();
        printListOfObjectsR();   
    }
 
    printListOfAllObjects();
}

void perceptionThread::perceptICube(){
    if(inputPortiCube.getInputCount()){
        dataAllCubes = inputPortiCube.read(false);
        if(dataAllCubes != NULL){
            for(int i = 0; i < dataAllCubes->size(); i++){
                cout<<"iCube_" <<i<<":  ";
                cout<<dataAllCubes->get(i).asList()->get(0).asInt32()<<",   [ ";
                if(dataAllCubes->get(i).asList()->get(1).asList() != NULL)
                    cout<<dataAllCubes->get(i).asList()->get(1).asList()->get(0).asInt32()<<"  "<<dataAllCubes->get(i).asList()->get(1).asList()->get(1).asInt32()<<"  "<<dataAllCubes->get(i).asList()->get(1).asList()->get(2).asInt32()<<"  "<<dataAllCubes->get(i).asList()->get(1).asList()->get(3).asInt32()<<"  "<<dataAllCubes->get(i).asList()->get(1).asList()->get(4).asInt32()<<"  "<<dataAllCubes->get(i).asList()->get(1).asList()->get(5).asInt32();
                cout<<"],   "<<dataAllCubes->get(i).asList()->get(2).asString()<<endl;
            }
        }else
            cout<<"iCubeProcessor DID NOT send data"<<endl;
    }else
        cout<<"iCubeProcessor not connected"<<endl;
}

//change the flag "eyesOpen" to true(1) if the eyes are open, false(0) otherwise
void perceptionThread::iCub_eyesOpen(){
    if(inputPortEyelids_icub.getInputCount()){
        Bottle* inputEyelids = inputPortEyelids_icub.read(false);
        if(inputEyelids != nullptr)
            eyesOpen = inputEyelids->get(0).asInt16();
    }
}

void perceptionThread::run(){
    now = time(0);
    char* timeNow_ = ctime(&now);
    timeNow = timeNow_;
    erase_all(timeNow, "\n");

    iCub_eyesOpen();
    cout<<"eyesOpen?: "<<eyesOpen<<endl;

    //Stuff related to vision
    if(eyesOpen){
        //read Facial Expression and generate FaceSuccessInput
        perceptAffect();

        //check if face is still in scene
        face_current = processFaceStimuli();
        gaze_current = processGazeStimuli();

        //process if there are objects in the scene using color segmentation
        perceptObject();
    }else{
        //Not seeing person
        affectInput = "noFace";
        faceSuccessInput = 0;
        faceCertInput = 0.0;
        faceXInput = 0.0;
        faceYInput = 0.0;
        faceDepthInput = 0.0; 
        face_current = NOFACE;
        gaze_current = 0.0;
        //Not seeing objects
        numberOfObjectsL = 0;
        numberOfObjectsR = 0;
        imageBlobs_bothCameras.clear();
    }

    //process stimuli from touch
    touch_current = perceptSkin();

    //read the battery value (battery is a simulated sensor)
    perceptBattery();

    //read if there are iCubes in the setup and if they are being touched
    if(dataAllCubes != NULL)
        dataAllCubes->clear();
    perceptICube();

    cout<<"Face detected "<<face_current<<endl;
    cout<<"Touch detected "<<touch_current<<endl;
    cout<<"Gaze level "<<gaze_current<<endl;

    writeAllOutputPorts();

    saveData();
    saveDataObjects();
    saveDataICubes();

    //time++;
    
    touch_prev = touch_current;
    face_prev = face_current;
    gaze_prev = gaze_current;

    cout<<"----------------------------"<<endl;
}

void perceptionThread::writeAllOutputPorts(){
    if(outputBatteryPort.getOutputCount()){
        Bottle batteryBottle;
        batteryBottle.clear();
        batteryBottle.addFloat64(batteryLevel);
        outputBatteryPort.prepare() = batteryBottle;
        outputBatteryPort.write();
    }

    if(outputAllObjectsSeen.getOutputCount()){
        Bottle allObjSeen;
        Bottle objs;
        
        allObjSeen.clear();
        allObjSeen.addInt16(imageBlobs_bothCameras.size());
        
        if(imageBlobs_bothCameras.size() > 0){    
            for (auto element : imageBlobs_bothCameras){
                objs.clear();
                objs.addInt16(element.topLeftX_leftCam);
                objs.addInt16(element.topLeftY_leftCam);
                objs.addInt16(element.bottomRightX_leftCam);
                objs.addInt16(element.bottomRightY_leftCam);
                objs.addInt16(element.topLeftX_rightCam);
                objs.addInt16(element.topLeftY_rightCam);
                objs.addInt16(element.bottomRightX_rightCam);
                objs.addInt16(element.bottomRightY_rightCam);
                objs.addString(element.color);
                allObjSeen.addList() = objs;
            }
        }
        outputAllObjectsSeen.prepare() = allObjSeen;
        outputAllObjectsSeen.write();
    }
  

    if (outputGazeFaceSkinPort.getOutputCount()){
        Bottle motivationInput;
        motivationInput.clear();
        motivationInput.addFloat64(face_current);
        motivationInput.addFloat64(gaze_current);
        motivationInput.addFloat64(touch_current);
        outputGazeFaceSkinPort.prepare() = motivationInput;
        outputGazeFaceSkinPort.write();
    }

    if (outputSkinPort.getOutputCount()){
        yarp::os::Bottle outputSkin;
        outputSkin.clear();
        outputSkin.addInt16(originOfTouch);
        outputSkin.addInt16(sideOfTouch);
        outputSkinPort.prepare() = outputSkin;
        outputSkinPort.write();
    }

    if (outputAffectPort.getOutputCount()){
        yarp::os::Bottle outputAffect;
        outputAffect.clear();
        outputAffect.addString(affectInput);
        outputAffect.addFloat64(focusX);
        outputAffect.addFloat64(focusY);
        outputAffectPort.prepare() = outputAffect;
        outputAffectPort.write();
    }
}

void perceptionThread::saveHeaders(){
    //Save header in the file
    ofstream fout; 
    
    fout.open(filenameAllData, ios::app);
    fout << fileHeaderAllData << "\n";
    fout.close( );

    fout.open(filenameAllObjects, ios::app); 
    fout << fileHeaderAllObjects << "\n";
    fout.close( );

    fout.open(filenameAllICubes, ios::app); 
    fout << fileHeaderAllICubes << "\n";
    fout.close( );
}

void perceptionThread::saveData(){
    ofstream fout;
    fout.open(filenameAllData, ios::app);
    
    fout << timeNow << ',' << to_string(Time::now() - timeInitial) << ',' << to_string(batteryLevel) << ',' << to_string(touch_current) << ',' << to_string(originOfTouch) << ',' << to_string(sideOfTouch) << ',' <<
        to_string(faceSuccessInput) << ',' <<  to_string(faceCertInput) << ',' << affectInput << ',' << to_string(faceXInput) << ',' << to_string(faceYInput) 
        << ',' << to_string(faceDepthInput) << ',' << to_string(face_current) << ',' << to_string(gaze_current);
    
    fout <<"\n";
    fout.close();
}
                
void perceptionThread::saveDataObjects(){
    ofstream fout;
    
    if(imageBlobs_bothCameras.size() > 0){
        fout.open(filenameAllObjects, ios::app);

        for(int i = 0; i < imageBlobs_bothCameras.size(); i++){
            fout << timeNow << ',' << to_string(Time::now() - timeInitial) << ',' << imageBlobs_bothCameras[i].color << ',' 
                << to_string(imageBlobs_bothCameras[i].topLeftX_leftCam) << ',' << to_string(imageBlobs_bothCameras[i].topLeftY_leftCam) << ',' << to_string(imageBlobs_bothCameras[i].bottomRightX_leftCam) << ',' << to_string(imageBlobs_bothCameras[i].bottomRightY_leftCam) << ',' 
                << to_string(imageBlobs_bothCameras[i].topLeftX_rightCam) << ',' << to_string(imageBlobs_bothCameras[i].topLeftY_rightCam) << ',' << to_string(imageBlobs_bothCameras[i].bottomRightX_rightCam) << ',' << to_string(imageBlobs_bothCameras[i].bottomRightY_rightCam);
            fout <<"\n";
        }
        
        fout.close();
    }   
}

void perceptionThread::saveDataICubes(){
    ofstream fout;
    
    if(dataAllCubes != NULL){
        fout.open(filenameAllICubes, ios::app);
        std::time_t temp = Time::now();

        for(int i = 0; i < dataAllCubes->size(); i++){
            fout << timeNow << ',' << to_string(temp - timeInitial) << ',' << i << ',' << dataAllCubes->get(i).asList()->get(0).asInt32()<<",";
            if(dataAllCubes->get(i).asList()->get(1).asList() != NULL)
                fout<<dataAllCubes->get(i).asList()->get(1).asList()->get(0).asInt32()<<","<<dataAllCubes->get(i).asList()->get(1).asList()->get(1).asInt32()<<","<<dataAllCubes->get(i).asList()->get(1).asList()->get(2).asInt32()<<","<<dataAllCubes->get(i).asList()->get(1).asList()->get(3).asInt32()<<","<<dataAllCubes->get(i).asList()->get(1).asList()->get(4).asInt32()<<","<<dataAllCubes->get(i).asList()->get(1).asList()->get(5).asInt32();
            fout<<dataAllCubes->get(i).asList()->get(2).asString()<<'\n';
        }

        fout.close();
    }
}

bool perceptionThread::processing(){
    // here goes the processing...
    return true;
}


void perceptionThread::threadRelease() {   
    inputPortBlobsListL.interrupt();
    inputPortBlobsListR.interrupt();
    inputSkinPort.interrupt();
    inputBatteryLevelPort.interrupt();
    inputAffectPort.interrupt();
    inputPortEyelids_icub.interrupt();
    inputPortiCube.interrupt();
    outputBatteryPort.interrupt();
    outputAllObjectsSeen.interrupt();
    outputGazeFaceSkinPort.interrupt();
    outputSkinPort.interrupt();
    outputAffectPort.interrupt();
    outputiCubesPort.interrupt();
    
    inputPortBlobsListL.close();
    inputPortBlobsListR.close();
    inputSkinPort.close();
    inputBatteryLevelPort.close();
    inputAffectPort.close();
    inputPortEyelids_icub.close();
    inputPortiCube.close();
    outputBatteryPort.close();
    outputAllObjectsSeen.close();
    outputGazeFaceSkinPort.close();
    outputSkinPort.close();
    outputAffectPort.close();
    outputiCubesPort.close();
}