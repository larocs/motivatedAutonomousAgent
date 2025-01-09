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
 * @file iCubeProcessorThread.cpp
 * @brief Implementation of the eventDriven thread (see iCubeProcessorThread.h).
 */

//Original code is from Sara M.

#include <iCub/iCubeProcessorThread.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace boost;

#define THPERIOD 0.5//s

iCubeProcessorThread::iCubeProcessorThread():PeriodicThread(THPERIOD) {
    robot = "icub";   
}

iCubeProcessorThread::iCubeProcessorThread(string _robot, ResourceFinder &_rf, int _numberOfICubes):PeriodicThread(THPERIOD){
    robot = _robot;
    rf = _rf;
    numberOfICubes = _numberOfICubes;
}

iCubeProcessorThread::~iCubeProcessorThread() {

}

void iCubeProcessorThread::setName(string str) {
    this->name=str;
}


std::string iCubeProcessorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void iCubeProcessorThread::setInputPortName(string InpPort) {
    
}

bool iCubeProcessorThread::openAllPorts(){
    yDebug("Opening and Waiting for All Ports");

    /* ======================== Input ========================*/
    for (int i = 0; i < numberOfICubes; i++){
        BufferedPort<Bottle>* dataPort;
        BufferedPort<Bottle>* eventPort;
        dataPort = new BufferedPort<Bottle>;
        eventPort = new BufferedPort<Bottle>;

        string name_temp = dataPortNamePrefix + to_string(i) + ":i";
        if(!dataPort->open(getName(name_temp.c_str()).c_str())) {
            yError("unable to open port to read data from the iCube %d",i);
            return false;  // unable to open; let RFModule know so that it won't run
        }

        name_temp = eventPortNamePrefix + to_string(i) + ":i";
        if(!eventPort->open(getName(name_temp.c_str()).c_str())) {
            yError("unable to open port to read event from the iCube %d",i);
            return false;  // unable to open; let RFModule know so that it won't run
        }

        inputICubesDataPorts.push_back(dataPort);
        inputICubesEventsPorts.push_back(eventPort);
    }

    /* ======================== Output ========================*/   
    yDebug("OPENING for Perception connection");
    if(!outputICubeDataPort.open(getName("/iCubeData:o").c_str())) {
        yError("unable to open port to send iCube processed stimuli ");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    yDebug("Everything opened!");

    return true;
}

bool iCubeProcessorThread::threadInit() {
    filepath = rf.find("filepath").asString();
    filename = filepath + filename;

    saveHeaders();
   
    if(!openAllPorts())
        return false;

    yInfo("Initialization of the processing thread correctly ended");

    timeInitial = Time::now();

    return true;
}

void iCubeProcessorThread::run(){
    /************************************ Time related -> just for saving stuff ************************************/
    now = time(0);
    char* timeNow_ = ctime(&now);
    timeNow = timeNow_;
    erase_all(timeNow, "\n");
    /************************************/

    readICube();
    
    printDataAlliCubes();
    
    sendToPerception();

    save();

    dataAllCubes.clear();
    
    cout <<"-------------------------------------------------------"<<endl;
}

void iCubeProcessorThread::processICubeData(Bottle* bottleCubeData){
    int touchedFaces = 0;

    if(bottleCubeData->size()){
        quaternions = {bottleCubeData->get(0).asFloat64(), bottleCubeData->get(1).asFloat64(),bottleCubeData->get(2).asFloat64(), bottleCubeData->get(3).asFloat64()};
        touchesFace_raw = {bottleCubeData->get(4).asString(), bottleCubeData->get(5).asString(), bottleCubeData->get(6).asString(), bottleCubeData->get(7).asString(), bottleCubeData->get(8).asString(), bottleCubeData->get(9).asString()};
        accelerations = {bottleCubeData->get(10).asFloat64(),bottleCubeData->get(11).asFloat64(), bottleCubeData->get(12).asFloat64()};
    
        for(int i = 0; i < touchesFace_raw.size(); i++)
            if(touchesFace_raw[i].find("1") != std::string::npos){
                touchedFaces += 1;
                facesBeingTouched.addInt32(1);
            }else
                facesBeingTouched.addInt32(0);

        dataCube.addInt32(touchedFaces);
        dataCube.addList() = facesBeingTouched;
    }
}

void iCubeProcessorThread::processICubeEvent(Bottle* bottleCubeEvents){
    if(bottleCubeEvents->size()){
        pose = bottleCubeEvents->get(0).asString();
        dataCube.addString(pose);
    }
}

void iCubeProcessorThread::resetStructures(){
    pose.clear();
    touchesFace_raw.clear();
    quaternions.clear();
    accelerations.clear();
    facesBeingTouched.clear();
    dataCube.clear();
}

void iCubeProcessorThread::readICube(){
     for(int i = 0; i < numberOfICubes; i++){
        resetStructures();
        cout<<i<<endl;
        //Data related
        if(inputICubesDataPorts[i]->getInputCount()){
            Bottle* bottleCubeData = NULL;
            bottleCubeData = inputICubesDataPorts[i]->read(false);//used false because if one cube lose connection the module remains blocked until the cube connects again

            if(bottleCubeData != NULL)
                processICubeData(bottleCubeData);
            else{
                cout<<"iCube_"<<i<<" DID NOT send data"<<endl;
                dataCube.addInt32(NO_DATA);//Means that didnt receive data from cube in that reading (it is different from receiving data with 0 faces touched)
                dataCube.addList() = facesBeingTouched;
            }
        }else{
            cout<<"iCube_"<<i<<" not connected"<<endl;
            dataCube.addInt32(NO_CONNECTION);//Means that iCube_x is not connected
            dataCube.addList() = facesBeingTouched;
        }

        //Events related
        if(inputICubesEventsPorts[i]->getInputCount()){
            Bottle* bottleCubeEvents = NULL;
            bottleCubeEvents = inputICubesEventsPorts[i]->read(false);//send data just when there is a new movement (sometimes lose the reading due time issues)

            if(bottleCubeEvents != NULL)
                processICubeEvent(bottleCubeEvents);
        }

        dataAllCubes.addList() = dataCube;
        dataCube.clear();
    }
}

void iCubeProcessorThread::printDataAlliCubes(){
    cout<<"print"<<endl;
    if(dataAllCubes.size() > 0){
        for(int i = 0; i < dataAllCubes.size(); i++){
            cout<<"iCube_" <<i<<":  ";
            cout<<dataAllCubes.get(i).asList()->get(0).asInt32()<<",   [ ";
            if(dataAllCubes.get(i).asList()->get(1).asList() != NULL)
                cout<<dataAllCubes.get(i).asList()->get(1).asList()->get(0).asInt32()<<"  "<<dataAllCubes.get(i).asList()->get(1).asList()->get(1).asInt32()<<"  "<<dataAllCubes.get(i).asList()->get(1).asList()->get(2).asInt32()<<"  "<<dataAllCubes.get(i).asList()->get(1).asList()->get(3).asInt32()<<"  "<<dataAllCubes.get(i).asList()->get(1).asList()->get(4).asInt32()<<"  "<<dataAllCubes.get(i).asList()->get(1).asList()->get(5).asInt32();
            cout<<"],   "<<dataAllCubes.get(i).asList()->get(2).asString()<<endl;
        }
    }
}

void iCubeProcessorThread::sendToPerception(){
    if(outputICubeDataPort.getOutputCount()){
        outputICubeDataPort.prepare() = dataAllCubes;
        outputICubeDataPort.write();
    }
}

void iCubeProcessorThread::saveHeaders(){
    //Save header in the file
    ofstream fout; 
    
    fout.open(filename, ios::app);
    fout << fileHeader << "\n";
    fout.close( );
}

void iCubeProcessorThread::save(){
    ofstream fout;
    fout.open(filename, ios::app);
    std::time_t temp = Time::now();
    
    if(dataAllCubes.size() > 0){
        for(int i = 0; i < dataAllCubes.size(); i++){
            fout << timeNow << ',' << to_string(temp - timeInitial) << ',' << i << ',' << dataAllCubes.get(i).asList()->get(0).asInt32()<<",";
            if(dataAllCubes.get(i).asList()->get(1).asList() != NULL)
                fout<<dataAllCubes.get(i).asList()->get(1).asList()->get(0).asInt32()<<","<<dataAllCubes.get(i).asList()->get(1).asList()->get(1).asInt32()<<","<<dataAllCubes.get(i).asList()->get(1).asList()->get(2).asInt32()<<","<<dataAllCubes.get(i).asList()->get(1).asList()->get(3).asInt32()<<","<<dataAllCubes.get(i).asList()->get(1).asList()->get(4).asInt32()<<","<<dataAllCubes.get(i).asList()->get(1).asList()->get(5).asInt32();
            fout<<dataAllCubes.get(i).asList()->get(2).asString()<<'\n';
        }
    }

    fout.close();
}

bool iCubeProcessorThread::processing(){
    // here goes the processing...
    return true;
}

void iCubeProcessorThread::threadRelease(){
    for(int i = 0; i < numberOfICubes; i++){
        inputICubesDataPorts[i]->interrupt();
        inputICubesDataPorts[i]->close();
        delete inputICubesDataPorts[i];

        inputICubesEventsPorts[i]->interrupt();
        inputICubesEventsPorts[i]->close();
        delete inputICubesEventsPorts[i];
    }
 
    outputICubeDataPort.interrupt();
    outputICubeDataPort.close();
}