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
 * @file iCubSimInteractionThread.cpp
 * @brief Implementation of the eventDriven thread (see iCubSimInteractionThread.h).
 */

#include <iCub/iCubSimInteractionThread.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace boost;

#define THPERIOD 0.5//s

iCubSimInteractionThread::iCubSimInteractionThread():PeriodicThread(THPERIOD) {
    robot = "icub";        
}

/* iCubSimInteractionThread::iCubSimInteractionThread(string _robot, string _configFile):PeriodicThread(THPERIOD){
    robot = _robot;
    configFile = _configFile;
} */

iCubSimInteractionThread::iCubSimInteractionThread(string _robot,ResourceFinder &_rf):PeriodicThread(THPERIOD){
    robot = _robot;
    rf = _rf;
}

iCubSimInteractionThread::~iCubSimInteractionThread() {

}

void iCubSimInteractionThread::setName(string str) {
    this->name=str;
}


std::string iCubSimInteractionThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void iCubSimInteractionThread::setInputPortName(string InpPort) {
    
}

bool iCubSimInteractionThread::openAllPorts(){
    if(!inputPortResetWorld.open(getName("/resetWorld:i").c_str())){
        yError("unable to open port to send unmasked events ");
        return false;
    }
    
    if(robot.compare("icubSim") == 0){
        //create RPC client and connect it to the world port
        rpcWorld.open("/iCubSimInteraction/world");
        Network::connect("/iCubSimInteraction/world", "/icubSim/world");
    }

    //create driver for the head
    options.put("device", "remote_controlboard");
    options.put("local", "/iCubSimInteraction/head");
    options.put("remote", "/" + robot + "/head");

    robotHead = new PolyDriver();
    robotHead->open(options);

    if(!robotHead->isValid()){
        printf("Cannot connect to robot head\n");
        return false;
    }

    return true;
}

bool iCubSimInteractionThread::threadInit() {
    episode = 0;

    objectsFilename = rf.find("objectsFilename").asString() + objectsFilename;
    cout<<objectsFilename<<endl;
    filepath = rf.find("filepath").asString();
    filename = filepath + filename;

    saveHeaders();

    if(!openAllPorts())
        return false;

    //Just add objects in the world if using the simulator
    if(robot.compare("icubSim") == 0){
        createObjects();
        deletAllObjectsWorld();
        addObjectsWorld();
    }
    
    yInfo("Initialization of the processing thread correctly ended");

    return true;
}

void iCubSimInteractionThread::run(){
    //reset simulator when the episode finishes in RL. 
    if(inputPortResetWorld.getInputCount()){
        Bottle* readR = inputPortResetWorld.read(false);
        if(readR != nullptr){
            if(robot.compare("icubSim") == 0){//Just add objects in the world if using the simulator
                deletAllObjectsWorld();
                addObjectsWorld();
                episode++;
            }
            //turnRobotHead();
        }    
    }
    cout<<"----------------"<<endl;
}

void iCubSimInteractionThread::saveObjWorld(int index){
    ofstream fout;
    fout.open(filename, ios::app);
    
    fout << episode << ',' << objectsWorld[index].typeObject << ',' << objectsWorld[index].size1 << ',' << objectsWorld[index].size2 << ',' << objectsWorld[index].size3
    << ',' << objectsWorld[index].pos1 << ',' << objectsWorld[index].pos2 << ',' << objectsWorld[index].pos3
    << ',' << objectsWorld[index].color1 << ',' << objectsWorld[index].color2 << ',' << objectsWorld[index].color3;

    fout <<"\n";
    fout.close();
}

void iCubSimInteractionThread::saveHeaders(){
    //Save header in the file
    ofstream fout; 
    
    fout.open(filename, ios::app);
    fout << fileHeader << "\n";
    fout.close( );
}

void iCubSimInteractionThread::createObjects(){
    ifstream fin;
    fin.open(objectsFilename); 
    string myline;
    
    std::string newObjectInfos[11];
    InfoObjectsWorld newObject;

    size_t pos;
    std::string token;
    std::string delimiter = ",";
    int i;

    if(fin.is_open()){
        while(fin){    
            getline(fin, myline);
            cout<<myline<<endl;
            i = 0;
            pos = 0;
            while ((pos = myline.find(delimiter)) != std::string::npos){
                token = myline.substr(0, pos);
                newObjectInfos[i] = token;
                std::cout << token << std::endl;
                myline.erase(0, pos + delimiter.length());
                i++;
            }
            std::cout << myline << std::endl;
            newObjectInfos[i] = myline;//get the last parameter in the line

            if(i > 0){//just get as objects the lines that contain data in the file
                newObject.typeObject = newObjectInfos[0];
                newObject.size1 = stof(newObjectInfos[1]);
                newObject.size2 = stof(newObjectInfos[2]);
                newObject.size3 = stof(newObjectInfos[3]);
                newObject.pos1 = stof(newObjectInfos[4]);
                newObject.pos2 = stof(newObjectInfos[5]);
                newObject.pos3 = stof(newObjectInfos[6]);
                newObject.color1 = stoi(newObjectInfos[7]);
                newObject.color2 = stoi(newObjectInfos[8]);
                newObject.color3 = stoi(newObjectInfos[9]);

                objectsWorld.push_back(newObject);
            }
        }
        cout<<"#objs: "<<objectsWorld.size()<<endl;
    }else
        cout<<"Couldn't open the "<<objectsFilename<<" file"<<endl;

    totalOjects = objectsWorld.size();
}

void iCubSimInteractionThread::addObjectsWorld(){
    Bottle response;
    //prepare command for RPC
    Bottle addObjInWorld;

    //Desconsider the first object (table)
    std::uniform_int_distribution<int> distrObjcs(1, totalOjects-1);
    int objsInScene = distrObjcs(rdx);

    for(int i = 0; i < objsInScene + 1; i++){//+1 because add the table + objects above the table (randomly amount)
        addObjInWorld.clear();
        addObjInWorld.addString("world");
        addObjInWorld.addString("mk");
        addObjInWorld.addString(objectsWorld[i].typeObject);//box, sbox, sph, ssph, cyl, scyl (s-xxx is for static objects)
        addObjInWorld.addFloat64(objectsWorld[i].size1);
        addObjInWorld.addFloat64(objectsWorld[i].size2);
        addObjInWorld.addFloat64(objectsWorld[i].size3);
        addObjInWorld.addFloat64(objectsWorld[i].pos1);
        addObjInWorld.addFloat64(objectsWorld[i].pos2);
        addObjInWorld.addFloat64(objectsWorld[i].pos3);
        addObjInWorld.addInt32(objectsWorld[i].color1);
        addObjInWorld.addInt32(objectsWorld[i].color2);
        addObjInWorld.addInt32(objectsWorld[i].color3);
        cout<<addObjInWorld.toString()<<endl;
        //make RPC call
        response.clear();
        rpcWorld.write(addObjInWorld, response);
        cout<<"Adding object: "<<response.toString()<<endl;

        saveObjWorld(i);
    }
}

void iCubSimInteractionThread::deletAllObjectsWorld(){
    //prepare command for RPC
    Bottle worldDelAll;
    worldDelAll.addString("world");
    worldDelAll.addString("del");
    worldDelAll.addString("all");

    //make RPC call
    Bottle response;
    rpcWorld.write(worldDelAll, response);
    cout<<"Deleting all objects: "<<response.toString()<<endl;
}

void iCubSimInteractionThread::turnRobotHead(){
    //get number of joints
    IPositionControl *pos;
    robotHead->view(pos);
    int jnts = 0 ;
    pos->getAxes(&jnts); //number of joints

    //prepare new positions for all joints
    Vector positions;
    positions.resize(jnts);

    for(int j = 0; j < jnts; j++)
        positions[j] = 0;

    std::uniform_real_distribution<float> distrNeck(-30, 22); // Neck pitch joint range limit
    std::uniform_real_distribution<float> distrYaw(-45, 45); //Neck yaw joint range limit

    positions[0] = distrNeck(rdx);//Neck pitch (vertical)
    positions[2] = distrYaw(rdx);//Neck yaw (horizontal)

    //move to desired positions
    pos->positionMove(positions.data());
    bool done = false;

    while(!done)
        pos->checkMotionDone(&done);
}

bool iCubSimInteractionThread::processing(){
    // here goes the processing...
    return true;
}

void iCubSimInteractionThread::threadRelease() {
    inputPortResetWorld.interrupt();
    inputPortResetWorld.close();
}