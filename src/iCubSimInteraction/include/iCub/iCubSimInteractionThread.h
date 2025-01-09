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
 * @file iCubSimInteractionThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _ICUBSIMINTERACTION_PERIODTHREAD_H_
#define _ICUBSIMINTERACTION_PERIODTHREAD_H_

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
#include <vector>
#include <random>
#include <string.h>

class iCubSimInteractionThread : public yarp::os::PeriodicThread {
private:

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    std::string name;

    yarp::os::ResourceFinder rf;

    std::string objectsFilename = "objectsSettings.csv";

    std::string filepath;
    std::string filename = "objectsWorld.csv";
    std::string fileHeader = "episode,typeObject,size1,size2,size3,pos1,pos2,pos3,color1,color2,color3";

    int episode;

    typedef struct InfoObjectsWorld_{
        std::string typeObject;
        float size1;
        float size2;
        float size3;
        float pos1;
        float pos2;
        float pos3;
        int color1;
        int color2;
        int color3;
    }InfoObjectsWorld;

    std::vector<InfoObjectsWorld> objectsWorld;
    int totalOjects;

    std::mt19937 rdx{static_cast<long unsigned int>(21)};

    yarp::os::BufferedPort<yarp::os::Bottle> inputPortResetWorld;        //reset simulator when the episode finishes in RL
  
    //Connect to iCub_SIM
    yarp::os::RpcClient rpcWorld;

    //Turn head angle
    yarp::os::Property options;
    yarp::dev::PolyDriver *robotHead;
    
public:
    /**
    * constructor default
    */
    iCubSimInteractionThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    //iCubSimInteractionThread(std::string robotname,std::string configFile);
    iCubSimInteractionThread(std::string _robot,yarp::os::ResourceFinder &_rf);
    /**
     * destructor
     */
    ~iCubSimInteractionThread();

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

    void createObjects();
    void deletAllObjectsWorld();
    void addObjectsWorld();
    void turnRobotHead();

    void saveObjWorld(int index);
    void saveHeaders();
    
};

#endif  //_ICUBSIMINTERACTION_PERIODTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------