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
 * @file sleepingThread.h
 * @brief Definition of a thread that look at and point to a specific object detected in the scene.
 */


#ifndef _SLEEPING_PERIODTHREAD_H_
#define _SLEEPING_PERIODTHREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cstring>
#include <boost/algorithm/string.hpp>
#include <string.h>

#define REDDY_ROBOT     "reddy"
#define BERRY_ROBOT     "berry"

class sleepingThread : public yarp::os::PeriodicThread {
private:

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    
    std::string robotPlatform;      // name of the real robot being used

    std::string name;  

    int init;

    yarp::os::ResourceFinder rf;
    
    yarp::os::BufferedPort<yarp::os::Bottle> outputSpeechPort;

    yarp::os::Port outputEmotionsPort;

    yarp::os::RpcClient outputMovementRAPort;
    yarp::os::RpcClient outputMovementLAPort; 
    yarp::os::RpcClient outputMovementTPort;


    yarp::os::RpcClient outRpcEyelids_berry;  // RPC port to control the eyelids -- Berry
    yarp::os::RpcClient outRpcEyelids_reddy;  // RPC port to control the eyelids -- Reddy

public:
    /**
    * constructor default
    */
    sleepingThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    sleepingThread(std::string robotname,yarp::os::ResourceFinder &_rf);

    sleepingThread(std::string robotname, yarp::os::ResourceFinder &_rf, std::string robotPlatform);

    /**
     * destructor
     */
    ~sleepingThread();

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
    
    void executeMovement(const double time, const double offset, const std::string bodyPart, const std::string movement);
    void setFaceModular(std::string affectEyebr, std::string affectMouth);
    void speakText(std::string speech);
    

    void moveEyelids(const double time, const double offset, std::string movement);
    yarp::os::Bottle getEyelidsPose(std::string movement);
};

#endif  //_SLEEPING_PERIODTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------