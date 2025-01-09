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
 * @file iCubeProcessorThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _ICUBEPROCESSOR_PERIODTHREAD_H_
#define _ICUBEPROCESSOR_PERIODTHREAD_H_

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

#define NO_DATA -1
#define NO_CONNECTION -2

class iCubeProcessorThread : public yarp::os::PeriodicThread {
private:

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    std::string name;

    yarp::os::ResourceFinder rf;

    std::string filepath;
    std::string filename = "iCubeProcessor.csv";
    std::string fileHeader = "time,durationExp,iCubeNumber,numberFacesTouched,face_0,face_1,face_2,face_3,face_4,face_5,pose";

    time_t now;
    std::string timeNow;
    std::time_t timeInitial;

    std::vector<std::string> touchesFace_raw;
    yarp::os::Bottle facesBeingTouched;
    std::vector<double> quaternions,accelerations;
    std::string pose;

    yarp::os::Bottle dataCube;
    yarp::os::Bottle dataAllCubes;

    //First steps to try to have more than one cube
    int numberOfICubes;
    std::string dataPortNamePrefix = "/iCubeData_";
    std::string eventPortNamePrefix = "/iCubeEvents_";
    std::vector<yarp::os::BufferedPort<yarp::os::Bottle>*> inputICubesDataPorts;
    std::vector<yarp::os::BufferedPort<yarp::os::Bottle>*> inputICubesEventsPorts;

    
   
    yarp::os::BufferedPort<yarp::os::Bottle> outputICubeDataPort;
    
public:
    /**
    * constructor default
    */
    iCubeProcessorThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    //iCubeProcessorThread(std::string robotname,std::string configFile);
    iCubeProcessorThread(std::string _robot,yarp::os::ResourceFinder &_rf, int _numberOfICubes);
    /**
     * destructor
     */
    ~iCubeProcessorThread();

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

    void saveHeaders();
    void save();

    void readICube();
    void printDataAlliCubes();
    void resetStructures();
    void sendToPerception();
    void processICubeData(yarp::os::Bottle* bottleCubeData);
    void processICubeEvent(yarp::os::Bottle* bottleCubeEvents);
};

#endif  //_ICUBEPROCESSOR_PERIODTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------