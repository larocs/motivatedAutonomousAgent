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
 * @file actionThread.h
 * @brief Definition of a thread that look at and point to a specific object detected in the scene.
 */


#ifndef _ACTION_PERIODTHREAD_H_
#define _ACTION_PERIODTHREAD_H_

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
#define ICUBSIM_ROBOT   "icubSim"

class actionThread : public yarp::os::PeriodicThread {
private:

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    
    std::string robotPlatform;      // name of the real robot being used

    std::string name;  

    yarp::os::ResourceFinder rf;
    
    yarp::dev::PolyDriver *clientGazeCtrl;
    yarp::dev::IGazeControl *iGaze;
    yarp::os::Property optGaze;

    yarp::os::Bottle action;
    yarp::os::Bottle* inputObjectToActAt;
    std::string colorToPlay;

    std::string filepath;
    std::string filenameARE = "action_ARE.csv";
    std::string fileHeaderARE = "time,durationExp,color,middleX_leftCam,middleY_leftCam,middleX_rightCam,middleY_rightCam";

    std::string filenameAllData = "action_allData.csv";
    std::string fileHeaderAllData = "time,durationExp,actionType";

    int middleX_leftCam, middleY_leftCam, middleX_rightCam, middleY_rightCam;
    double z = 1.0;   // distance [m] of the object from the image plane (extended to infinity): yes, you probably need to guess, but it works pretty robustly

    time_t now;
    std::string timeNow;
    std::time_t timeInitial;
    int turnFlag;
    std::time_t timePastIdle;

    yarp::os::BufferedPort<yarp::os::Bottle> inputCommandSMPort;
   
    yarp::os::Port outputPortObjectToActAt;
    yarp::os::BufferedPort<yarp::os::Bottle> outputPortRecharge;

    yarp::os::BufferedPort<yarp::os::Bottle> outputSpeechPort;

    yarp::os::Port outputEmotionsPort;

    yarp::os::RpcClient outputMovementRAPort;
    yarp::os::RpcClient outputMovementLAPort; 
    yarp::os::RpcClient outputMovementTPort;


    yarp::os::RpcClient outRpcEyelids_berry;  // RPC port to control the eyelids -- Berry
    yarp::os::RpcClient outRpcEyelids_reddy;  // RPC port to control the eyelids -- Reddy
    yarp::os::BufferedPort<yarp::os::Bottle> outputPortEyelids_icub;  // RPC port to control the eyelids -- iCubSim

public:
    /**
    * constructor default
    */
    actionThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    actionThread(std::string robotname,yarp::os::ResourceFinder &_rf);

    actionThread(std::string robotname, yarp::os::ResourceFinder &_rf, std::string robotPlatform);

    /**
     * destructor
     */
    ~actionThread();

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
    void initAllVars();

    void saveData(std::string actionType);
    void saveDataARE(int pXL, int pYL, int pXR, int pYR);
    void saveHeaders();

    void readCommandSM();

    void actARE(std::string actionARE, int pXL, int pYL, int pXR, int pYR);
    void computeTargetCentroid(int topXLeft, int topYLeft, int bottomXLeft, int bottomYLeft, int topXRight, int topYRight, int bottomXRight, int bottomYRight);
    void writeToARE();
    void homeARE();

    void moveHeadDirection(std::string direction);
    void executeMovement(const double time, const double offset, const std::string bodyPart, const std::string movement);
    void setFaceModular(std::string affectEyebr, std::string affectMouth);
    void speakText(std::string speech);

    void turnHeadAngle(float angle);
    void turnHeadTwoAngles(float angleX, float angleY);
    
    void lookAround();
    void lookAtPointMono(int pointX, int pointY, int camera);
    void lookAtPointStereo(int pointX_Left, int pointY_Left, int pointX_Right, int pointY_Right);

    void changeIGazeSpeed();

    void moveEyelids(const double time, const double offset, std::string movement);
    yarp::os::Bottle getEyelidsPose(std::string movement);
    yarp::os::Bottle updateStatusEyelids_ToPerception(std::string movement);
};

#endif  //_ACTION_PERIODTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------