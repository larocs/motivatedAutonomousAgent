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
 * @file motivationThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _MOTIVATION_PERIODTHREAD_H_
#define _MOTIVATION_PERIODTHREAD_H_

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
#include <random>
#include <algorithm>
#include <string.h>

//Used when the robot's eyes are closed
#define NOFACE -1.0 //Must tbe same value in perception

//Used to initialize the variables
#define LEARNING_PHASE      "learn"
#define TESTING_PHASE       "evaluate"
#define FINETUNING_PHASE    "finetuning"
#define RULE_BASED          "rulebased"
#define DRIVE_BASED        "drivebased"

#define SOCIAL_PROFILE  "social"
#define PLAYFUL_PROFILE "playful"
#define REGULAR_PROFILE "regular"

class motivationThread : public yarp::os::PeriodicThread {
private:

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    
    std::string name;               // rootname of all the ports opened by this thread

    yarp::os::ResourceFinder rf;

    std::string robot_color;        //Used to not consider the robot's arm as an object to interact
    std::string robot_profile;

    //Save data
    std::string filepath;
    std::string filenameAllData = "motivation.csv";
    std::string fileHeaderAllData = "time,durationExp,batteryLevel,qntdObjsSeeing,touch_current,face_current,gaze_current,comfort_current,boredom,explore_homeostasis,range_explore,affect_homeostasis,range_affect,survive_homeostasis,range_survive,exploreDrive,affectDrive,surviveDrive,indexObjChoosen";

    std::string filenameAllObjectsMemory = "motivation_AllObjectsMemory.csv";
    std::string fileHeaderAllObjectsMemory = "time,durationExp,objectColor,value,seeing";

    std::mt19937 rdx{static_cast<long unsigned int>(21)};

    typedef struct objects_{
        std::string color;
        double value;
        bool seeing;
    }objects;

    std::vector<objects> allObjectsMemory;

    //Boredom
    const float alpha = 1.0;//0.5;//increase boredom rate for objects
    const float maxValue = 10.0; //Max value that a object can have (having the max value means that is the object that I'm interacting/choosing now). Smaller the value. more interesting
    int indexObjChoosen;
    double mostInterestingReward;
    double mostInterestingRewardRandObj = maxValue;//Define a correct value
    double boredom;
    const int playingTime = 50; 
    int playingTime_remaining;
    double playingValue;

    //Drives
    double exploreDrive;
    double surviveDrive;
    double affectDrive;
    bool computeDrives = true;

    //defined as class variable just to save data easier;
    double batteryLevel;
    int numberOfObjectsScene;

    yarp::os::Bottle* dataAllCubes;

    time_t now;
    std::string timeNow;
    std::time_t timeInitial;

    //Variables for Comfort computing
    float touch_current, face_current, gaze_current;
    float comfort_current, comfort_prev;
    const double beta_comfort_Ambivalent = 0.96;//0.92
    const double beta_comfort_Ambivalent_Sleeping = 0.98;
    const double tau_comfort_Ambivalent = 0.5;//50;
    int saturActingTime;

    //-------------------------Values are defined in the configuration file
    //1 if using the drive, 0 otherwise
    int DRIVE_BOREDOM;
    int DRIVE_AFFECT;
    int DRIVE_SURVIVE;

    double MAX_BOREDOM;
    double MIN_BOREDOM;
    double INCREASE_BOREDOM;

    double MIN_COMFORT;
    double MAX_COMFORT;

    double MIN_BATTERY;
    double MAX_BATTERY;

    double SURVIVAL_HOMEOSTASIS;
    double AFFECT_HOMEOSTASIS;
    double EXPLORE_HOMEOSTASIS;

    //Range to define the lowerbound (H - range) and the upperbound (H + range) around the homeostasis threshold
    int RANGE_EXPLORE;
    int RANGE_AFFECT;
    int RANGE_SURVIVE;

    std::string mode;          //LEARNING_PHASE, TESTING_PHASE, FINETUNING_PHASE, RULE_BASED

    //-------------------------End of the Values defined in the configuration file

    yarp::os::BufferedPort<yarp::os::Bottle> inputBatteryLevelPort;             //read battery level from perception
    yarp::os::BufferedPort<yarp::os::Bottle> inputAllObjects;                   //read all objects in the robot's FOV
    yarp::os::BufferedPort<yarp::os::Bottle> inputInteractionPort;              //read face_current, gaze_current, touch_current

    yarp::os::BufferedPort<yarp::os::Bottle> inputUpdateBoredomPort;          //read if the decision was play, then update boredom (internal drive)
    yarp::os::BufferedPort<yarp::os::Bottle> inputPortResetBoredomComfort;    //reset boredom and comfort level when the episode finishes in RL

    yarp::os::BufferedPort<yarp::os::Bottle> inputPortEndInitialBehavior;

    yarp::os::BufferedPort<yarp::os::Bottle> outputExploreDriveAndObject;     //write the explore drive + most interesting object to interact
    yarp::os::BufferedPort<yarp::os::Bottle> outputSurvivalDrivePort;           //write the survival drive (battery)
    yarp::os::BufferedPort<yarp::os::Bottle> outputAffectDrivePort;             //write the affect drive according to the comfort value

    //Ports used to debug the behavior of the needs (Boredom, Comfort)
    yarp::os::BufferedPort<yarp::os::Bottle> outputComfortPort;
    yarp::os::BufferedPort<yarp::os::Bottle> outputBoredomPort;

    yarp::os::BufferedPort<yarp::os::Bottle> inputPortSaturetedAffect;

    yarp::os::BufferedPort<yarp::os::Bottle> inputiCubesPort;

public:
    /**
    * constructor default
    */
    motivationThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    motivationThread(std::string robotname,std::string configFile);
  
    /**
    * constructor 
    * @param robot_color color of the robot
    */
    //motivationThread(std::string robotname,std::string configFile, std::string robot_color);
    motivationThread(std::string _robot,yarp::os::ResourceFinder &_rf, std::string robot_color, std::string robot_profile);
   
    /**
     * destructor
     */
    ~motivationThread();

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
    void writeAllOutputPorts();
    
    void initAllVars();

    void saveData();
    void saveHeaders();
    void saveObjectsMemory();

    void updateObjectMemory();
    
    void computeInterestInObjects();

    void iCubesProcessing();

    void computeAffect();
    void computeEnergy();
    void computeBoredom();
    void startDrivesComputation();

    float comfortProcessing();

    float computeDrive(float need, int homeostasis_base, int range);

    void resetVars();

    void initVarsFromFile();
    void printData();

};

#endif  //_MOTIVATION_PERIODTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------