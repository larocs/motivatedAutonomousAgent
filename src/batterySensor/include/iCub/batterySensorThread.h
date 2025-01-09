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
 * @file batterySensorThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _BATTERYSENSOR_PERIODTHREAD_H_
#define _BATTERYSENSOR_PERIODTHREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <random>

//Used to initialize the variables
#define LEARNING_PHASE      "learn"
#define TESTING_PHASE       "evaluate"
#define FINETUNING_PHASE    "finetuning"
#define RULE_BASED          "rulebased"

class batterySensorThread : public yarp::os::PeriodicThread {
private:

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    
    std::string name;  

    yarp::os::ResourceFinder rf;

    std::string filepath;
    std::string filename = "battery.csv";
    std::string fileHeader = "time,durationExp,batteryLevel";

    time_t now;
    std::string timeNow;
    std::time_t timeInitial;

    double batteryLevel;
    double decreaseRate;
    const int rechargeTime = 10; 
    int rechargeTime_remaining;

    //-------------------------Values are defined in the configuration file
    double VALUE_TO_RECHARGE;
    double MIN_BATTERY_LEVEL;
    double MAX_BATTERY_LEVEL;
    double SURVIVAL_HOMEOSTASIS;
    double PERCEN_HOMEOSTASIS;
    int RANGE_SURVIVE;
    
    //each behavior impacts the battery consumption in a different way
    double INIT_END_CONS;
    double IDLE_CONS;
    double PLAY_CONS;//2;
    double RECHARGE_CONS;
    double INTERACT_CONS;//2;
    double LOOKDOWN_CONS;

    std::string mode;          //LEARNING_PHASE, TESTING_PHASE, FINETUNING_PHASE

    //-------------------------End of the Values defined in the configuration file

    std::mt19937 rdx{static_cast<long unsigned int>(21)};
    
    yarp::os::BufferedPort<yarp::os::Bottle> inputPortRecharge;             //if action is "recharge" receives info here to update the battery level
    yarp::os::BufferedPort<yarp::os::Bottle> inputUpdBatteryConsPort;       //battery consumption is according to the behavior to be executed
    yarp::os::BufferedPort<yarp::os::Bottle> inputPortResetBattery;        //reset battery level when the episode finishes in RL

    yarp::os::BufferedPort<yarp::os::Bottle> outputPortBatteryLevel;
    yarp::os::BufferedPort<yarp::os::Bottle> outputPortNoBattery;

public:
    /**
    * constructor default
    */
    batterySensorThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    //batterySensorThread(std::string robotname,std::string configFile);
    batterySensorThread(std::string robotname, yarp::os::ResourceFinder &_rf);
    /**
     * destructor
     */
    ~batterySensorThread();

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
    double recharge();
    bool noBattery();
    void updateDecreaseRate(std::string behavior);
    void resetBattery();
    void initVarsFromFile();
    void printData();

    void saveData();
    void saveHeaders();
};

#endif  //_BATTERYSENSOR_PERIODTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------