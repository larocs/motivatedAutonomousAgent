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
 * @file decisionMakingModule.cpp
 * @brief Implementation of the decisionMakingModule (see header file).
 */

#include "iCub/decisionMakingModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool decisionMakingModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name",
                           Value("/decisionMaking"), 
                           "module name (string)").asString();
    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";

    /*inputPortName           = rf.check("inputPortName",
			                Value(":i"),
                            "Input port name (string)").asString();*/
    
    
    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port

    if (rf.check("config")) {
        configFile=rf.findFile(rf.find("config").asString().c_str());
        if (configFile=="") {
            return false;
        }
    }
    else {
        configFile.clear();
    }

        //The default color is red (color of the iCub used in the CONTACT (COgNiTive Architecture for Collaborative Technologies) Unit)
    robot_color            = rf.check("robotColor", 
                           Value("red"), 
                           "Robot color (string)").asString();

    //The dafault is the social profile (prefers interact with the person). The Value() must be the same defined in the #define on motivation
    robot_profile           = rf.check("robotProfile", 
                           Value("social"), 
                           "Robot profile (string)").asString();


    /* create the thread and pass pointers to the module parameters */
    //pThread = new decisionMakingThread(robotName, configFile);
    //pThread = new decisionMakingThread(robotName, configFile, robot_color);
    pThread = new decisionMakingThread(robotName, rf, robot_color, robot_profile);
    pThread->setName(getName().c_str());
    //pThread->setInputPortName(inputPortName.c_str());
    
    /* now start the thread to do the work */
    pThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool decisionMakingModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool decisionMakingModule::close() {
    handlerPort.close();
    /* stop the thread */
    yDebug("stopping the thread \n");
    pThread->stop();
    return true;
}

bool decisionMakingModule::respond(const Bottle& command, Bottle& reply) 
{
    string helpMessage =  string(getName().c_str()) + 
                " commands are: \n" +  
                "help \n" +
                "quit \n";
    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        //pThread->suspend();
        pThread->stop();
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    
    return true;
}

/* Called periodically every getPeriod() seconds */
bool decisionMakingModule::updateModule()
{
    return true;
}

double decisionMakingModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1;
}