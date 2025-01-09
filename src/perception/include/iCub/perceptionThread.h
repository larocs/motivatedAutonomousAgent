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
 * @file perceptionThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _PERCEPTION_PERIODTHREAD_H_
#define _PERCEPTION_PERIODTHREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <cstring>
#include <list>
#include <math.h>
#include <vector>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <string.h>

class perceptionThread : public yarp::os::PeriodicThread {
private:

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    yarp::os::ResourceFinder rf;
    
    //int time;
    time_t now;
    std::string timeNow;
    std::time_t timeInitial;

    //Save data
    std::string filepath;
    std::string filenameAllData = "perception_AllData.csv";
    std::string fileHeaderAllData = "time,durationExp,batteryLevel,touch_current,originOfTouch,sideOfTouch,faceSuccessInput,faceCertInput,affectInput,faceXInput,faceYInput,faceDepthInput,face_current,gaze_current";

    std::string filenameAllObjects = "perception_AllObjects.csv";
    std::string fileHeaderAllObjects = "time,durationExp,objectColor,topLeftX_leftCam,topLeftY_leftCam,bottomRightX_leftCam,bottomRightY_leftCam,topLeftX_rightCam,topLeftY_rightCam,bottomRightX_rightCam,bottomRightY_rightCam";

    std::string filenameAllICubes = "perception_iCubes.csv";
    std::string fileHeaderAllICubes = "time,durationExp,iCubeNumber,numberFacesTouched,face_0,face_1,face_2,face_3,face_4,face_5,pose";

    int numberOfObjectsL, numberOfObjectsR;

    typedef struct BlobsImage_{
        std::string color;
        int16_t topLeftX_leftCam;
        int16_t topLeftY_leftCam;
        int16_t bottomRightX_leftCam;
        int16_t bottomRightY_leftCam;
        int16_t topLeftX_rightCam;
        int16_t topLeftY_rightCam;
        int16_t bottomRightX_rightCam;
        int16_t bottomRightY_rightCam;
    }BlobsImage;

    std::vector<BlobsImage> imageBlobs_bothCameras;

    yarp::os::Bottle* inputBlobsListL;
    yarp::os::Bottle* inputBlobsListR;

    //Skin Perception
    enum bodyPart {torso, leftHand, leftForearm, leftUpper, rightHand, rightForearm, rightUpper, noTouch};
    enum bodySide {torsoS, leftS, rightS, noTouchS};

    bodyPart originOfTouch;
    bodySide sideOfTouch;
    float touch_current, touch_prev, touchStabilityFlag;
    
    //Battery Perception
    double batteryLevel;

    //iCube Perception
    yarp::os::Bottle* dataAllCubes;

    bool eyesOpen;

    //OpenFace (affect) Perception
    double faceXInput, faceYInput, faceDepthInput, faceCertInput, focusX, focusY;
    bool gazeInput;
    int faceSuccessInput;
    std::string affectInput;
    float gaze_current, gaze_prev, face_current, face_prev;

    //Constants that can be changed according to each experiment. The default values are for Sara's experiments
    const float DISTANT = 0.25;//0.0;
    const float DISGUSTED =  0.35;//0.25;
    const float FROWNING =  0.35;//0.25;
    const float NEUTRAL = 0.6;//0.5;
    const float CONTEMPLATING = 0.85;//0.75;
    const float SMILING = 1.0;
    const float NOFACE = -1.0;

    const float minThresholdFace = -0.1;
    const float maxThresholdFace = 0.1;
    const float gazeMaxActivation = 1.0;
    const float minThresholdGaze = 0.3;//0.5;
    const float touchMaxActivation = 1.0;
    const float minThresholdTouch = 0.2;

    const float alpha_sensors = 0.9;//0.6

    //input ports
    yarp::os::BufferedPort<yarp::os::Bottle> inputPortBlobsListL;           //read objects from colorSegmentation from left camera    
    yarp::os::BufferedPort<yarp::os::Bottle> inputPortBlobsListR;           //read objects from colorSegmentation from right camera
    yarp::os::BufferedPort<yarp::os::Bottle> inputSkinPort;                 //read the data from skin
    yarp::os::BufferedPort<yarp::os::Bottle> inputBatteryLevelPort;         //read the battery level from the simulated sensor
    yarp::os::BufferedPort<yarp::os::Bottle> inputAffectPort;               //read the affect from yarpOpenFace
    yarp::os::BufferedPort<yarp::os::Bottle> inputPortEyelids_icub;
    yarp::os::BufferedPort<yarp::os::Bottle> inputPortiCube;
    
    //output ports  
    yarp::os::BufferedPort<yarp::os::Bottle> outputBatteryPort;             //write the battery level
    yarp::os::BufferedPort<yarp::os::Bottle> outputAllObjectsSeen;          //write all the objects seen from both eyes
    yarp::os::BufferedPort<yarp::os::Bottle> outputGazeFaceSkinPort;        //write face_current, gaze_current, touch_current
    yarp::os::BufferedPort<yarp::os::Bottle> outputSkinPort;                //write originOfTouch, sideOfTouch
    yarp::os::BufferedPort<yarp::os::Bottle> outputAffectPort;              //write affectInput, focusX, focusY
    yarp::os::BufferedPort<yarp::os::Bottle> outputiCubesPort;              //write touchedFaces, facesBeingTouched, pose of each iCube

    std::string name;                                                                // rootname of all the ports opened by this thread
    
public:
    /**
    * constructor default
    */
    perceptionThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    //perceptionThread(std::string robotname,std::string configFile);
    perceptionThread(std::string robotname,yarp::os::ResourceFinder &_rf);
    /**
     * destructor
     */
    ~perceptionThread();

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
    void initAllVars();
    void writeAllOutputPorts();
    void saveData();
    void saveHeaders();
    
    //Functions related to object perception -- Can detect a variable amount of objects in the scene using both cameras or just one
    //Knows which objects were seen by each camera (just in a scenario with unique colorful objects)
    void perceptObject();
    void detectObjectsR();
    void printListOfObjectsR();
    void detectObjectsL();
    void printListOfObjectsL();
    void printListOfAllObjects();
    void saveDataObjects();

    //Functions related to skin touch perception
    bool readSkin();
    float perceptSkin();
    float processTactileStimuli();

    //Functions related to iCube perception
    void perceptICube();
    void saveDataICubes();
    
    //Functions related to the openFace perception
    void perceptAffect();
    float processFaceStimuli();
    float processGazeStimuli();
    void readAffectEvaluation();

    //Functions related to the battery level perception
    void perceptBattery();

    void iCub_eyesOpen();
};

#endif  //_PERCEPTION_PERIODTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------