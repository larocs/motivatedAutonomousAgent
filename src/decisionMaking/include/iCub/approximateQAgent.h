#ifndef _APPROXIMATEQAGENT_H_
#define _APPROXIMATEQAGENT_H_

#include <cstring>
#include <fstream>
#include <random>
#include <iostream>
#include <string.h>

#define EGREEDY_DECAY_LINEAR         "linear"
#define EGREEDY_DECAY_EXPONENTIAL    "exponential"
#define EGREEDY_DECAY_CONSTANT       "constant"

class approximateQAgent{
    private:
        double alpha;                   //learning rate
        double gamma;                   //discount factor
        double epsilon;                 //exploration probability at start
        double epsilon_min;             //minimum exploration probability
        double epsilon_decay;           //exponential decay rate for exploration prob
        std::string eGreedyDecay;       //epsilon decay: ['linear', 'exponential', 'constant']

        int total_behaviors;                    //Number of behaviors the robot can have (specified in Decision Making)
        int previousStatesToRepeat;             //Number of previous state to be used to compose the robot state in the Qlearning
        int features;                           //number of features that is repeated for each state (and the previous ones)
        int useLastBehavior;                    //if the last behavior is used or not to compose the state on MDP
        int total_featuresState;

        int episodes;

        double *stateFeatures;  //State used in MDP -- it is composed of the totalFeatures of the oldestState used to the newest one (from left to right)
        double *oldStateFeatures;
        double *featuresPerBehavior;    //weights associated to each feature for each action/behavior

        double *allEpsilon;     //save all the epsilon values during training phase -- used just to check if the epsilon decrease is right

        std::string weightsFilename = "weights_FeaturesPerAction.csv";
        std::mt19937 rdx{static_cast<long unsigned int>(21)};

        std::string epsilonFilename = "epsilonValues.csv";

    public:
        approximateQAgent();
        approximateQAgent(double alpha, double gamma, double epsilon, double epsilon_min, double epsilon_decay, std::string glie, int episodes);

        ~approximateQAgent();

        bool recoverWeights();
        void saveWeights();
        void init_featuresWeight();

        void setEpsilon(double epsilon);
        void updateEpsilon(int episodesDone, int totalEpisodes);
        void saveEpsilonData();

        void setTotalBehaviors(int behaviors);
        void setNumberOfFeatures(int features_, int previousStatesToRepeat_, int useActionDone);
        
        void setFeatures(double newFeatures[], int actionDone);
        void setFeaturesOldState();
        double* getFeatures(int state_S_or_SL);
        void shiftStateFeatures();
        
        double getQvalue(int actionIndex, int state_S_or_SL);
        double getMaxQValue();
        
        void update(int actionIdid, double reward);
 
        int getAction();
        
        double getRandomDouble(double lowerLimit, double upperLimit);
        int getRandomInt(int lowerLimit, int upperLimit);

        void setFilenamePath(std::string filepath);
};

#endif  //_APPROXIMATEQAGENT_H_