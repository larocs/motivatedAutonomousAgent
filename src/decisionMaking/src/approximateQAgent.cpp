#include "iCub/approximateQAgent.h"
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>
#include <sstream>

using namespace std;

approximateQAgent::approximateQAgent(){
    //Default values
    alpha = 0.0001;  
    gamma = 0.9;    
    epsilon = 1.0;  
    epsilon_min = 0.01;
    epsilon_decay = 0.0003;
    eGreedyDecay = EGREEDY_DECAY_LINEAR;
}

approximateQAgent::approximateQAgent(double alpha_, double gamma_, double epsilon_, double epsilon_min_, double epsilon_decay_, string eGreedyDecay_, int episodes_){
    //learning parameters
    alpha = alpha_; 
    gamma = gamma_;

    //exploration parameters
    epsilon = epsilon_;
    epsilon_min = epsilon_min_;
    epsilon_decay = epsilon_decay_;
    eGreedyDecay = eGreedyDecay_;
    
    episodes = episodes_;

    allEpsilon = new double[episodes];
    allEpsilon[0] = epsilon;
}

approximateQAgent::~approximateQAgent(){
    delete [] stateFeatures;
    delete [] featuresPerBehavior;
    delete [] allEpsilon;
    delete [] oldStateFeatures;
}

void approximateQAgent::setFilenamePath(string filepath){
    weightsFilename = filepath + weightsFilename;
    epsilonFilename = filepath + epsilonFilename;
}

double approximateQAgent::getRandomDouble(double lowerLimit, double upperLimit){
    std::uniform_real_distribution<double> dis(lowerLimit, upperLimit);//produces [lowerLimit, upperLimit)
    double random = dis(rdx);
    return random;
}

int approximateQAgent::getRandomInt(int lowerLimit, int upperLimit){
    std::uniform_int_distribution<int> dis(lowerLimit, upperLimit);//produces [lowerLimit, upperLimit]
    int random = dis(rdx);
    return random;
}

void approximateQAgent::setEpsilon(double epsilon_){
    epsilon = epsilon_;

    allEpsilon[0] = epsilon_;//This line is just to save the file with the correct value of epsilon in the testing phase

    cout<<"epsilon: "<<epsilon<<endl;
}

bool approximateQAgent::recoverWeights(){
    ifstream fin;
    fin.open(weightsFilename); 
    string myline;
    
    if(fin.is_open()){
        int i = 0;
        while(fin){    
            getline(fin, myline);
            istringstream iss(myline);
            string token;
            while(getline(iss, token, ',')){
                cout<<token<< " ";
                featuresPerBehavior[i] = stod(token);
                i++;
            }
        }
    }else{
        cout<<"Couldn't open the "<<weightsFilename<<" file"<<endl;
        return false;
    }

    return true;
}

void approximateQAgent::saveWeights(){
    ofstream fout;
    fout.open(weightsFilename); 

    for(int i = 0; i < total_behaviors * total_featuresState; i++)
            fout << to_string(featuresPerBehavior[i]) <<",";
    fout <<"\n";
    fout.close();
}

void approximateQAgent::saveEpsilonData(){
    ofstream fout;
    fout.open(epsilonFilename); 

    fout<<"epsilon\n";

    for(int i = 0; i < episodes; i++)
        fout<<to_string(allEpsilon[i])<<"\n";

    fout.close();
}

//https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution 
void approximateQAgent::init_featuresWeight(){
    for(int i = 0; i < total_behaviors * total_featuresState; i++)
        featuresPerBehavior[i] = getRandomDouble(0.001, 0.009);
    saveWeights();
}

void approximateQAgent::setTotalBehaviors(int behaviors){
    total_behaviors = behaviors;
    cout<<"Total Behaviors: "<<total_behaviors<<endl;
}

void approximateQAgent::setNumberOfFeatures(int features_, int previousStatesToRepeat_, int useActionDone){
    features = features_;
    previousStatesToRepeat = previousStatesToRepeat_;
    useLastBehavior = useActionDone;
    cout<<"Total Features to Repeat: "<<features<<endl;

    total_featuresState = features * (previousStatesToRepeat + 1) + useLastBehavior;
    stateFeatures = new double[total_featuresState];

    oldStateFeatures = new double[total_featuresState];

    featuresPerBehavior = new double[total_behaviors * total_featuresState];
}

void approximateQAgent::setFeatures(double newFeatures[], int actionDone){
    int i = features * previousStatesToRepeat;
    for(int j = 0; i < total_featuresState - useLastBehavior; i++, j++)
        stateFeatures[i] = newFeatures[j];

    if(useLastBehavior)
        stateFeatures[i] = actionDone;
}

void approximateQAgent::setFeaturesOldState(){
    for(int i = 0; i < total_featuresState; i++)
        oldStateFeatures[i] = stateFeatures[i];
}

/*
State   -   Time
S       =   T0
S'      =   T1
S''     =   T2
stateFeatures = S+S'+S''+A
*/
//Update the states. S = S'; S' = S''; S'' = newest Data
void approximateQAgent::shiftStateFeatures(){
    for(int i = 0; i < features * previousStatesToRepeat; i++)
        stateFeatures[i] = stateFeatures[i + features];
}

//https://stackoverflow.com/questions/3473438/return-array-in-a-function
double* approximateQAgent::getFeatures(int state_S_or_SL){
     if(state_S_or_SL == 0)
        return oldStateFeatures;
    else
        return stateFeatures;
}

double approximateQAgent::getQvalue(int actionIndex, int state_S_or_SL){
    double qValue = 0;
    double *features = getFeatures(state_S_or_SL);

    for(int i = 0; i < total_featuresState; i++)
        qValue += features[i] * featuresPerBehavior[actionIndex * total_featuresState + i];

    return qValue;
}

double approximateQAgent::getMaxQValue(){
    double maxQinSL, value;

    maxQinSL = getQvalue(0, 1);//I assumed that the first is the best

    for(int i = 1; i < total_behaviors; i++){
        value = getQvalue(i, 1);
        if(value > maxQinSL)
            maxQinSL = value;
    }
    return maxQinSL;
}

void approximateQAgent::update(int actionIdid, double reward){
    double Q_sa, Max_Qsl, TD_target;
    cout<<"update "<<endl;
    Q_sa = getQvalue(actionIdid, 0);
    Max_Qsl = getMaxQValue();
    TD_target = reward + gamma * Max_Qsl;

    //cout<<"Qsa: "<<Q_sa<<"   Max_Qsl: "<<Max_Qsl<<"    TD_target: "<<TD_target<<"   actionIdid: "<<actionIdid<<endl;

    //for(int i = 0; i < total_featuresState; i++)
        //cout<<"featuresPerBehavior_BEFORE: "<<featuresPerBehavior[actionIdid * total_featuresState + i]<<endl;

    for(int i = 0; i < total_featuresState; i++)
        featuresPerBehavior[actionIdid * total_featuresState + i] += alpha * ((TD_target - Q_sa) * stateFeatures[i]);

    //for(int i = 0; i < total_featuresState; i++)
        //cout<<"featuresPerBehavior_AFTER: "<<featuresPerBehavior[actionIdid * total_featuresState + i]<<endl;
}

void approximateQAgent::updateEpsilon(int episodesDone, int totalEpisodes){
    if(eGreedyDecay.compare(EGREEDY_DECAY_LINEAR) == 0)
        epsilon = max(epsilon_min, epsilon - (1.0/totalEpisodes));
    
    else if(eGreedyDecay.compare(EGREEDY_DECAY_EXPONENTIAL) == 0)
        epsilon = max(epsilon_min, epsilon * (1.0 - epsilon_decay));
    //else if(eGreedyDecay.compare(EGREEDY_DECAY_CONSTANT) == 0)
        //epsilon = epsilon;

    allEpsilon[episodesDone - 1] = epsilon;
    cout<<"epsilon: "<<epsilon<<endl;   
}

int approximateQAgent::getAction(){
    int action, selectedAction;
    double maxQinSL;

    if(getRandomDouble(0, 1) < epsilon){
        //exploration, random choice
        action = getRandomInt(0, total_behaviors - 1);//-1 cause produces [lowerLimit, upperLimit]
        selectedAction = action;
    }else{
        //exploitation, max value for given state
        selectedAction = 0; //Assume that the first is the better
        maxQinSL = getQvalue(0, 1);
        double value;
        for(int i = 1; i < total_behaviors; i++){
            value = getQvalue(i, 1);
            if(value > maxQinSL){
                maxQinSL = value;
                selectedAction = i;
            }
        }
    }
    return selectedAction;
}